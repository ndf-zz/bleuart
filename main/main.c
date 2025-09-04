// SPDX-License-Identifier: MIT

/*
 * BLEUART - BLE to RS232 converter w/ wifi FOTA
 *
 * Based on ESP-IDF examples: uart_echo, https_request, bleprf & power_save
 * 
 * IDF Example code Copyright (C) 2015-2023 Espressif Systems (Apache 2.0)
 * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/COPYRIGHT.html
 *
 * September 2025 ndf-zz@6-v.org
 */

#include "nvs_flash.h"
#include "esp_pm.h"
#include "esp_bt.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h"
#include "esp_wifi.h"
#include "driver/uart.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// prototypes
void ble_store_config_init(void);  // not in esp header
static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
		     struct ble_gatt_access_ctxt *ctxt, void *arg);
static void wifi_sta_do_disconnect(void);

// build constants
#define BLEUART_DLEN 30  // limit dname string length
#define BLEUART_CLEN 6  // limit cname string length
#define BLEUART_FLEN 98  // limit FOTA trigger length (32+64+sep)
#define BLEUART_ULEN (sizeof(CONFIG_BLEUART_FOTA_URL)+42)  // update URL len
#define BLEUART_PORT 2  // ESP32
#define BLEUART_TXD 17  // ESP32 UART 2 TX GPIO
#define BLEUART_RXD 16  // ESP32 UART 2 RX GPIO
//#define BLEUART_ADVMS 10000  // adv duration
#define BLEUART_ADVMS (BLE_HS_FOREVER)
#define BLEUART_IDLEMS 30000  // main loop idle time
#define BLEUART_RXQLEN 4  // RX event queue len @ 19200 only a few slots reqd
#define BLEUART_BLEN UART_HW_FIFO_LEN(BLEUART_PORT)  // UART RX buffer len
#define BLEUART_CKEY "cname"  // NVS key for CNAME
#define BLEUART_DKEY "dname"  // NVS key for DNAME
#define BLEUART_PKEY "pin"  // NVS key for pairing pin
#define BLEUART_WIFITRY 8  // WIFI connect attempts
#define BLEUART_WIFIMS 5000  // WIFI connect timeout
#define BLEUART_RFLAG (BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC \
	| BLE_GATT_CHR_F_READ_AUTHEN) //  | BLE_GATT_CHR_F_READ_AUTHOR)
#define BLEUART_WFLAG (BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC \
	| BLE_GATT_CHR_F_WRITE_AUTHEN) //  | BLE_GATT_CHR_F_WRITE_AUTHOR)
#define BLEUART_KEYFLAG (BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID)

// task notifcation flags
#define EVT_RESET 0  // stack reset
#define EVT_ADV_INIT (0x1<<0)  // adv init process complete
#define EVT_ADV_START (0x1<<1)  // adv started
#define EVT_CONN (0x1<<2)  // connection established
#define EVT_ADV_END (0x1<<3)  // adv ended
#define EVT_FOTA (0x1<<4)  // request FOTA
#define EVT_REBOOT (0x1<<7)  // request reboot

// globals
static const char *TAG = "BU";  // log and netif ID
static esp_pm_lock_handle_t sleep_lock;  // light sleep inhibit lock
static QueueHandle_t uart_evq;  // RX event queue
static uint8_t own_addr_type;  // adv_init
static uint8_t addr_val[6];  // adv_init
static TaskHandle_t main_task;  // main
static TaskHandle_t rx_task;  // uart receive task
static esp_netif_t *sta_netif = NULL;  // ota netif
static SemaphoreHandle_t sem_ipv4 = NULL;  // IPv4 assigned
static SemaphoreHandle_t sem_ipv6 = NULL;  // IPv6 assigned
static int wifitry = 0;  // wifi sta connect attempts
static const char *ipv6_addr_types_to_str[6] = {
	"?", "G", "LL", "S", "U", "46"
};

// BLEUART service ID
static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(
	0xd9, 0x07, 0x59, 0xcb, 0x57, 0x8a, 0x6b, 0x95,
	0x47, 0x46, 0xc7, 0x99, 0xa4, 0xb0, 0xe6, 0x11
);

// pin 0x0000...
static const ble_uuid128_t pin_uuid = BLE_UUID128_INIT(
	0xd9, 0x07, 0x59, 0xcb, 0x57, 0x8a, 0x6b, 0x95,
	0x47, 0x46, 0xc7, 0x99, 0xa4, 0xb0, 0x00, 0x00
);
static uint16_t pin_handle;
static uint32_t pin = 0;  // ble access key

// UART "RX" 0x0001...
static const ble_uuid128_t rx_uuid = BLE_UUID128_INIT(
	0xd9, 0x07, 0x59, 0xcb, 0x57, 0x8a, 0x6b, 0x95,
	0x47, 0x46, 0xc7, 0x99, 0xa4, 0xb0, 0x01, 0x00
);
static uint8_t rx[BLEUART_BLEN];
static int rx_len;
static uint16_t rx_handle;
static uint16_t rx_conn_handle;
static bool rx_conn = false;
static bool rx_ind = false;

// UART "TX" 0x0010...
static const ble_uuid128_t tx_uuid = BLE_UUID128_INIT(
	0xd9, 0x07, 0x59, 0xcb, 0x57, 0x8a, 0x6b, 0x95,
	0x47, 0x46, 0xc7, 0x99, 0xa4, 0xb0, 0x10, 0x00
);
static uint16_t tx_handle;

// Unit identifier "CNAME" 0x0100...
static const ble_uuid128_t cname_uuid = BLE_UUID128_INIT(
	0xd9, 0x07, 0x59, 0xcb, 0x57, 0x8a, 0x6b, 0x95,
	0x47, 0x46, 0xc7, 0x99, 0xa4, 0xb0, 0x00, 0x01
);
static uint16_t cname_handle;
static char cname[BLEUART_CLEN+2];
static uint8_t manuf[BLEUART_CLEN+2];

// Device identifier "DNAME" 0x1000...
static const ble_uuid128_t dname_uuid = BLE_UUID128_INIT(
	0xd9, 0x07, 0x59, 0xcb, 0x57, 0x8a, 0x6b, 0x95,
	0x47, 0x46, 0xc7, 0x99, 0xa4, 0xb0, 0x00, 0x10
);
static uint16_t dname_handle;
static char dname[BLEUART_DLEN+2];

// Firmware revision characateristic 0x2a26
static const ble_uuid16_t fwrev_uuid = BLE_UUID16_INIT(0x2a26);
const esp_app_desc_t *app_desc;
static uint16_t fwrev_handle;

// Firmware update trigger 0xf07a...
static const ble_uuid128_t fota_uuid = BLE_UUID128_INIT(
	0xd9, 0x07, 0x59, 0xcb, 0x57, 0x8a, 0x6b, 0x95,
	0x47, 0x46, 0xc7, 0x99, 0xa4, 0xb0, 0x7a, 0xf0
);
static uint16_t fota_handle;
static char fota[BLEUART_FLEN+2];

// GATT service table
static const struct ble_gatt_svc_def gatt_svcs[] = {
	{.type = BLE_GATT_SVC_TYPE_PRIMARY,
	 .uuid = &svc_uuid.u,
	 .characteristics = (struct ble_gatt_chr_def[]) {
		{.uuid = &pin_uuid.u,
		 .access_cb = gatt_access,
		 .flags = BLEUART_WFLAG,
		 .val_handle = &pin_handle
		},
		{.uuid = &rx_uuid.u,
		 .access_cb = gatt_access,
		 .flags = BLEUART_RFLAG | BLE_GATT_CHR_F_NOTIFY,
		 .val_handle = &rx_handle
		},
		{.uuid = &tx_uuid.u,
		 .access_cb = gatt_access,
		 .flags = BLEUART_WFLAG,
		 .val_handle = &tx_handle
		},
		{.uuid = &cname_uuid.u,
		 .access_cb = gatt_access,
		 .flags = BLEUART_RFLAG | BLEUART_WFLAG,
		 .val_handle = &cname_handle
		},
		{.uuid = &dname_uuid.u,
		 .access_cb = gatt_access,
		 .flags = BLEUART_WFLAG,
		 .val_handle = &dname_handle
		},
		{.uuid = &fwrev_uuid.u,
		 .access_cb = gatt_access,
		 .flags = BLEUART_RFLAG,
		 .val_handle = &fwrev_handle
		},
		{.uuid = &fota_uuid.u,
		 .access_cb = gatt_access,
		 .flags = BLEUART_WFLAG,
		 .val_handle = &fota_handle
		},
		{ 0 },
	 },
	},
	{ 0 },
};

// Save whole dname or cname buffer to NVS
static void save_name(const char *key, char *val, size_t len)
{
	nvs_handle_t nvsh;
	esp_err_t rc = nvs_open(TAG, NVS_READWRITE, &nvsh);
	ESP_LOGE(TAG, "nvs open=%d", rc);
	if (rc == ESP_OK) {
		rc = nvs_set_blob(nvsh, key, val, len);
		nvs_commit(nvsh);
		nvs_close(nvsh);
		ESP_LOGI(TAG, "sn: %s[%d] %d", key, len, rc);
	}
}

static void save_pin(void)
{
	nvs_handle_t nvsh;
	if (nvs_open(TAG, NVS_READWRITE, &nvsh) == ESP_OK) {
		esp_err_t rc = nvs_set_u32(nvsh, BLEUART_PKEY, pin);
		nvs_commit(nvsh);
		nvs_close(nvsh);
		ESP_LOGI(TAG, "sp: '%lu' %d", pin, rc);
	}
	
}

static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
			  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	int rc;
	size_t len;
	ESP_LOGI(TAG,"ga: %d,%d", attr_handle, ctxt->op);
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR:
		if (attr_handle == rx_handle) {
			rc = os_mbuf_append(ctxt->om, &rx[0], rx_len);
			ESP_LOGI(TAG,"rx: X>B %d %d", rx_len, rc);
			return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
		} else if (attr_handle == cname_handle) {
			rc = os_mbuf_append(ctxt->om, &cname[0], BLEUART_CLEN);
			return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
		} else if (attr_handle == fwrev_handle) {
			rc = os_mbuf_append(ctxt->om, &(app_desc->version)[0],
						strlen(app_desc->version));
			return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		break;
	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		len = ctxt->om->om_len;
		if (attr_handle == tx_handle) {
			ESP_LOGI(TAG, "tx: B>U %d", ctxt->om->om_len);
			uart_write_bytes(BLEUART_PORT,
					 (const char *)ctxt->om->om_data,
					 ctxt->om->om_len);
			return 0;
		} else if (attr_handle == cname_handle) {
			if (len > BLEUART_CLEN) {
				len = BLEUART_CLEN;
			}
			memset(&cname[0], 0x00, BLEUART_CLEN);
			memcpy(&cname[0], ctxt->om->om_data, len);
			save_name(BLEUART_CKEY, &cname[0], BLEUART_CLEN);
			return 0;
		} else if (attr_handle == dname_handle) {
			if (len > BLEUART_DLEN) {
				len = BLEUART_DLEN;
			}
			memset(&dname[0], 0x00, BLEUART_DLEN);
			memcpy(&dname[0], ctxt->om->om_data, len);
			save_name(BLEUART_DKEY, &dname[0], BLEUART_DLEN);
			ble_svc_gap_device_name_set(dname);
			return 0;
		} else if (attr_handle == pin_handle) {
			if (len == 4) {
				pin = ctxt->om->om_data[0]&0xff;
				pin |= (ctxt->om->om_data[1]<<8)&0xff00;
				pin |= (ctxt->om->om_data[2]<<16)&0xff0000;
				pin |= (ctxt->om->om_data[3]<<24)&0xff000000;
				if (pin > 999999) {
					pin = 999999;
				}
				save_pin();
			} else {
				ESP_LOGE(TAG, "p: bad");
				return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
			}
			return 0;
		} else if (attr_handle == fota_handle) {
			if (len > BLEUART_FLEN) {
				len = BLEUART_FLEN;
			}
			memset(&fota[0], 0x00, BLEUART_FLEN);
			memcpy(&fota[0], ctxt->om->om_data, len);
			// notify main proc
			xTaskNotify(main_task, EVT_FOTA,
					eSetValueWithOverwrite);
			return 0;
		}
		break;
	default:
		break;
	}
	return BLE_ATT_ERR_UNLIKELY;
}

void subscribe_cb(struct ble_gap_event *event)
{
	if (event->subscribe.attr_handle == rx_handle) {
		rx_conn_handle = event->subscribe.conn_handle;
		rx_conn = true;
		rx_ind = event->subscribe.cur_notify;
		if (rx_ind) {
			esp_pm_lock_acquire(sleep_lock);
			ESP_LOGI(TAG, "ls: a");
		}
	}
}

static void format_addr(char *addr_str, uint8_t addr[])
{
	sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[5], addr[4],
		addr[3], addr[2], addr[1], addr[0]);
}

static void log_conn(struct ble_gap_conn_desc *desc)
{
	char addr_str[18] = {0};
	format_addr(addr_str, desc->peer_id_addr.val);
	ESP_LOGI(TAG, "cn: %d %s %d,%d,%d",
		desc->conn_handle, addr_str, desc->sec_state.encrypted,
		desc->sec_state.authenticated, desc->sec_state.bonded);
}

static void update_manuf(void)
{
	int mid = CONFIG_BLEUART_MID;
	manuf[0] = (uint8_t)(mid & 0xff);
	manuf[1] = (uint8_t)((mid >> 8) & 0xff);
	memcpy((char *)&manuf[2], &cname[0], BLEUART_CLEN);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
	int rc = 0;
	struct ble_gap_conn_desc desc;

	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		ESP_LOGW(TAG, "ce: %d", event->connect.status);
		ble_gap_conn_find(event->connect.conn_handle, &desc);
		log_conn(&desc);
		//struct ble_gap_upd_params params = {
			//.itvl_min = desc.conn_itvl,
			//.itvl_max = desc.conn_itvl,
			//.latency = 64,
			//.supervision_timeout = desc.supervision_timeout
		//};
		//ble_gap_update_params(event->connect.conn_handle, &params);
		xTaskNotify(main_task, EVT_CONN, eSetBits);
		break;
	case BLE_GAP_EVENT_DISCONNECT:
		ESP_LOGW(TAG, "dc: %d", event->disconnect.reason);
		rx_conn = false;
		rx_conn_handle = 0;
		esp_pm_lock_release(sleep_lock);
		ESP_LOGI(TAG, "ls: re");
		xTaskNotify(main_task, EVT_ADV_INIT, eSetValueWithOverwrite);
		break;
	case BLE_GAP_EVENT_CONN_UPDATE:
		ESP_LOGI(TAG, "cu: %d", event->conn_update.status);
		ble_gap_conn_find(event->conn_update.conn_handle, &desc);
		log_conn(&desc);
		break;
	case BLE_GAP_EVENT_ADV_COMPLETE:
		ESP_LOGI(TAG, "ac: %d", event->adv_complete.reason);
		xTaskNotify(main_task, EVT_ADV_END, eSetBits);
		break;
	case BLE_GAP_EVENT_SUBSCRIBE:
		ESP_LOGI(TAG, "gs: %d n: %d>%d i: %d>%d",
			 event->subscribe.attr_handle,
			 event->subscribe.prev_notify,
			 event->subscribe.cur_notify,
			 event->subscribe.prev_indicate,
			 event->subscribe.cur_indicate);
		subscribe_cb(event);
		break;
	case BLE_GAP_EVENT_NOTIFY_TX:
		xTaskNotify(rx_task, event->notify_tx.status,
				eSetValueWithOverwrite);
		break;
	case BLE_GAP_EVENT_ENC_CHANGE:
		ble_gap_conn_find(event->enc_change.conn_handle, &desc);
		log_conn(&desc);
		break;
	case BLE_GAP_EVENT_PASSKEY_ACTION:
		struct ble_sm_io pkey = {0};
		if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
			pkey.action = event->passkey.params.action;
			pkey.passkey = pin;
			ESP_LOGW(TAG, "pk: %lu", pkey.passkey);
		}
		rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
		break;
	case BLE_GAP_EVENT_AUTHORIZE:
		ESP_LOGI(TAG, "au: %d %d",
		    event->authorize.conn_handle,
		    event->authorize.attr_handle,
		    event->authorize.is_read);
		event->authorize.out_response = BLE_GAP_AUTHORIZE_ACCEPT;
		break;
	case BLE_GAP_EVENT_REPEAT_PAIRING:
		ESP_LOGI(TAG, "rp: %d", event->repeat_pairing.conn_handle);
		ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
		ble_store_util_delete_peer(&desc.peer_id_addr);
		return BLE_GAP_REPEAT_PAIRING_RETRY;
		break;
	case BLE_GAP_EVENT_IDENTITY_RESOLVED:
		ESP_LOGI(TAG, "ir");
		break;
	default:
		ESP_LOGI(TAG, "ge: %d", event->type);
		break;
	}
	return rc;
}

static void start_advertising(void)
{
	struct ble_hs_adv_fields adv_fields = {0};
	struct ble_hs_adv_fields rsp_fields = {0};
	struct ble_gap_adv_params adv_params = {0};

	// fill and set adv fields
	update_manuf();
	adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	adv_fields.tx_pwr_lvl = 9;
	adv_fields.tx_pwr_lvl_is_present = 1;
	adv_fields.appearance = CONFIG_BLEUART_APPEARANCE;
	adv_fields.appearance_is_present = 1;
	adv_fields.le_role = 0x00;
	adv_fields.le_role_is_present = 1;
	adv_fields.mfg_data = &manuf[0];
	adv_fields.mfg_data_len = BLEUART_CLEN + 2;
	ble_gap_adv_set_fields(&adv_fields);

	// fill and set rsp
	rsp_fields.device_addr = addr_val;
	rsp_fields.device_addr_type = own_addr_type;
	rsp_fields.device_addr_is_present = 1;
	rsp_fields.adv_itvl = BLE_GAP_ADV_FAST_INTERVAL2_MIN;
	rsp_fields.adv_itvl_is_present = 1;
	ble_gap_adv_rsp_set_fields(&rsp_fields);

	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL2_MIN;
	adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL2_MAX;
	ble_gap_adv_start(own_addr_type, NULL, BLEUART_ADVMS, &adv_params,
			       gap_event, NULL);
	xTaskNotify(main_task, EVT_ADV_START, eSetBits);
}

static void adv_init(void)
{
	ble_hs_util_ensure_addr(0);
	ble_hs_id_infer_auto(0, &own_addr_type);
	ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
	char addr_str[18] = {0};
	format_addr(addr_str, addr_val);
	ESP_LOGI(TAG, "oa: %s", addr_str);
	xTaskNotify(main_task, EVT_ADV_INIT, eSetBits);
}

static void on_stack_reset(int reason)
{
	xTaskNotify(main_task, EVT_RESET, eSetValueWithOverwrite);
}

static void ble_host(void *param)
{
	nimble_port_run();  // does not return in this case
}

static void read_uart(void)
{
	size_t avail = 0;
	uint32_t nv;

	uart_get_buffered_data_len(BLEUART_PORT, &avail);
	ESP_LOGI(TAG,"av: %d", avail);
	if (avail) {
		// pull all available data out of uart
		if (avail > BLEUART_BLEN) {
			avail = BLEUART_BLEN;
		}
		rx_len = uart_read_bytes(BLEUART_PORT, &rx[0],
			    avail, portMAX_DELAY);
		ESP_LOGI(TAG, "rx: U>X %d", rx_len);
		if (rx_len && rx_ind && rx_conn) {
			ble_gatts_notify(rx_conn_handle, rx_handle);
			xTaskNotifyWait(0, ULONG_MAX, &nv, portMAX_DELAY);
		}
	}
}

static void uart_receive(void)
{
	uart_event_t event;
	if (xQueueReceive(uart_evq, (void *)&event, portMAX_DELAY)) {
		switch (event.type) {
		case UART_DATA:
			read_uart();
			break;
		case UART_FIFO_OVF:
		case UART_BUFFER_FULL:
			ESP_LOGW(TAG, "ua: o/f");
			uart_flush(BLEUART_PORT);
			xQueueReset(uart_evq);
			break;
		default:
			ESP_LOGI(TAG, "ua: evt %d", event.type);
			break;
		}
	}
}

static void uart_rx(void *arg)
{
	do {
		uart_receive();
	} while (1);
}

static bool own_netif(const char *prefix, esp_netif_t *netif)
{
	return strncmp(prefix, esp_netif_get_desc(netif),
				 strlen(prefix) - 1) == 0;
}

static void on_wifi_disconnect(void *arg, esp_event_base_t event_base,
			       int32_t event_id, void *event_data)
{
	wifitry++;
	if (wifitry > BLEUART_WIFITRY) {
		ESP_LOGI(TAG, "wf: %d", wifitry);
		if (sem_ipv4) {
			xSemaphoreGive(sem_ipv4);
		}
		if (sem_ipv6) {
			xSemaphoreGive(sem_ipv6);
		}
		wifi_sta_do_disconnect();
		return;
	}
	wifi_event_sta_disconnected_t *disconn = event_data;
	if (disconn->reason == WIFI_REASON_ROAMING) {
		return;
	}
	ESP_LOGI(TAG, "wf: d/c %d", disconn->reason);
	esp_wifi_connect();
}

static void on_sta_got_ipv4(void *arg, esp_event_base_t event_base,
		      int32_t event_id, void *event_data)
{
	wifitry = 0;
	ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
	if (!own_netif(TAG, event->esp_netif)) {
		return;
	}
	ESP_LOGI(TAG, "v4: " IPSTR, IP2STR(&event->ip_info.ip));
	if (sem_ipv4) {
		xSemaphoreGive(sem_ipv4);
	}
}

static void on_sta_got_ipv6(void *arg, esp_event_base_t event_base, 
			int32_t event_id, void *event_data)
{
	ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
	if (!own_netif(TAG, event->esp_netif)) {
		return;
	}
	esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&event->ip6_info.ip);
	ESP_LOGI(TAG, "v6[%s]: " IPV6STR, ipv6_addr_types_to_str[ipv6_type],
		IPV62STR(event->ip6_info.ip));
	if (ipv6_type == ESP_IP6_ADDR_IS_GLOBAL) {
		if (sem_ipv6) {
			xSemaphoreGive(sem_ipv6);
		}
	}
}

static void on_wifi_connect(void *esp_netif, esp_event_base_t event_base,
			    int32_t event_id, void *event_data)
{
	esp_netif_create_ip6_linklocal(esp_netif);
}

static void wifi_sta_do_disconnect(void)
{
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
					&on_wifi_disconnect);
	esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
					 &on_sta_got_ipv4);
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,
					 &on_wifi_connect);
	esp_event_handler_unregister(IP_EVENT, IP_EVENT_GOT_IP6,
					 &on_sta_got_ipv6);
	esp_wifi_disconnect();
}

static void wifi_connect(char *ssid, char *psk)
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg)); 
	esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
	esp_netif_config.if_desc = TAG;
	esp_netif_config.route_prio = 128;
	sta_netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
	esp_wifi_set_default_wifi_sta_handlers();
	esp_wifi_set_storage(WIFI_STORAGE_RAM);
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_start();
	wifi_config_t wifi_config = {
	.sta = {
		.ssid = { 0 },
		.password =  { 0 },
		.scan_method = WIFI_ALL_CHANNEL_SCAN,
		.sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
		.threshold.rssi = -127,
		.threshold.authmode = WIFI_AUTH_WPA2_PSK,
		},
	};
	strncpy((char *)&wifi_config.sta.ssid[0], ssid, 32);
	strncpy((char *)&wifi_config.sta.password[0], psk, 64);
	sem_ipv4 = xSemaphoreCreateBinary();
	if (sem_ipv4 == NULL) {
		return;
	}
	sem_ipv6 = xSemaphoreCreateBinary();
	if (sem_ipv6 == NULL) {
		vSemaphoreDelete(sem_ipv4);
		return;
	}
	wifitry = 0;
	esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
					&on_wifi_disconnect, NULL);
	esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
					&on_sta_got_ipv4, NULL);
	esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,
					&on_wifi_connect, sta_netif);
	esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6,
					&on_sta_got_ipv6, NULL);
	esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
	esp_err_t ret = esp_wifi_connect();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "wc: %x", ret);
		return;
	}
	xSemaphoreTake(sem_ipv4, portMAX_DELAY);
	vSemaphoreDelete(sem_ipv4);
	sem_ipv4 = NULL;
	xSemaphoreTake(sem_ipv6, portMAX_DELAY);
	vSemaphoreDelete(sem_ipv6);
	sem_ipv6 = NULL;
}

static void start_fota(void *arg)
{
	esp_pm_lock_acquire(sleep_lock);
	char ssid[BLEUART_FLEN+2] = { 0 };
	char psk[BLEUART_FLEN+2] = { 0 };
	strcpy(&ssid[0], &fota[0]);
	int pind = strlen(ssid)+1;
	strcpy(&psk[0], &fota[pind]);
	char fota_url[BLEUART_ULEN];
	strncpy(&fota_url[0], CONFIG_BLEUART_FOTA_URL, BLEUART_ULEN);
	pind = strlen(fota_url);
	ble_uuid_to_str(&svc_uuid.u, &fota_url[pind]);
	pind = strlen(fota_url);
	strcpy(&fota_url[pind], ".bin");
	ESP_LOGI(TAG, "wc: '%s'", ssid);
	esp_netif_init();
	esp_event_loop_create_default();
	wifi_connect(&ssid[0], &psk[0]);
	ESP_LOGW(TAG, "fu: '%s'", fota_url);
	esp_http_client_config_t config = {
		.url = &fota_url[0],
		.crt_bundle_attach = esp_crt_bundle_attach,
		.keep_alive_enable = true,
		.timeout_ms = 10000,
		.user_agent = CONFIG_BLEUART_FOTA_AGENT,
		.addr_type = HTTP_ADDR_TYPE_INET6
	};
	esp_https_ota_config_t ota_config = {
		.http_config = &config,
	};
	esp_err_t ret = esp_https_ota(&ota_config);
	if (ret == ESP_OK) {
		ESP_LOGW(TAG, "fu: OK");
	} else {
		ESP_LOGW(TAG, "fu: fail");
	}
	esp_wifi_disconnect();
	esp_restart();
	vTaskDelete(NULL);
}

static void bu_pm_init(void)
{
	esp_pm_config_t pm_config = {
		.max_freq_mhz = 80,  // BLE requires >= 80 for stability
		.min_freq_mhz = 80,
		.light_sleep_enable = true
	};
	esp_pm_configure(&pm_config);
	esp_err_t rc = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, NULL,
						&sleep_lock);
	ESP_LOGI(TAG, "pm: %d", rc);
}

static void bu_nvs_init(void)
{
	strncpy(&dname[0], CONFIG_BLEUART_DNAME, BLEUART_DLEN);
	strncpy(&cname[0], CONFIG_BLEUART_CNAME, BLEUART_CLEN);
	if (nvs_flash_init()) {
		ESP_LOGW(TAG, "nv: erase");
		nvs_flash_erase();
		nvs_flash_init();
	} else {
		ESP_LOGI(TAG, "nv: init");
	}
	nvs_handle_t nvsh;
	esp_err_t rc = nvs_open(TAG, NVS_READONLY, &nvsh);
	ESP_LOGI(TAG, "nvs open rc=%d", rc);
	if (rc == ESP_OK) {
		size_t len = BLEUART_DLEN;
		nvs_get_blob(nvsh, BLEUART_DKEY, &dname[0], &len);
		len = BLEUART_CLEN;
		nvs_get_blob(nvsh, BLEUART_CKEY, &cname[0], &len);
		rc = nvs_get_u32(nvsh, BLEUART_PKEY, &pin);
		if (rc != ESP_OK) {
			pin = 0x00;
		}
		if (pin > 999999) {
			pin = 999999;
		}
		nvs_close(nvsh);
	}
	ESP_LOGI(TAG, "dn: '%s' cn: '%s' pk: '%lu'", dname, cname, pin);
}

static void bu_init_uart(void)
{
	uart_config_t uart_config = {
		.baud_rate = CONFIG_BLEUART_BAUD,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};
	esp_err_t rc = uart_driver_install(BLEUART_PORT,
			4*UART_HW_FIFO_LEN(BLEUART_PORT),
			2*UART_HW_FIFO_LEN(BLEUART_PORT), BLEUART_RXQLEN,
			&uart_evq, ESP_INTR_FLAG_IRAM);
	if (rc == ESP_OK) {
		uart_param_config(BLEUART_PORT, &uart_config);
		uart_set_pin (BLEUART_PORT, BLEUART_TXD, BLEUART_RXD,
			 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	}
	ESP_LOGI(TAG, "ua: %d", rc);
}

static void bu_ble_init(void)
{
	esp_err_t rc = nimble_port_init();
	if (rc == ESP_OK) {
		ble_svc_gap_init();
		ble_svc_gap_device_name_set(dname);
		ble_svc_gatt_init();
		ble_gatts_count_cfg(gatt_svcs);
		ble_gatts_add_svcs(gatt_svcs);
		ble_hs_cfg.reset_cb = on_stack_reset;
		ble_hs_cfg.sync_cb = adv_init;
		ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
		ble_hs_cfg.sm_sc = 1;
		ble_hs_cfg.sm_mitm = 1;
		ble_hs_cfg.sm_bonding = 1;
		ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
		ble_hs_cfg.sm_our_key_dist = BLEUART_KEYFLAG;
		ble_hs_cfg.sm_their_key_dist = BLEUART_KEYFLAG;
		ble_store_config_init();
		esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9); 
		esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
		esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN ,ESP_PWR_LVL_P9);
	}
	ESP_LOGI(TAG, "bl: %d", rc);
}

void app_main(void)
{
	//nvs_flash_erase();
	main_task = xTaskGetCurrentTaskHandle();
	app_desc = esp_app_get_description();
	bu_nvs_init();
	bu_pm_init();
	bu_ble_init();
	bu_init_uart();

	// start BLE and UART tasks - pinned to cpu 1
	xTaskCreatePinnedToCore(ble_host, "ble_host", 4096, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(uart_rx, "uart_rx", 2432, NULL, 3, &rx_task, 1);

	// manage BLE advertisement in a loop
	uint32_t ev;
	do {
		if (xTaskNotifyWait(0, 0, &ev,
				BLEUART_IDLEMS / portTICK_PERIOD_MS)) {
			ESP_LOGI(TAG, "mn: 0x%02lx", ev);
			if (ev == EVT_ADV_INIT) {
				start_advertising();
			} else if (ev & EVT_FOTA) {
				xTaskCreatePinnedToCore(start_fota, "fota",
						4096, NULL, 3, NULL, 1);
			}
		} else if (ev & EVT_ADV_END) {
			xTaskNotify(main_task, EVT_ADV_INIT,
						eSetValueWithOverwrite);
		}
	} while (1);
}
