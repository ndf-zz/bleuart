#!/bin/sh
# SPDX-License-Identifier: MIT
#
# Prepare a blank ESP32-WROOM-32E for use with bleuart:
#
#   - no flash encryption
#   - factory app + 2 OTA partitions
#   - secure boot (reflashable)
set -e
if [ -n "${ESP_IDF_VERSION}" ] ; then
  echo "IDF ${ESP_IDF_VERSION} detected"
else
  echo "IDF Env not detected, run export"
  exit
fi

idf.py erase-flash  # will fail if device already burned
idf.py bootloader
espefuse.py burn_key secure_boot_v1 build/bootloader/secure-bootloader-key-256.bin
esptool.py --chip esp32 --before=default_reset --after=no_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x1000 build/bootloader/bootloader.bin
idf.py flash monitor
echo Re-flash bootloader to correct re-flashable digest
