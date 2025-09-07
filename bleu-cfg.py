#!/usr/bin/python3
#
# configure blueart device
#
#  usage: bleu-cfg [-h] [-p PIN] [-c CNAME] [-f CNAME] [-a ADDR] [-i]
#
#  options:
#    -h, --help          show this help message and exit
#    -p, --pin PIN       Update bonding pin (0-999999)
#    -c, --cname CNAME   Update device location (CNAME)
#    -f, --filter CNAME  Filter on location (CNAME)
#    -a, --addr ADDR     Connect to BLE device at ADDR
#    -i, --init          Replace device serial number
#
# save discovered bleuart devices, matched on manufacturer ID
# or address, to blue-cfg.csv

import asyncio
import sys
import argparse
from bleak import BleakClient, BleakScanner
import logging
from struct import pack
from secrets import choice
import os
import csv

DBFILENAME = 'bleu-cfg.csv'
MID = 0xffff
INITPIN = 0
CLEN = 6

_res = {
    'rc': -1,
    'msg': [],
}

gatt = {
    'dname': '2a00',
    'fwver': '2a26',
    'pin': '0000b0a4-99c7-4647-956b-8a57cb5907d9',
    'cname': '0100b0a4-99c7-4647-956b-8a57cb5907d9',
}

logging.basicConfig(level=logging.INFO)
_log = logging.getLogger('bleu-cfg')
_log.setLevel(logging.DEBUG)


async def connect(address=None, cname=None):
    """Find first matching device and return bleak device handle"""
    if address is not None:
        d = await BleakScanner.find_device_by_address(address)
        if d is not None:
            _log.debug('Found: %s %s', d.address, d.name)
            return d
    else:
        devices = await BleakScanner.discover(return_adv=True)
        for d, a in devices.values():
            if a.manufacturer_data is not None:
                if MID in a.manufacturer_data:
                    mfc = a.manufacturer_data[MID]
                    if cname is not None:
                        if mfc and mfc.startswith(cname):
                            _log.debug('Found: %s %s: %s', d.address, d.name,
                                       mfc)
                        return d
                    else:
                        _log.debug('Found: %s %s: %s', d.address, d.name, mfc)
                        return d
    _log.error('Device not found')
    return None


def load_db():
    """Read device history from DBFILENAME"""
    data = {}
    if os.path.exists(DBFILENAME):
        with open(DBFILENAME) as f:
            cr = csv.DictReader(f)
            for r in cr:
                key = r['addr']
                data[key] = {
                    'addr': key,
                    'serial': r['serial'],
                    'version': r['version'],
                    'dname': r['dname'],
                    'cname': r['cname'],
                    'pin': r['pin']
                }
            _log.debug('Loaded %d record%s from %s', len(data),
                       '' if len(data) == 1 else 's', DBFILENAME)
    return data


def save_db(data):
    """Save device history to DBFILENAME"""
    with open(DBFILENAME, 'w') as f:
        cw = csv.DictWriter(f,
                            fieldnames=('addr', 'serial', 'version', 'dname',
                                        'cname', 'pin'))
        cw.writeheader()
        for k, v in data.items():
            cw.writerow({
                'addr': k,
                'serial': v['serial'],
                'version': v['version'],
                'dname': v['dname'],
                'cname': v['cname'],
                'pin': v['pin'],
            })
        _log.debug('Saved %d record%s to %s', len(data),
                   '' if len(data) == 1 else 's', DBFILENAME)


async def read_and_update(db, d, serialno, pin, cname):
    """Read current values out of device and update if required"""
    if d.address not in db:
        db[d.address] = {
            'addr': d.address,
            'serial': None,
            'version': None,
            'dname': None,
            'cname': None,
            'pin': None,
        }
    _log.debug('Connecting to %s', d.address)
    async with BleakClient(d) as c:
        try:
            dbr = db[d.address]
            if serialno is not None:
                dbr['serial'] = serialno
            dname = await c.read_gatt_char(gatt['dname'])
            dbr['dname'] = dname.decode('utf-8', 'replace')
            _log.debug('%s Connect %r', d.address, bytes(dname))
            origcname = await c.read_gatt_char(gatt['cname'])
            dbr['cname'] = origcname.decode('utf-8', 'replace')
            fwver = await c.read_gatt_char(gatt['fwver'])
            dbr['version'] = fwver.decode('utf-8', 'replace')
            _log.info('%s CNAME=%r, Fwver=%r', d.address, bytes(origcname),
                      bytes(fwver))
            if pin is not None:
                pu = pack('<L', pin)
                _log.info('%s Update pin: %r (%d)', d.address, pu, pin)
                rc = await c.write_gatt_char(gatt['pin'], pu, True)
                _log.debug('%s Update pin rc=%r', d.address, rc)
                dbr['pin'] = pin
            if cname is not None:
                _log.info('%s Update CNAME: %r', d.address, cname)
                rc = await c.write_gatt_char(gatt['cname'], cname, True)
                _log.debug('%s Update CNAME rc=%r', d.address, rc)
                dbr['cname'] = cname.decode('utf-8', 'replace')
            _res['rc'] = 0
            _log.debug('%s Disconnect', d.address)
            await c.disconnect()
        except * EOFError:
            if _res['rc'] != 0 and not _res['msg']:
                _res['msg'].append('Error connecting to device')
            _log.debug('%s EOF', d.address)
            pass
        except * Exception as e:
            for pe in e.exceptions:
                _log.error('%s %s: %s', d.address, pe.__class__.__name__, pe)
                _res['msg'].append(str(pe))


async def main():
    parser = argparse.ArgumentParser(prog='bleu-cfg')
    parser.add_argument('-p',
                        '--pin',
                        default=None,
                        metavar='PIN',
                        help='Update bonding pin (0-999999)')
    parser.add_argument('-c',
                        '--cname',
                        default=None,
                        metavar='CNAME',
                        help='Update device location (CNAME)')
    parser.add_argument('-f',
                        '--filter',
                        default=None,
                        metavar='CNAME',
                        help='Filter on location (CNAME)')
    parser.add_argument('-a',
                        '--addr',
                        default=None,
                        metavar='ADDR',
                        help='Connect to BLE device at ADDR')
    parser.add_argument('-i',
                        '--init',
                        action='store_true',
                        help='Replace device serial number')
    args = parser.parse_args()
    pin = None
    if args.pin is not None:
        try:
            newpin = int(args.pin)
            pin = max(0, min(999999, newpin))
        except Exception as e:
            _log.warning('Ignored invalid pin')
    cname = None
    if args.cname is not None:
        cname = args.cname.encode('utf-8', 'replace')
        if len(cname) > CLEN:
            _log.warning('CNAME truncated')
            cname = cname[0:6]
    cfilter = None
    if args.filter is not None:
        cfilter = args.filter.encode('utf-8', 'replace')
        if len(cfilter) > CLEN:
            _log.warning('FILTER truncated')
            cfilter = cfilter[0:6]

    serialno = None
    if args.init:
        serialno = 'H%05d' % (choice(range(100000)), )
        if cname is None:
            cname = serialno.encode('utf-8', 'replace')
        if pin is None:
            pin = INITPIN
    _log.debug('Args: pin=%r, cname=%r, filter=%r, address=%r, init=%r', pin,
               cname, cfilter, args.addr, args.init)
    db = load_db()
    try:
        d = await connect(address=args.addr, cname=cfilter)
        if d is not None:
            await read_and_update(db, d, serialno, pin, cname)
        else:
            _res['rc'] = -2
            _res['msg'].append('BLEUART not found')
    except EOFError:
        _log.debug('EOF')
        pass
    except Exception as e:
        _res['rc'] = -1
        _log.error('Main %s: %s', e.__class__.__name__, e)
        _res['msg'].append(str(e))
    finally:
        save_db(db)


if __name__ == "__main__":
    asyncio.run(main())
    _log.debug('Exit: %d %r', _res['rc'], _res['msg'])
    sys.exit(_res['rc'])
