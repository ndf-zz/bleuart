#!/bin/sh
# SPDX-License-Identifier: MIT
# 
# Re-flash bleuart on an already burned bleuart module. Assumes:
#
#   - security key matches current build dir
#   - no flash encryption
#   - factory app + 2 OTA partitions
#   - secure boot V1 (reflashable)
set -e
if [ -n "${ESP_IDF_VERSION}" ] ; then
  echo "IDF ${ESP_IDF_VERSION} detected"
else
  echo "IDF Env not detected, run export"
  exit
fi

esptool.py --before default_reset --after no_reset --chip esp32 erase_flash --force
idf.py bootloader
esptool.py --chip esp32 --before=default_reset --after=no_reset write_flash --flash_mode dio --flash_freq 40m --flash_size 4MB 0x0 build/bootloader/bootloader-reflash-digest.bin --force
idf.py flash monitor
