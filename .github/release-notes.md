Auto-built firmware, rebuilt on every push to `main`. The download link never changes and there are no version numbers — whatever is here is the newest build. This release is updated in place; it never piles up old builds.

## Flash it

Download the files for your chip: `esp32s3` assumes 8MB flash, `esp32p4` assumes 32MB flash (other flash sizes need a build from source — see the README).

**Fresh board — merged image (bootloader + partition table + app in one file):**

```bash
esptool.py --chip esp32s3 -p <PORT> write_flash 0x0 usbipdcpp_esp32s3_merged.bin
esptool.py --chip esp32p4 -p <PORT> write_flash 0x0 usbipdcpp_esp32p4_merged.bin
```

Writing from `0x0` erases the NVS partition, so any WiFi credentials stored there are wiped — configure WiFi after boot through the web UI (`http://<device-ip>/`) or the serial console (see "Management & Configuration" in the README).

**Upgrade only the application — keeps stored WiFi credentials:**

```bash
esptool.py --chip esp32s3 -p <PORT> write_flash 0x10000 usbipdcpp_esp32s3_app.bin
esptool.py --chip esp32p4 -p <PORT> write_flash 0x10000 usbipdcpp_esp32p4_app.bin
```

**Or write the separate parts manually** (same layout `idf.py flash` uses):

```bash
esptool.py --chip esp32s3 -p <PORT> write_flash \
  0x0 usbipdcpp_esp32s3_bootloader.bin \
  0x8000 usbipdcpp_esp32s3_partition-table.bin \
  0x10000 usbipdcpp_esp32s3_app.bin
```
