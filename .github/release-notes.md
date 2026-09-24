Auto-built firmware, rebuilt on every push to `main`. The download link never changes and there are no version numbers — whatever is here is the newest build.

## Flash it

Download the file matching your chip and write it at offset 0:

```bash
esptool.py --chip esp32s3 -p <PORT> write_flash 0x0 usbipdcpp_esp32s3_merged.bin
esptool.py --chip esp32p4 -p <PORT> write_flash 0x0 usbipdcpp_esp32p4_merged.bin
```

The merged image contains the bootloader, the partition table and the application in a single file.

## Notes

- `esp32s3` assumes 8MB flash and `esp32p4` assumes 32MB flash. For other flash sizes, build from source (see the README).
- Writing from `0x0` erases the NVS partition, so WiFi credentials stored there are wiped. Configure WiFi after boot through the web UI (`http://<device-ip>/`) or the serial console — see the "Management & Configuration" section of the README.
