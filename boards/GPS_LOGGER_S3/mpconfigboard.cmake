# The Heltec Wireless Tracker (ESP32-S3FN8: 8 MB flash, no PSRAM, UC6580
# GNSS on the board): MicroPython's ESP32_GENERIC_S3 without BLE and SPIRAM
# (mpconfigboard.h drops the rest the logger never uses) and with two 1792 KB
# OTA app slots (partitions-ota.csv) for firmware updates over WiFi. Built by
# tools/build_firmware.sh (BOARD_DIR=boards/GPS_LOGGER_S3).
set(IDF_TARGET esp32s3)

# the partition table lives here, outside the port: its path into a
# generated defaults file (sdkconfig paths are relative to the port)
set(GPS_PT ${CMAKE_BINARY_DIR}/sdkconfig.gps_partitions)
file(WRITE ${GPS_PT}
    "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"${MICROPY_BOARD_DIR}/partitions-ota.csv\"\n")

set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.flash_qio_80m
    boards/sdkconfig.csi
    ${MICROPY_BOARD_DIR}/sdkconfig.board
    ${GPS_PT}
)
