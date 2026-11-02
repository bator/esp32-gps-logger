# The GPS logger's ESP32-C6 board (16 MB flash): MicroPython's
# ESP32_GENERIC_C6 without BLE (the port's BLE defaults left out, BT off in
# sdkconfig.board; mpconfigboard.h drops the rest the logger never uses)
# and with two 1792 KB OTA app slots (partitions-ota.csv) for firmware
# updates over WiFi. Built by tools/build_firmware.sh
# (make BOARD_DIR=boards/GPS_LOGGER_C6).
set(IDF_TARGET esp32c6)

# the partition table lives here, outside the port: its path into a
# generated defaults file (sdkconfig paths are relative to the port)
set(GPS_PT ${CMAKE_BINARY_DIR}/sdkconfig.gps_partitions)
file(WRITE ${GPS_PT}
    "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"${MICROPY_BOARD_DIR}/partitions-ota.csv\"\n")

set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.riscv
    boards/sdkconfig.c6
    boards/sdkconfig.csi
    ${MICROPY_BOARD_DIR}/sdkconfig.board
    ${GPS_PT}
)
