# The GPS logger's ESP32-C6 board (16 MB flash): MicroPython's
# ESP32_GENERIC_C6 with two OTA app slots (partitions-8MiBplus-ota.csv) for
# firmware updates over WiFi. Built by tools/build_firmware.sh
# (make BOARD_DIR=boards/GPS_LOGGER_C6).
include(boards/mpconfigboard_esp32c6_common.cmake)

list(APPEND SDKCONFIG_DEFAULTS
    boards/sdkconfig.csi
    ${MICROPY_BOARD_DIR}/sdkconfig.board
)
