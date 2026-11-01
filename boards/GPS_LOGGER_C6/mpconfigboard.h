// The GPS logger's ESP32-C6 board: as ESP32_GENERIC_C6. The board name must
// keep "ESP32C6": gps_logger.py and deploy.sh tell the C6 by it.

#define MICROPY_HW_BOARD_NAME               "ESP32C6 module"
#define MICROPY_HW_MCU_NAME                 "ESP32-C6"

// Enable UART REPL for modules that have an external USB-UART and don't use native USB.
#define MICROPY_HW_ENABLE_UART_REPL         (1)
