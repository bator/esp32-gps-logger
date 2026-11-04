// The Heltec Wireless Tracker. The name must keep "Heltec" and the MCU name
// "ESP32S3": gps_logger.py, main.py and deploy.sh tell the board by them.

#define MICROPY_HW_BOARD_NAME               "Heltec Wireless Tracker"
#define MICROPY_HW_MCU_NAME                 "ESP32S3"

// The USB-C socket is on the chip's USB Serial/JTAG (no TinyUSB CDC): the
// same port for the ROM download mode and the REPL, as on the C6.
#define MICROPY_HW_ENABLE_USBDEV            (0)
#define MICROPY_HW_ENABLE_UART_REPL         (0)

// What the logger never uses: BLE, ESP-NOW, Ethernet, mDNS queries (its URLs
// are plain hosts), SD card, I2S. The mDNS responder stays: the page is
// opened as HOSTNAME.local.
#define MICROPY_PY_BLUETOOTH                (0)
#define MICROPY_PY_ESPNOW                   (0)
#define MICROPY_PY_NETWORK_LAN              (0)
#define MICROPY_PY_MACHINE_SDCARD           (0)
#define MICROPY_PY_MACHINE_I2S              (0)
#define MICROPY_HW_ENABLE_MDNS_QUERIES      (0)
#define MICROPY_HW_ENABLE_MDNS_RESPONDER    (1)
