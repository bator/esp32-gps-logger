// The Heltec Wireless Tracker. The name must keep "Heltec": deploy.sh tells the
// board by it.

#define MICROPY_HW_BOARD_NAME               "Heltec Wireless Tracker"
#define MICROPY_HW_MCU_NAME                 "ESP32S3"

// The USB-C socket is on the chip's USB Serial/JTAG (no TinyUSB CDC): the
// same port for the ROM download mode and the REPL.
#define MICROPY_HW_ENABLE_USBDEV            (0)
#define MICROPY_HW_ENABLE_UART_REPL         (0)

// The Python heap starts big enough. From 64 KB the logger with the display grew
// it by a second 64 KB (measured: 124 KB, of the IDF heap 104 KB left, 62 KB the
// largest block), and one TLS POST left that in pieces of 33-40 KB, under the 48 KB
// the next one needs. 96 KB once takes less of the IDF heap, and in one piece.
#define MICROPY_GC_INITIAL_HEAP_SIZE        (96 * 1024)

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
