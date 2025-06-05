#define MICROPY_HW_BOARD_NAME "Radxa X4"
#define MICROPY_HW_MCU_NAME "rp2040"

//no MICROPY_HW_LED_STATUS

#define CIRCUITPY_BOARD_I2C         (1)
#define CIRCUITPY_BOARD_I2C_PIN     {{.scl = &pin_GPIO29, .sda = &pin_GPIO28}}

#define CIRCUITPY_BOARD_SPI         (1)
#define CIRCUITPY_BOARD_SPI_PIN     {{.clock = &pin_GPIO10, \
                                      .mosi = &pin_GPIO11, \
                                      .miso = &pin_GPIO8}}

#define CIRCUITPY_BOARD_UART        (2)
#define CIRCUITPY_BOARD_UART_PIN    {{.tx = &pin_GPIO0, .rx = &pin_GPIO1}, \
                                     {.tx = &pin_GPIO20, .rx = &pin_GPIO21}}
