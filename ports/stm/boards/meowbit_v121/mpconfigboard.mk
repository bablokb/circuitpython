USB_VID = 0x239A
USB_PID = 0x80CF
USB_PRODUCT = "Meowbit"
USB_MANUFACTURER = "Kittenbot"

SPI_FLASH_FILESYSTEM = 1
EXTERNAL_FLASH_DEVICES = W25Q16JVxQ
# INTERNAL_FLASH_FILESYSTEM = 1

BOOTLOADER_OFFSET = 0x8010000

MCU_SERIES = F4
MCU_VARIANT = STM32F401xE
MCU_PACKAGE = LQFP64

CIRCUITPY_BUILD_EXTENSIONS = uf2

OPTIMIZATION_FLAGS = -Os

LD_COMMON = boards/common_default.ld
LD_FILE = boards/STM32F401xe_boot.ld
# For debugging - also comment BOOTLOADER_OFFSET and BOARD_VTOR_DEFER
# LD_FILE = boards/STM32F401xe_fs.ld

CIRCUITPY_AESIO = 0
CIRCUITPY_BITMAPFILTER = 0
CIRCUITPY_BITMAPTOOLS = 0
CIRCUITPY_BLEIO_HCI = 0
CIRCUITPY_EPAPERDISPLAY = 0
CIRCUITPY_FRAMEBUFFERIO = 0
CIRCUITPY_I2CDISPLAYBUS = 0
CIRCUITPY_KEYPAD_DEMUX = 0
CIRCUITPY_SHARPDISPLAY = 0
CIRCUITPY_TILEPALETTEMAPPER = 0
CIRCUITPY_ULAB = 0
CIRCUITPY_ZLIB = 0

CIRCUITPY_STAGE = 1

FROZEN_MPY_DIRS += $(TOP)/frozen/circuitpython-stage/meowbit
