/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x1B000, LENGTH = 0x24000
  RAM (rwx) :  ORIGIN = 0x20002800, LENGTH = 0x5800
}

INCLUDE "nrf5x_common.ld"