/* TREZORv2 firmware linker script */


MEMORY {
  FLASH_KERNEL (rx)  : ORIGIN = 0x0C050000, LENGTH = 160K
  FLASH_APP (rx)  : ORIGIN = 0x0C050000 + 160K, LENGTH = 1664K - 160K
}

_codelen = 160K - SIZEOF(.vendorheader) - SIZEOF(.header) + SIZEOF(.flash_app);


SECTIONS {
  .vendorheader : ALIGN(4) {
    KEEP(*(.vendorheader))
  } >FLASH_KERNEL AT>FLASH_KERNEL

  .header : ALIGN(4) {
    KEEP(*(.header));
  } >FLASH_KERNEL AT>FLASH_KERNEL

  .flash_kernel : ALIGN(512) {
    KEEP(*(.kernel));
    . = ALIGN(512);
  } >FLASH_KERNEL AT>FLASH_KERNEL

  .flash_app : ALIGN(512) {
    KEEP(*(.coreapp));
    *(.coreapp*);
    . = ALIGN(512);
  } >FLASH_APP AT>FLASH_APP

}
