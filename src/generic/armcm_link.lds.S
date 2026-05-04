// Generic ARM Cortex-M linker script
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_FLASH_APPLICATION_ADDRESS

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

MEMORY
{
  rom (rx) : ORIGIN = CONFIG_FLASH_APPLICATION_ADDRESS , LENGTH = CONFIG_FLASH_SIZE
  ram (rwx) : ORIGIN = CONFIG_RAM_START , LENGTH = CONFIG_RAM_SIZE
#if CONFIG_HAVE_DMA_BUF_REGION
  /* Separate SRAM region reachable by DMA controllers.  On STM32H7 this is
     D2 SRAM at 0x30000000 — DTCM at 0x20000000 (the normal `ram` region) is
     CPU-private and not visible to DMA1/DMA2.  Buffers shared with DMA
     (phase-stepping XDIRECT TX, etc.) belong here. */
  dma_ram (rwx) : ORIGIN = CONFIG_DMA_BUF_START , LENGTH = CONFIG_DMA_BUF_SIZE
#endif
}

SECTIONS
{
    .text : {
        . = ALIGN(4);
        _text_vectortable_start = .;
        KEEP(*(.vector_table))
        _text_vectortable_end = .;
        *(.text .text.*)
        *(.rodata .rodata*)
    } > rom

    . = ALIGN(4);
    _data_flash = .;

#if CONFIG_ARMCM_RAM_VECTORTABLE
    .ram_vectortable (NOLOAD) : {
        _ram_vectortable_start = .;
        . = . + ( _text_vectortable_end - _text_vectortable_start ) ;
        _ram_vectortable_end = .;
    } > ram
#endif

    .data : AT (_data_flash)
    {
        . = ALIGN(4);
        _data_start = .;
        *(.ramfunc .ramfunc.*);
        *(.data .data.*);
        . = ALIGN(4);
        _data_end = .;
    } > ram

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > ram

#if CONFIG_HAVE_DMA_BUF_REGION
    /* DMA-coherent zero-init buffers (e.g. phase-stepping XDIRECT TX).
       Placed in D2 SRAM on H7 because DTCM is CPU-private. */
    .dma_buf (NOLOAD) :
    {
        . = ALIGN(32);
        _dma_buf_start = .;
        *(.dma_buf .dma_buf.*)
        . = ALIGN(32);
        _dma_buf_end = .;
    } > dma_ram
#else
    /* Without a separate DMA region, .dma_buf falls into normal BSS so the
       attribute is harmless on boards where the main RAM is already
       DMA-reachable. */
    .dma_buf_fallback (NOLOAD) :
    {
        . = ALIGN(32);
        *(.dma_buf .dma_buf.*)
        . = ALIGN(32);
    } > ram
#endif

    _stack_start = CONFIG_RAM_START + CONFIG_RAM_SIZE - CONFIG_STACK_SIZE ;
    .stack _stack_start (NOLOAD) :
    {
        . = . + CONFIG_STACK_SIZE;
        _stack_end = .;
    } > ram

    /DISCARD/ : {
        // The .init/.fini sections are used by __libc_init_array(), but
        // that isn't needed so no need to include them in the binary.
        *(.init)
        *(.fini)
        // Don't include exception tables
        *(.ARM.extab)
        *(.ARM.exidx)
    }
}
