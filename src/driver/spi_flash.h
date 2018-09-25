#pragma once

#define M25P16_PAGESIZE 256

typedef struct flashGeometry_s {
    uint16_t sectors; // Count of the number of erasable blocks on the device

    uint16_t pagesPerSector;
    uint16_t pageSize; // In bytes

    uint32_t sectorSize; // This is just pagesPerSector * pageSize

    uint32_t totalSize;  // This is just sectorSize * sectors
} flashGeometry_t;



bool spi_flash_init(void);

bool spi_flash_eraseSector(uint32_t address);
bool spi_flash_eraseCompletely(void);

bool spi_flash_pageProgram(uint32_t address, const uint8_t *data, int length);

bool spi_flash_pageProgramBegin(uint32_t address);
bool spi_flash_pageProgramContinue(const uint8_t *data, int length);
void spi_flash_pageProgramFinish(void);

int spi_flash_readBytes(uint32_t address, uint8_t *buffer, int length);

bool spi_flash_isReady(void);
bool spi_flash_waitForReady(uint32_t timeoutMillis);

const flashGeometry_t* spi_flash_getGeometry(void);


