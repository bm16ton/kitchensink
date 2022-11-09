#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <stdint.h>

uint8_t w25_read_sr1(uint32_t spi);
uint8_t w25_read_sr2(uint32_t spi);
bool w25_is_wprotect(uint32_t spi;
bool w25_chip_erase(uint32_t spi);
uint32_t w25_read_data(uint32_t spi,uint32_t addr,void *data,uint32_t bytes);
void w25_write_en(uint32_t spi,bool en);
unsigned w25_write_data(uint32_t spi,uint32_t addr,void *data,uint32_t bytes);
void w25_erase_block(uint32_t spi,uint32_t addr,uint8_t cmd);
void flash_status(void);
uint32_t dump_page(uint32_t spi,uint32_t addr);
void erase(uint32_t spi,uint32_t addr);
void load_ihex(uint32_t spi);
uint16_t w25_manuf_device(uint32_t spi);
uint32_t w25_JEDEC_ID(uint32_t spi);
void w25_read_uid(uint32_t spi,void *buf,uint16_t bytes);
void spiflash_setup(void);

#endif
