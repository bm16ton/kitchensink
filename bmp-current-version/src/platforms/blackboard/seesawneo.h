#ifndef SEESAWNEO_H
#define SEESAWNEO_H

//uint16_t i2ctx[72] = {0};

void i2c_dma_start(void);

void sendi2cdma(uint8_t *tx_buf, int tx_len);

void sendi2ctest(void);

void i2c2_init(void);

void I2C_write(uint8_t SensorAddr, uint8_t * pWriteBuffer, uint16_t NumByteToWrite);

void neoodddma(uint8_t green2, uint8_t red2, uint8_t blue2);

void clearseesawdma(uint8_t numofneo);

void neoevendma(uint8_t green, uint8_t red, uint8_t blue);

void neotimodd(void);

void clearnwrite(uint8_t start, uint8_t start2, uint8_t green, uint8_t red, uint8_t blue);

void neowrite(uint8_t start, uint8_t start2, uint8_t green, uint8_t red, uint8_t blue);

void seesawneoint(uint8_t numpix);

void clearseesaw(uint8_t numofneo);

void neoeveryother(uint8_t green, uint8_t red, uint8_t blue, uint8_t green2, uint8_t red2, uint8_t blue2);

void neoeveryotherdma(uint8_t green, uint8_t red, uint8_t blue, uint8_t green2, uint8_t red2, uint8_t blue2);

void neodown(uint8_t pins, uint8_t green, uint8_t red, uint8_t blue);

void neoup(uint8_t pins, uint8_t green, uint8_t red, uint8_t blue);

void neoevendma(uint8_t green, uint8_t red, uint8_t blue);

void neoeven(uint8_t green, uint8_t red, uint8_t blue);

void neoodddma(uint8_t green2, uint8_t red2, uint8_t blue2);

void neoodd(uint8_t green, uint8_t red, uint8_t blue);

void neotimtest(void);
#endif
