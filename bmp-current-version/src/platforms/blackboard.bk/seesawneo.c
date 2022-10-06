#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include "seesawneo.h"
#include <string.h>

uint8_t numpix = 24;

void clearnwrite(uint8_t start, uint8_t start2, uint8_t green, uint8_t red, uint8_t blue) {
    clearseesaw(numpix);
   uint8_t cmdWrite1[] = { 0xe, 0x4, start, start2, green, red, blue };
   i2c_transfer7(I2C3, 0x49, cmdWrite1, sizeof(cmdWrite1), NULL, 0);
   for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }   
     uint8_t cmdWrite2[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, 0x49, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  } 
}  

void neowrite(uint8_t start, uint8_t start2, uint8_t green, uint8_t red, uint8_t blue) {

   uint8_t cmdWrite1[] = { 0xe, 0x4, start, start2, green, red, blue };
   i2c_transfer7(I2C3, 0x49, cmdWrite1, sizeof(cmdWrite1), NULL, 0);
   for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }   
     uint8_t cmdWrite2[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, 0x49, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  } 
}  
 
void neoeven(uint8_t green, uint8_t red, uint8_t blue) {
uint8_t i;
uint8_t write;
write = numpix * 3;
for (i = 3; i < write; i += 6) {
   if (i >= write) {
   return;
   } 
   uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, i, green, red, blue };
   i2c_transfer7(I2C3, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);

  
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
}   
  uint8_t cmdWrite2[] = { 0xe, 0x5 };
  i2c_transfer7(I2C3, 0x49, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  } 

}

void neoodd(uint8_t green2, uint8_t red2, uint8_t blue2) {
uint8_t ii;
uint8_t write2;
write2 = numpix * 3;
for (ii = 6; ii < write2; ii += 6) {
if (ii >= write2) {
   return;
   }
   
   uint8_t cmdWrite3[] = { 0xe, 0x4, 0x0, ii, green2, red2, blue2 };
   i2c_transfer7(I2C3, 0x49, cmdWrite3, sizeof(cmdWrite3), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
  uint8_t cmdWrite4[] = { 0xe, 0x5 };
  i2c_transfer7(I2C3, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");

   }  
   
for (uint32_t loop2 = 6; loop2 < 22000000; ++loop2) {
    __asm__("nop");
  } 
 } 
}


void neoeveryother(uint8_t green, uint8_t red, uint8_t blue, uint8_t green2, uint8_t red2, uint8_t blue2) {
uint8_t mloop;
for (mloop = 0; mloop < 2; mloop++) {

neoeven(green, red, blue);
for (uint32_t loop = 0; loop < 18000000; ++loop) {
    __asm__("nop");
  } 

clearseesaw(24);

neoodd(green2, red2, blue2);

for (uint32_t loop2 = 0; loop2 < 18000000; ++loop2) {
    __asm__("nop");
  } 
  
clearseesaw(24);  
 
 }
}

void neoup(uint8_t pins, uint8_t green, uint8_t red, uint8_t blue) {
uint8_t i;
uint8_t bufs;
bufs = pins * 3;
for (i = 3; i < bufs; i += 3) {

uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, i, green, red, blue };
i2c_transfer7(I2C3, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);
for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}

uint8_t cmdWrite4[] = { 0xe, 0x5 };
i2c_transfer7(I2C3, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}

for (uint32_t loop2 = 0; loop2 < 18000000; ++loop2) {
    __asm__("nop");
  } 
 }
}
  
void neodown(uint8_t pins, uint8_t green, uint8_t red, uint8_t blue) {
uint8_t i;
uint8_t bufs;
bufs = pins * 3;
for (i = bufs; i >= 1; i -= 3) {

uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, i, green, red, blue };
i2c_transfer7(I2C3, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);

for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}

uint8_t cmdWrite4[] = { 0xe, 0x5 };
i2c_transfer7(I2C3, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}

  
for (uint32_t loop2 = 0; loop2 < 18000000; ++loop2) {
    __asm__("nop");
  } 
 }
}

void seesawneoint(uint8_t npix) {
  uint8_t bufs;
  numpix = npix;
  bufs = npix * 3;
    uint8_t cmdWrite[] = { 0xe, 0x2, 0x1 };
	i2c_transfer7(I2C3, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite1[] = { 0xe, 0x3, 0x0, bufs };
	i2c_transfer7(I2C3, 0x49, cmdWrite1, sizeof(cmdWrite1), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite2[] = { 0xe, 0x1, 0xa };
	i2c_transfer7(I2C3, 0x49, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite3[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, 0x49, cmdWrite3, sizeof(cmdWrite3), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite4[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
}

void clearseesaw(uint8_t numofneo) {
uint8_t clear = numofneo * 3;
int i;
clear = numofneo * 3;

for (i = 3; i < clear; i += 3) {

 
   uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, i, 0x0, 0x0, 0x0 };
   i2c_transfer7(I2C3, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);

  
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
}
      uint8_t cmdWrite4[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }


}  
  
void sendi2cdma(void){
    uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, 0x6, 0xff, 0x0, 0x0 };
    
    i2c_dma_start(cmdWrite, sizeof(cmdWrite));
    uint8_t cmdWrite4[] = { 0xe, 0x5 };
    i2c_dma_start(cmdWrite4, sizeof(cmdWrite4));
}


int i2c_dma_start(uint8_t *tx_buf, int tx_len) {

//	dma_channel_reset(DMA1, DMA_STREAM4);
	if (tx_len > 0) {
    rcc_periph_clock_enable(RCC_DMA1);
//	nvic_enable_irq(NVIC_DMA1_STREAM4_IRQ);
	dma_stream_reset(DMA1, DMA_STREAM4);
	dma_set_priority(DMA1, DMA_STREAM4, DMA_SxCR_PL_LOW);
	dma_set_memory_size(DMA1, DMA_STREAM4, DMA_SxCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_STREAM4, DMA_SxCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_STREAM4);
	dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM4);
//	dma_enable_circular_mode(DMA1, DMA_STREAM4);
	dma_set_transfer_mode(DMA1, DMA_STREAM4, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	dma_set_peripheral_address(DMA1, DMA_STREAM4, (uint32_t) &I2C_DR(I2C3));
	dma_set_memory_address(DMA1, DMA_STREAM4, (uint32_t) tx_buf);
	dma_set_number_of_data(DMA1, DMA_STREAM4, tx_len);
//	dma_set_memory_address(DMA1, DMA_STREAM4, (uint32_t) tfttx);
//	dma_set_number_of_data(DMA1, DMA_STREAM4, 1024);
//	dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM4);
//	dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM4);
	
//	dma_enable_stream(DMA1, DMA_STREAM4); 
//	spi_enable_tx_dma(SPI2);
//    nvic_enable_irq(NVIC_DMA1_STREAM4_IRQ);
//    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM4);
    dma_channel_select(DMA1, DMA_STREAM4, DMA_SxCR_CHSEL_3);
    dma_enable_stream(DMA1, DMA_STREAM4);

	/* Enable dma transfer complete interrupts */

//	if (tx_len > 0) {
//		dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM4);
//	}

	/* Activate dma channels */

	
		dma_enable_stream(DMA1, DMA_STREAM4);
	

	/* Enable the spi transfer via dma
	 * This will immediately start the transmission,
	 * after which when the receive is complete, the
	 * receive dma will activate
	 */

    
    	i2c_enable_dma(I2C3);
    

    

    return 0;
    }
    return 0;
}

