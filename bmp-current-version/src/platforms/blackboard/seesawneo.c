#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include "seesawneo.h"
#include <string.h>
#include "timing_stm32.h"
#include <libopencm3/stm32/timer.h>

#define PRESCALE    10000
#define PERIOD      8400

uint8_t numpix = 24;
int timvalpoo = 0;

void neotimtest(void);
static void DMA_Transmit(uint8_t * pBuffer, uint8_t size);
void i2c_dma_start(void);


void tim2_isr(void)
{
        if(timer_interrupt_source(TIM2, TIM_SR_UIF)) {
	        timer_clear_flag(TIM2, TIM_SR_UIF);   // Clear the Interrup Flag
	        neotimtest();	      // Toggle LED.
//	        timer_enable_counter(TIM2);
	}
}

static void timer2_setup(void)
{
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_reset_pulse(RCC_APB1RSTR_TIM2RST);

	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM2, PRESCALE);     // TIM2_PSC = PRESCALE;  
	timer_set_period(TIM2, PERIOD);          // TIM2_ARR = PERIOD;    Sets the auto-reload reg
	timer_enable_update_event(TIM2);         // TIM2_CR1.UDIS = 0;
	timer_update_on_any(TIM2);               // TIM2_CR1.URS = 0;
	timer_generate_event(TIM2, TIM_EGR_UG);  // TIM2_EGR.UG = 1;
	timer_enable_irq(TIM2, TIM_DIER_UIE);    // Enable the Interrupt
	timer_continuous_mode(TIM2);
	timer_enable_preload(TIM2);

	nvic_enable_irq(NVIC_TIM2_IRQ);          // Enable the Interrupt in the Interrupt Controller
	timer_enable_counter(TIM2);              // Start the counter.
}

void neotimodd(void) {
    timer2_setup();
//    neotimtest();
}

void neotimtest(void){

if (timvalpoo == 0) {
   uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, 0x6, 0xff, 0xff, 0x0 };
    I2C_write(0x49, cmdWrite, sizeof(cmdWrite));
    timvalpoo = 1;
} else if (timvalpoo == 1) {
    uint8_t cmdWrite4[] = { 0xe, 0x5 };
    I2C_write(0x49, cmdWrite4, sizeof(cmdWrite4));
    timvalpoo = 2;
    } else if (timvalpoo == 2) {
   uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, 0x6, 0x0, 0x0, 0xff };
    I2C_write(0x49, cmdWrite, sizeof(cmdWrite));
    timvalpoo = 3;
} else if (timvalpoo == 3) {
    uint8_t cmdWrite4[] = { 0xe, 0x5 };
    I2C_write(0x49, cmdWrite4, sizeof(cmdWrite4));
    timvalpoo = 0;
    }
}  

void clearnwrite(uint8_t start, uint8_t start2, uint8_t green, uint8_t red, uint8_t blue) {
    clearseesaw(numpix);
   uint8_t cmdWrite1[] = { 0xe, 0x4, start, start2, green, red, blue };
   i2c_transfer7(I2C2, 0x49, cmdWrite1, sizeof(cmdWrite1), NULL, 0);
   for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }   
     uint8_t cmdWrite2[] = { 0xe, 0x5 };
	i2c_transfer7(I2C2, 0x49, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  } 
}  

void neowrite(uint8_t start, uint8_t start2, uint8_t green, uint8_t red, uint8_t blue) {

   uint8_t cmdWrite1[] = { 0xe, 0x4, start, start2, green, red, blue };
   i2c_transfer7(I2C2, 0x49, cmdWrite1, sizeof(cmdWrite1), NULL, 0);
   for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }   
     uint8_t cmdWrite2[] = { 0xe, 0x5 };
	i2c_transfer7(I2C2, 0x49, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
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
   i2c_transfer7(I2C2, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);

  
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
}   
  uint8_t cmdWrite2[] = { 0xe, 0x5 };
  i2c_transfer7(I2C2, 0x49, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  } 

}

void neoodd(uint8_t green2, uint8_t red2, uint8_t blue2) {
uint8_t ii;
uint8_t write2;
write2 = numpix * 3;
for (ii = 0; ii < write2; ii += 6) {
if (ii >= write2) {
   return;
   }
   
   uint8_t cmdWrite3[] = { 0xe, 0x4, 0x0, ii, green2, red2, blue2 };
   i2c_transfer7(I2C2, 0x49, cmdWrite3, sizeof(cmdWrite3), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
 }
  uint8_t cmdWrite4[] = { 0xe, 0x5 };
  i2c_transfer7(I2C2, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");

   }  
}
//for (uint32_t loop2 = 6; loop2 < 9000000; ++loop2) {
//    __asm__("nop");
//  } 
  
//}
void neoodddma(uint8_t green2, uint8_t red2, uint8_t blue2) {
uint8_t ii;
uint8_t write2;
write2 = numpix * 3;
for (ii = 0; ii < write2; ii += 6) {
if (ii >= write2) {
   return;
   }
   
   uint8_t cmdWrite3[] = { 0xe, 0x4, 0x0, ii, green2, red2, blue2 };
//   i2c_transfer7(I2C2, 0x49, cmdWrite3, sizeof(cmdWrite3), NULL, 0);
   I2C_write(0x49, cmdWrite3, sizeof(cmdWrite3));
//  for (uint32_t loop = 0; loop < 100; ++loop) {
//    __asm__("nop");
//  }
 }
  uint8_t cmdWrite4[] = { 0xe, 0x5 };
//  i2c_transfer7(I2C2, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
  I2C_write(0x49, cmdWrite4, sizeof(cmdWrite4));
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");

   }  
}

void neoeveryother(uint8_t green, uint8_t red, uint8_t blue, uint8_t green2, uint8_t red2, uint8_t blue2) {
uint8_t mloop;
for (mloop = 0; mloop < 2; mloop++) {

neoeven(green, red, blue);
for (uint32_t loop = 0; loop < 10; ++loop) {
    __asm__("nop");
  } 
  
neoodd(green2, red2, blue2);

for (uint32_t loop2 = 0; loop2 < 18000000; ++loop2) {
    __asm__("nop");
  } 
  
  neoeven(green2, red2, blue2);
for (uint32_t loop = 0; loop < 10; ++loop) {
    __asm__("nop");
  } 
  
neoodd(green, red, blue);

for (uint32_t loop2 = 0; loop2 < 18000000; ++loop2) {
    __asm__("nop");
  } 

//clearseesaw(24);  
 
 }
}

void neoup(uint8_t pins, uint8_t green, uint8_t red, uint8_t blue) {
uint8_t i;
uint8_t bufs;
bufs = pins * 3;
for (i = 0; i < bufs; i += 3) {

uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, i, green, red, blue };
i2c_transfer7(I2C2, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);
for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}

uint8_t cmdWrite4[] = { 0xe, 0x5 };
i2c_transfer7(I2C2, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}

for (uint32_t loop2 = 0; loop2 < 9000000; ++loop2) {
    __asm__("nop");
  } 
 }
}

void neoall(uint8_t pins, uint8_t green, uint8_t red, uint8_t blue) {
uint8_t i;
uint8_t bufs;
bufs = pins * 3;
for (i = 0; i < bufs; i += 3) {

uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, i, green, red, blue };
i2c_transfer7(I2C2, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);
for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}

uint8_t cmdWrite4[] = { 0xe, 0x5 };
i2c_transfer7(I2C2, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}
 
 }
}
  
void neodown(uint8_t pins, uint8_t green, uint8_t red, uint8_t blue) {
int16_t i;
int16_t bufs;
bufs = pins * 3;
neoall(pins, green, red, blue);
for (i = bufs; i >= 0; i -= 3) {

uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, i, 0x00, 0x00, 0x00 };
i2c_transfer7(I2C2, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);

for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}

uint8_t cmdWrite4[] = { 0xe, 0x5 };
i2c_transfer7(I2C2, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
}

  
for (uint32_t loop2 = 0; loop2 < 9000000; ++loop2) {
    __asm__("nop");
  } 
 }
}

void seesawneoint(uint8_t npix) {
  uint8_t bufs;
  numpix = npix;
  bufs = npix * 3;
    uint8_t cmdWrite[] = { 0xe, 0x2, 0x1 };
	i2c_transfer7(I2C2, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite1[] = { 0xe, 0x3, 0x0, bufs };
	i2c_transfer7(I2C2, 0x49, cmdWrite1, sizeof(cmdWrite1), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite2[] = { 0xe, 0x1, 0xa };
	i2c_transfer7(I2C2, 0x49, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite3[] = { 0xe, 0x5 };
	i2c_transfer7(I2C2, 0x49, cmdWrite3, sizeof(cmdWrite3), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite4[] = { 0xe, 0x5 };
	i2c_transfer7(I2C2, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
}

void clearseesaw(uint8_t numofneo) {
uint8_t clear = numofneo * 3;
int i;
clear = numofneo * 3;

for (i = 0; i < clear; i += 3) {

 
   uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, i, 0x0, 0x0, 0x0 };
   i2c_transfer7(I2C2, 0x49, cmdWrite, sizeof(cmdWrite), NULL, 0);

  
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
}
      uint8_t cmdWrite4[] = { 0xe, 0x5 };
	i2c_transfer7(I2C2, 0x49, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }


}  

void i2c2_init(void)
		{
	    rcc_periph_clock_enable(RCC_I2C2);
	    rcc_periph_clock_enable(RCC_GPIOB);
	    rcc_periph_clock_enable(RCC_DMA1);
	    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO10);
	    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO10);
	    gpio_set_af(GPIOB, GPIO_AF4, GPIO10);
	    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO11);
	    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO11);
	    gpio_set_af(GPIOB, GPIO_AF4, GPIO11);

		i2c_reset(I2C2);
		I2C_CR1(I2C2) &=~ I2C_CR1_NOSTRETCH;//disable clock strech
		I2C_CR1(I2C2) &= ~I2C_CR1_ENGC;//diable generaral callback
//		I2C_CR2(I2C2) |= I2C_CR2_LAST;//set next DMA EOT is last transfer
//		i2c_set_dma_last_transfer(I2C2);
		i2c_enable_dma(I2C2);
//		I2C_CR2(I2C2) |=16;//set clock source to 16MHz
//        i2c_set_fast_mode(I2C2);
        i2c_set_clock_frequency(I2C2, 30);
        i2c_set_ccr(I2C2, 30 * 5);
	    i2c_set_trise(I2C2, 30 + 1);
		I2C_CR1(I2C2) |=I2C_CR1_PE;
		  for (uint32_t loop = 0; loop < 1000000; ++loop) {
          __asm__("nop");
            }
		}  
  
void sendi2ctest(void){

   uint8_t cmdWrite[] = { 0xe, 0x4, 0x0, 0x6, 0xff, 0xff, 0x0 };
    I2C_write(0x49, cmdWrite, sizeof(cmdWrite));
//    for (unsigned i = 0; i < 40000; i++)
//	  {
//		__asm__("nop");
//	  }


    uint8_t cmdWrite4[] = { 0xe, 0x5 };
    I2C_write(0x49, cmdWrite4, sizeof(cmdWrite4));

}  
 
void I2C_write(uint8_t SensorAddr,
    uint8_t * pWriteBuffer, uint16_t NumByteToWrite)
{
	while ((I2C_SR2(I2C2) & I2C_SR2_BUSY)) {
	}

	i2c_send_start(I2C2);
	/* Wait for the end of the start condition, master mode selected, and BUSY bit set */
	while ( !( (I2C_SR1(I2C2) & I2C_SR1_SB)
		&& (I2C_SR2(I2C2) & I2C_SR2_MSL)
		&& (I2C_SR2(I2C2) & I2C_SR2_BUSY) ));

	i2c_send_7bit_address(I2C2, SensorAddr, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(I2C2) & I2C_SR1_ADDR));

	/* Clearing ADDR condition sequence. */
	(void)I2C_SR2(I2C2);
    i2c_set_dma_last_transfer(I2C2);
  /* Start DMA */
		DMA_Transmit(pWriteBuffer, NumByteToWrite);
  /* Read SR1 */
			(void)I2C2_SR1;

  /* Read SR2 */
			(void)I2C2_SR2;
	
	while (!(I2C_SR1(I2C2) & I2C_SR1_BTF))
		;

	if (!(I2C_SR1(I2C2) & I2C_SR1_BTF)) {
		i2c_reset(I2C2);
	}

	i2c_send_stop(I2C2);
    i2c_clear_stop(I2C2);
	
}	 
  
static void DMA_Transmit(uint8_t * pBuffer, uint8_t size)
{
  /* Check null pointers */
  if(NULL != pBuffer)
  {
    DMA1_S7CR&=~DMA_SxCR_EN;
	while((DMA1_S7CR)&DMA_SxCR_EN){;}
		dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM7);

        dma_set_memory_address(DMA1, DMA_STREAM7, (uint32_t) pBuffer);
	    dma_set_number_of_data(DMA1, DMA_STREAM7, size);
	    dma_clear_interrupt_flags(DMA1, DMA_STREAM7, DMA_TCIF);
		dma_enable_stream(DMA1, DMA_STREAM7);
    	i2c_enable_dma(I2C2);
  }
  else
  {
    /* Null pointers, do nothing */
  }

}

void dma1_stream7_isr(void) {
if (dma_get_interrupt_flag(DMA1, DMA_STREAM7, DMA_TCIF)) {
    dma_clear_interrupt_flags(DMA1, DMA_STREAM7, DMA_TCIF);
    }
        // Turn our DMA channel back off, in preparation of the next transfer
        i2c_disable_dma(I2C2);
        dma_disable_stream(DMA1, DMA_STREAM7);
    

}

void i2c_dma_start(void) {
    rcc_periph_clock_enable(RCC_DMA1);
	dma_stream_reset(DMA1, DMA_STREAM7);
	dma_set_priority(DMA1, DMA_STREAM7, DMA_SxCR_PL_LOW);
	dma_set_memory_size(DMA1, DMA_STREAM7, DMA_SxCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA1, DMA_STREAM7, DMA_SxCR_PSIZE_8BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_STREAM7);
	dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM7);
	dma_set_transfer_mode(DMA1, DMA_STREAM7, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	dma_set_peripheral_address(DMA1, DMA_STREAM7, (uint32_t) &I2C_DR(I2C2));
	dma_set_memory_address(DMA1, DMA_STREAM7, (uint32_t) 0);
	dma_set_number_of_data(DMA1, DMA_STREAM7, 0);
    nvic_enable_irq(NVIC_DMA1_STREAM7_IRQ);
//    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM7);
    dma_channel_select(DMA1, DMA_STREAM7, DMA_SxCR_CHSEL_7);
    dma_enable_stream(DMA1, DMA_STREAM7);

}

