#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
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
for (mloop = 0; mloop < 10; mloop++) {

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
  

