#include "ILI9341.h"
#include "font.h"
#include <stdio.h>
#define FONT_SPACE 6
#define FONT_X 8
#define FONT_Y 8



void spi_setup(void)
{
uint32_t tmp;
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_GPIOC | RCC_GPIOA | RCC_GPIOF | RCC_GPIOE);

	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO7);

//spi initialization;
spi_set_master_mode(SPI1);
//spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);  // Działa OK
/*
spi_set_baudrate_prescaler(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_2);  // 
spi_set_clock_polarity_0(SPI1);
spi_set_clock_phase_0(SPI1);
spi_set_unidirectional_mode(SPI1); 
spi_enable_software_slave_management(SPI1);
spi_send_msb_first(SPI1);
spi_set_nss_high(SPI1);
SPI_I2SCFGR(SPI1) &= ~SPI_I2SCFGR_I2SMOD;
spi_enable(SPI1);
*/
	tmp = SPI_SR(SPI1);
	SPI_CR2(SPI1) |= (SPI_CR2_SSOE | SPI_CR2_RXNEIE);

	/* device clocks on the rising edge of SCK with MSB first */
	tmp = SPI_CR1_BAUDRATE_FPCLK_DIV_4 | /* 10.25Mhz SPI Clock (42M/4) */
	      SPI_CR1_MSTR |                 /* Master Mode */
	      SPI_CR1_BIDIOE |               /* Write Only */
	      SPI_CR1_SPE;                   /* Enable SPI */

	SPI_CR1(SPI1) = tmp; /* Do it. */
	if (SPI_SR(SPI1) & SPI_SR_MODF) {
		SPI_CR1(SPI1) = tmp; /* Re-writing will reset MODF */
//		fprintf(stderr, "Initial mode fault.\n");
	}
}




void TFTinit (void)
{

    TFT_CS_HIGH;
    TFT_DC_HIGH;
    char TFTDriver=0;
    int i=0;
	//printf("Przed resetem\r\n"); 
	
    TFT_RST_ON;
    for (i = 0; i < 1000; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
    TFT_RST_OFF;
    
    
	//printf("Po resecie\r\n"); 
	
    for(i=0;i<3;i++)
    {
		//printf("Pętla: %d\r\n",i); 
        TFTDriver = readID();
    }
    
    
  sendCMD(0xEF);
  WRITE_DATA(0x03);
  WRITE_DATA(0x80);
  WRITE_DATA(0x02);

  sendCMD(0xCF);
  WRITE_DATA(0x00);
  WRITE_DATA(0XC1);
  WRITE_DATA(0X30);

  sendCMD(0xED);
  WRITE_DATA(0x64);
  WRITE_DATA(0x03);
  WRITE_DATA(0X12);
  WRITE_DATA(0X81);

  sendCMD(0xE8);
  WRITE_DATA(0x85);
  WRITE_DATA(0x00);
  WRITE_DATA(0x78);

  sendCMD(0xCB);
  WRITE_DATA(0x39);
  WRITE_DATA(0x2C);
  WRITE_DATA(0x00);
  WRITE_DATA(0x34);
  WRITE_DATA(0x02);

  sendCMD(0xF7);
  WRITE_DATA(0x20);

  sendCMD(0xEA);
  WRITE_DATA(0x00);
  WRITE_DATA(0x00);

  sendCMD(0xC0);    //Power control
  WRITE_DATA(0x23);   //VRH[5:0]

  sendCMD(0xC1);    //Power control
  WRITE_DATA(0x10);   //SAP[2:0];BT[3:0]

  sendCMD(0xC5);    //VCM control
  WRITE_DATA(0x3e);
  WRITE_DATA(0x28);

  sendCMD(0xC7);    //VCM control2
  WRITE_DATA(0x86);  //--

  sendCMD(0x36);    // Memory Access Control
//#ifdef M5STACK
//  WRITE_DATA(TFT_MAD_MY | TFT_MAD_MV | TFT_MAD_COLOR_ORDER); // Rotation 0 (portrait mode)
//#else
  WRITE_DATA(0x40 | 0x00); // Rotation 0 (portrait mode)
//#endif

  sendCMD(0x3A);
  WRITE_DATA(0x55);

  sendCMD(0xB1);
  WRITE_DATA(0x00);
  WRITE_DATA(0x13); // 0x18 79Hz, 0x1B default 70Hz, 0x13 100Hz

  sendCMD(0xB6);    // Display Function Control
  WRITE_DATA(0x08);
  WRITE_DATA(0x82);
  WRITE_DATA(0x27);

  sendCMD(0xF2);    // 3Gamma Function Disable
  WRITE_DATA(0x00);

  sendCMD(0x26);    //Gamma curve selected
  WRITE_DATA(0x01);

  sendCMD(0xE0);    //Set Gamma
  WRITE_DATA(0x0F);
  WRITE_DATA(0x31);
  WRITE_DATA(0x2B);
  WRITE_DATA(0x0C);
  WRITE_DATA(0x0E);
  WRITE_DATA(0x08);
  WRITE_DATA(0x4E);
  WRITE_DATA(0xF1);
  WRITE_DATA(0x37);
  WRITE_DATA(0x07);
  WRITE_DATA(0x10);
  WRITE_DATA(0x03);
  WRITE_DATA(0x0E);
  WRITE_DATA(0x09);
  WRITE_DATA(0x00);

  sendCMD(0xE1);    //Set Gamma
  WRITE_DATA(0x00);
  WRITE_DATA(0x0E);
  WRITE_DATA(0x14);
  WRITE_DATA(0x03);
  WRITE_DATA(0x11);
  WRITE_DATA(0x07);
  WRITE_DATA(0x31);
  WRITE_DATA(0xC1);
  WRITE_DATA(0x48);
  WRITE_DATA(0x08);
  WRITE_DATA(0x0F);
  WRITE_DATA(0x0C);
  WRITE_DATA(0x31);
  WRITE_DATA(0x36);
  WRITE_DATA(0x0F);

  sendCMD(0x11);    //Exit Sleep
 

  delay(120);

  
  sendCMD(0x29);    //Display on

fillScreenALL();
    
    

}

char readID(void)
{
    char i=0;
    char data[3] ;
    char ID[3] = {0x00, 0x93, 0x41};
    char ToF=1;
    for(i=0;i<3;i++)
    {
		//rintf("Read reg 1\r\n"); 
        data[i]=Read_Register(0xd3,i+1);
        if(data[i] != ID[i])
        {
            ToF=0;
        }
        //printf("Read reg 2\r\n"); 
    }
    if(!ToF) /* data!=ID */
    {
//        printf("\n\rRead TFT ID failed, ID should be 0x09341, but read ID = 0x");
        for(i=0;i<3;i++)
        {
//            printf("Dane : %d - %d " , i , data[i] );
        }
        //Serial.println();
    } else
//		printf("TFT Found\n\r");
    return ToF;
}

char Read_Register(char Addr, char xParameter)
{
    char data=0;
    uint8_t tt = 0;
    int i;
    sendCMD(0xd9); /* ext command */
    WRITE_DATA(0x10+xParameter); /* 0x11 is the first Parameter */

    TFT_DC_LOW;
    TFT_CS_LOW;
    for (i = 0; i < 5; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
    tt = spi_xfer(SPI1, Addr);
		while (!(SPI_SR(SPI1) & SPI_SR_RXNE));
    TFT_DC_HIGH;
    for ( i = 0; i < 2; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
    tt =  SPI_DR(SPI1);
    tt = spi_readwrite(SPI1, 0);
    
    TFT_CS_HIGH;
    return tt;
}

uint8_t spi_readwrite(uint32_t spi, uint8_t data)
{
	while (SPI_SR(spi) & SPI_SR_BSY);
	SPI_DR(spi) = data;
	while (!(SPI_SR(spi) & SPI_SR_RXNE));
	return SPI_DR(spi);
}

void sendCMD(char index)
{
	int i;
	char tt;
    TFT_DC_LOW;
    TFT_CS_LOW;
    tt =  SPI_DR(SPI1);
    for (i = 0; i < 5; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
    spi_write(SPI1, index);
    /* Wait for transfer finished. */
		while (!(SPI_SR(SPI1) & SPI_SR_RXNE));
    TFT_CS_HIGH;
}

void WRITE_DATA(char data)
{
	int i;
    TFT_DC_HIGH;
    TFT_CS_LOW;
    for ( i = 0; i < 5; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
    spi_xfer(SPI1, data);
    /* Wait for transfer finished. */
	while (!(SPI_SR(SPI1) & SPI_SR_RXNE));
    TFT_CS_HIGH;

}

void sendData(uint16_t data)
{
	uint8_t i;
    uint8_t data1 = data>>8;
    uint8_t data2 = data&0xff;
    TFT_DC_HIGH;
    TFT_CS_LOW;
    for (i = 0; i < 10; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
    spi_xfer(SPI1, data1);
    /* Wait for transfer finished. */
	while (!(SPI_SR(SPI1) & SPI_SR_RXNE));
    spi_xfer(SPI1, data2);
    /* Wait for transfer finished. */
	while (!(SPI_SR(SPI1) & SPI_SR_RXNE));
    TFT_CS_HIGH;
}


void fillScreenALL(void)
{
    setCol(0, 239);
    setPage(0, 319);
    sendCMD(0x2c); /* start to write to display ra */
    uint16_t       i;                                                   /* m */
    
    TFT_DC_HIGH;
    TFT_CS_LOW;
    for (i = 0; i < 15; i++) { /* Wait a bit. */
			__asm__("NOP");
	}
	
    for( i=0; i<38400; i++)
    {
    spi_readwrite(SPI1, 0);
    spi_readwrite(SPI1, 0);
    spi_readwrite(SPI1, 0);
    spi_readwrite(SPI1, 0);
    }
    TFT_CS_HIGH;
    
}

void setCol(uint16_t  StartCol,uint16_t  EndCol)
{
    sendCMD(0x2A); /* Column Command address */
    sendData(StartCol);
    sendData(EndCol);
}

void setPage(uint16_t  StartPage,uint16_t  EndPage)
{
    sendCMD(0x2B); /* Column Command address */
    sendData(StartPage);
    sendData(EndPage);
}


void setXY(uint16_t poX, uint16_t poY)
{
    setCol(poX, poX);
    setPage(poY, poY);
    sendCMD(0x2c);
}

void setPixel(uint16_t poX, uint16_t poY,uint16_t color)
{
    setXY(poX, poY);
    sendData(color);
}

void drawHorizontalLine( uint16_t poX, uint16_t poY,uint16_t length,uint16_t color)
{
	uint16_t i;
    setCol(poX,poX + length);
    setPage(poY,poY);
    sendCMD(0x2c);
    for( i=0; i<length; i++)
    sendData(color);
}

void drawVerticalLine( uint16_t poX, uint16_t poY, uint16_t length,uint16_t color)
{
	uint16_t i;
    setCol(poX,poX);
    setPage(poY,poY+length);
    sendCMD(0x2c);
    for( i=0; i<length; i++)
    sendData(color);
}

uint16_t constrain(uint16_t X, uint16_t A, uint16_t B){
	
	if ( X < A )
		return A;
	else if ( X > B )
		return B;
	else
		return X;	
}

void fillScreen(uint16_t XL, uint16_t XR, uint16_t YU, uint16_t YD, uint16_t color)
{
    unsigned long XY=0;
    unsigned long i=0;

    if(XL > XR)
    {
        XL = XL^XR;
        XR = XL^XR;
        XL = XL^XR;
    }
    if(YU > YD)
    {
        YU = YU^YD;
        YD = YU^YD;
        YU = YU^YD;
    }
    XL = constrain(XL, MIN_X,MAX_X);
    XR = constrain(XR, MIN_X,MAX_X);
    YU = constrain(YU, MIN_Y,MAX_Y);
    YD = constrain(YD, MIN_Y,MAX_Y);

    XY = (XR-XL+1);
    XY = XY*(YD-YU+1);

    setCol(XL,XR);
    setPage(YU, YD);
    sendCMD(0x2c); /* start to write to display ra */
                                                                        /* m */

    TFT_DC_HIGH;
    TFT_CS_LOW;
    for (i = 0; i < 15; i++) { /* Wait a bit. */
			__asm__("NOP");
	}
	
    uint8_t Hcolor = color>>8;
    uint8_t Lcolor = color&0xff;
    for(i=0; i < XY; i++)
    {
        //SPI.transfer(Hcolor);
        //SPI.transfer(Lcolor);
        spi_xfer(SPI1, Hcolor);
		/* Wait for transfer finished. */
		while (!(SPI_SR(SPI1) & SPI_SR_RXNE));
		spi_xfer(SPI1, Lcolor);
		/* Wait for transfer finished. */
		while (!(SPI_SR(SPI1) & SPI_SR_RXNE));
    }

    TFT_CS_HIGH;
}

void fillRectangle(uint16_t poX, uint16_t poY, uint16_t length, uint16_t width, uint16_t color)
{
    fillScreen(poX, poX+length, poY, poY+width, color);
}

void drawLine( uint16_t x0,uint16_t y0,uint16_t x1, uint16_t y1,uint16_t color)
{

    uint16_t x = x1-x0;
    uint16_t y = y1-y0;
    uint16_t dx = abs(x), sx = x0<x1 ? 1 : -1;
    uint16_t dy = -abs(y), sy = y0<y1 ? 1 : -1;
    uint16_t err = dx+dy, e2; /* error value e_xy */
    for (;;){ /* loop */
        setPixel(x0,y0,color);
        e2 = 2*err;
        if (e2 >= dy) { /* e_xy+e_x > 0 */
            if (x0 == x1) break;
            err += dy; x0 += sx;
        }
        if (e2 <= dx) { /* e_xy+e_y < 0 */
            if (y0 == y1) break;
            err += dx; y0 += sy;
        }
    }

}


void drawCircle(uint16_t  poX, uint16_t  poY, uint16_t  r,uint16_t  color)
{
    int16_t  x , y , err , e2;
    x = -r;
    y = 0;
    err = 2-2*r;
    do {
        setPixel(poX-x, poY+y,color);
        setPixel(poX+x, poY+y,color);
        setPixel(poX+x, poY-y,color);
        setPixel(poX-x, poY-y,color);
        e2 = err;
        if (e2 <= y) {
            err += ++y*2+1;
            if (-x == y && e2 <= x) e2 = 0;
        }
        if (e2 > x) err += ++x*2+1;
    } while (x <= 0);
}


void fillCircle(uint16_t  poX, uint16_t  poY, uint16_t  r,uint16_t  color)
{
    int16_t  x = -r, y = 0, err = 2-2*r, e2;
    do {

        drawVerticalLine(poX-x, poY-y, 2*y, color);
        drawVerticalLine(poX+x, poY-y, 2*y, color);

        e2 = err;
        if (e2 <= y) {
            err += ++y*2+1;
            if (-x == y && e2 <= x) e2 = 0;
        }
        if (e2 > x) err += ++x*2+1;
    } while (x <= 0);

}

void drawChar( uint8_t ascii, uint16_t poX, uint16_t poY,uint16_t size, uint16_t fgcolor)
{
	uint8_t f;
	uint16_t i;
    if((ascii>=32)&&(ascii<=127))
    {
        ;
    }
    else
    {
        ascii = '?'-32;
    }
    for (i =0; i<FONT_X; i++ ) {
         uint8_t  temp = simpleFont[ascii-0x20][i];
        for( f=0;f<8;f++)
        {
            if((temp>>f)&0x01)
            {
                fillRectangle(poX+i*size, poY+f*size, size, size, fgcolor);
            }

        }

    }
}


void drawString(char *string,uint16_t poX, uint16_t poY, uint16_t size,uint16_t fgcolor)
{
    while(*string)
    {
        drawChar(*string, poX, poY, size, fgcolor);
        *string++;

        if(poX < MAX_X)
        {
            poX += FONT_SPACE*size; /* Move cursor right */
        }
    }
}

uint16_t RGBConv(uint16_t R, uint16_t G, uint16_t B)
{
    return ((unsigned int)( (( R >> 3 ) << 11 ) | (( G >> 2 ) << 5  ) | ( B  >> 3 )));
}
