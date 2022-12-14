#include <libopencm3/cm3/scb.h>
 #include "libopencm3/stm32/rcc.h"
 #include "libopencm3/stm32/gpio.h"
 #include "libopencm3/stm32/usart.h"
 #include "libopencm3/stm32/spi.h"
#include "fonts/font_fixedsys_mono_24.h"
#include "fonts/font_ubuntu_48.h"
#include "st7789_stm32_spi.h"
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/stm32/usart.h>
#include <errno.h>
#include "timing_stm32.h"
#include "intelhex.h"
#include <string.h>
#include <ctype.h>



 #define W25_CMD_MANUF_DEVICE   0x90
 #define W25_CMD_JEDEC_ID   0x9F
 #define W25_CMD_WRITE_EN   0x06
 #define W25_CMD_WRITE_DI   0x04
 #define W25_CMD_READ_SR1   0x05
 #define W25_CMD_READ_SR2   0x35
 #define W25_CMD_CHIP_ERASE   0xC7
 #define W25_CMD_READ_DATA   0x03
 #define W25_CMD_FAST_READ   0x0B
 #define W25_CMD_WRITE_DATA   0x02
 #define W25_CMD_READ_UID   0x4B
 #define W25_CMD_PWR_ON      0xAB
 #define W25_CMD_PWR_OFF      0xB9
 #define W25_CMD_ERA_SECTOR   0x20
 #define W25_CMD_ERA_32K      0x52
 #define W25_CMD_ERA_64K      0xD8


 #define DUMMY         0x00


 #define W25_SR1_BUSY      0x01
 #define W25_SR1_WEL      0x02



static const char *cap[3] = {
   "W25X16",   // 14
   "W25X32",   // 15
   "W25X64"   // 16
};   




void w25_wait(uint32_t spi) {
//   uint8_t res= w25_read_sr1(spi);
(void)spi;

//   while ( (res & W25_SR1_BUSY) )
   {
      	for (unsigned i = 0; i < 1500; i++)
	  {
		__asm__("nop");
	  }
//      res = w25_read_sr1(spi);
   }
}

uint8_t w25_read_sr1(uint32_t spi) {
   uint8_t sr1;

   gpio_clear(GPIOB, GPIO0);
   spi_xfer(spi,W25_CMD_READ_SR1);
   sr1 = spi_xfer(spi,DUMMY);
   gpio_set(GPIOB, GPIO0);
   return sr1;
}


uint8_t w25_read_sr2(uint32_t spi) {
   uint8_t sr1;

   gpio_clear(GPIOB, GPIO0);
   spi_xfer(spi,W25_CMD_READ_SR2);
   sr1 = spi_xfer(spi,DUMMY);
   gpio_set(GPIOB, GPIO0);
   return sr1;
}

bool w25_is_wprotect(uint32_t spi) {

	w25_wait(spi);
	return !(w25_read_sr1(spi) & W25_SR1_WEL);
}




bool w25_chip_erase(uint32_t spi) {

	if ( w25_is_wprotect(spi) ) {
		printf("Not Erased! Chip is not write enabled.\n");
		return false;
	}

	gpio_clear(GPIOB, GPIO0);
	spi_xfer(spi,W25_CMD_CHIP_ERASE);
	gpio_set(GPIOB, GPIO0);

	printf("Erasing chip..\n");

	if ( !w25_is_wprotect(spi) ) {
		printf("Not Erased! Chip erase failed.\n");
		return false;
	}

	printf("Chip erased!\n");
	return true;
}

uint32_t w25_read_data(uint32_t spi,uint32_t addr,void *data,uint32_t bytes) {
	uint8_t *udata = (uint8_t*)data;

	w25_wait(spi);

	gpio_clear(GPIOB, GPIO0);
	spi_xfer(spi,W25_CMD_FAST_READ);
	spi_xfer(spi,addr >> 16);
	spi_xfer(spi,(addr >> 8) & 0xFF);
	spi_xfer(spi,addr & 0xFF);
	spi_xfer(spi,DUMMY);

	for ( ; bytes-- > 0; ++addr )
		*udata++ = spi_xfer(spi,DUMMY);

	gpio_set(GPIOB, GPIO0);
	return addr;	
}


void w25_write_en(uint32_t spi,bool en) {

	w25_wait(spi);

	gpio_clear(GPIOB, GPIO0);
	spi_xfer(spi,en ? W25_CMD_WRITE_EN : W25_CMD_WRITE_DI);
	gpio_set(GPIOB, GPIO0);

	w25_wait(spi);
}


unsigned int w25_write_data(uint32_t spi,uint32_t addr,void *data,uint32_t bytes) {
	uint8_t *udata = (uint8_t*)data;

	w25_write_en(spi,true);
	w25_wait(spi);

	if ( w25_is_wprotect(spi) ) {
		printf("Write disabled.\n");
		return 0xFFFFFFFF;	// Indicate error
	}

	while ( bytes > 0 ) {
		gpio_clear(GPIOB, GPIO0);
		spi_xfer(spi,W25_CMD_WRITE_DATA);
		spi_xfer(spi,addr >> 16);
		spi_xfer(spi,(addr >> 8) & 0xFF);
		spi_xfer(spi,addr & 0xFF);
		while ( bytes > 0 ) {
			spi_xfer(spi,*udata++);
			--bytes;
			if ( (++addr & 0xFF) == 0x00 )
				break;
		}
		gpio_set(GPIOB, GPIO0);
	
		if ( bytes > 0 )
			w25_write_en(spi,true); // More to write
	}
	return addr;	
}

void w25_erase_block(uint32_t spi,uint32_t addr,uint8_t cmd) {
	const char *what;
	
	if ( w25_is_wprotect(spi) ) {
		printf("Write protected. Erase not performed.\n");
		return;
	}

	switch ( cmd ) {
	case W25_CMD_ERA_SECTOR:
		what = "sector";
		addr &= ~(4*1024-1);
		break;
	case W25_CMD_ERA_32K:
		what = "32K block";
		addr &= ~(32*1024-1);
		break;
	case W25_CMD_ERA_64K:
		what = "64K block";
		addr &= ~(64*1024-1);
		break;
	default:
		return;	// Should not happen
	}

	gpio_clear(GPIOB, GPIO0);
	spi_xfer(spi,cmd);
	spi_xfer(spi,addr >> 16);
	spi_xfer(spi,(addr >> 8) & 0xFF);
	spi_xfer(spi,addr & 0xFF);
	gpio_set(GPIOB, GPIO0);

	printf("%s erased, starting at %06X\n",
		what,(unsigned)addr);
}

void flash_status(void) {
	uint8_t s;

	s = w25_read_sr1(SPI3);
	printf("SR1 = %02X (%s)\n",
		s,
		s & W25_SR1_WEL
			? "write enabled"
			: "write protected");
	printf("SR2 = %02X\n",w25_read_sr2(SPI3));
}
/*
static unsigned
get_data24(const char *prompt) {
	unsigned v = 0u, count = 0u;
	char ch;

	printf("%s: ",prompt);

	while ( (ch = std_getc()) != '\r' && ch != '\n' ) {
		if ( ch == '\b' || ch == 0x7F ) {
			v >>= 4;
			printf("\b \b");
			if ( count > 0 )
				--count;
			continue;
		}
		if ( ch >= '0' && ch <= '9' ) {
			v <<= 4;
			v |= ch & 0x0F;
			printf("%d", ch);
		} else 	{
			if ( isalpha(ch) )
				ch = toupper(ch);
			if ( ch >= 'A' && ch <= 'F' ) {
				v <<= 4;
				v |= ((ch & 0x0F) - 1 + 10);
				printf("%d",ch);
			} else	{
				printf("?\b");
				continue;
			}
		}
		if ( ++count > 6 )
			break;
	}
	return v & 0xFFFFFF;
}

static unsigned
get_data8(const char *prompt) {
	unsigned v = 0u, count = 0u;
	char ch;

	if ( prompt )
		printf("%s: ",prompt);

	while ( (ch = std_getc()) != '\r' && ch != '\n' && !strchr(",./;\t",ch) ) {
		if ( ch == '"' || ch == '\'' ) {
			printf("%d",ch);
			v = std_getc();
			printf("%d", v);
			count = 1;
			break;
		}
		if ( ch == '\b' || ch == 0x7F ) {
			v >>= 4;
			printf("\b \b");
			if ( count > 0 )
				--count;
			continue;
		}
		if ( ch >= '0' && ch <= '9' ) {
			v <<= 4;
			v |= ch & 0x0F;
			printf("%d", ch);
		} else 	{
			if ( isalpha(ch) )
				ch = toupper(ch);
			if ( ch >= 'A' && ch <= 'F' ) {
				v <<= 4;
				v |= ((ch & 0x0F) - 1 + 10);
				printf("%d", ch);
			} else	{
				printf("?\b");
				continue;
			}
		}
		if ( ++count > 2 )
			break;
	}
	if ( !count )
		return 0xFFFF;	// No data
	return v & 0xFF;
}
*/
uint32_t dump_page(uint32_t spi,uint32_t addr) {
	char buf[17];

	addr &= ~0xFF;		// Start on page boundary

	for ( int x=0; x<16; ++x, addr += 16 ) {
		printf("%06X ",(unsigned)addr);
		w25_read_data(spi,addr,buf,16);
		for ( uint32_t offset=0; offset<16; ++offset )
			printf("%02X ",buf[offset]);
		for ( uint32_t offset=0; offset<16; ++offset ) {
			if ( buf[offset] < ' ' || buf[offset] >= 0x7F )
				printf(".");
			else	printf("%d",buf[offset]);
		}
		printf("\n");
	}
	return addr;
}
/*
void erase(uint32_t spi,uint32_t addr) {
	const char *what;
	char ch;

	if ( w25_is_wprotect(spi) ) {
		printf("Write protected. Erase not possible.\n");
		return;
	}

	printf(
		"\nErase what?\n"
		"  s ... Erase 4K sector\n"
		"  b ... Erase 32K block\n"
		"  z ... Erase 64K block\n"
		"  c ... Erase entire chip\n"
		"\nanything else to cancel\n: ");
		
	ch = std_getc();
	if ( isupper(ch) )
		ch = tolower(ch);

//	printf(ch);
	printf("%d", ch);
	printf("\n");

	switch ( ch ) {
	case 's':
		w25_erase_block(spi,addr,W25_CMD_ERA_SECTOR);
		what = "Sector";
		break;
	case 'b':
		w25_erase_block(spi,addr,W25_CMD_ERA_32K);
		what = "32K block";
		break;
	case 'z':
		w25_erase_block(spi,addr,W25_CMD_ERA_64K);
		what = "64K block";
		break;
	case 'c':
		w25_chip_erase(SPI3);
		return;
	default:
		printf("Erase CANCELLED.\n");
		return;
	}

	if ( w25_is_wprotect(spi) )
		printf("%s erased.\n",what);
	else	printf("%s FAILED.\n",what);
}

void load_ihex(uint32_t spi) {
	s_ihex ihex;
	char buf[200], ch;
	unsigned rtype, count = 0, ux;

	if ( w25_is_wprotect(spi) ) {
		printf("Flash is write protected.\n");
		return;
	}

	ihex_init(&ihex);
	printf("\nReady for Intel Hex upload:\n");

	for (;;) {
		printf("%08X ",(unsigned)ihex.compaddr);

		while ( (ch = std_getc()) != ':' ) {
			if ( ch == 0x1A || ch == 0x04 ) {
				printf("EOF\n");
				return;		// ^Z or ^D ends transmission
			}
		}
		buf[0] = ch;
//		printf(ch);
		printf("%d", ch);

		for (  ux=1; ux+1<sizeof buf; ++ux ) {
			buf[ux] = ch = std_getc();
			if ( ch == '\r' || ch == '\n' )
				break;
			if ( ch == 0x1A || ch == 0x04 ) {
				printf("(EOF)\n");
				return;		// ^Z or ^D ends transmission
			}
			printf("%d", ch);
		}
		buf[ux] = 0;		
		printf("\n");

		if ( !strchr(buf,':') ) {
			// Skip line with no hex
			continue;
		}

		rtype = ihex_parse(&ihex,buf);
		
		switch ( rtype ) {
		case IHEX_RT_DATA:	// data record
			w25_write_data(spi,ihex.addr&0x00FFFFFF,ihex.data,ihex.length);
			ihex.compaddr += ihex.length;
			break;
		case IHEX_RT_EOF:	// end	// of-file record
			break;
		case IHEX_RT_XSEG:	// extended segment address record
			break;
		case IHEX_RT_XLADDR:	// extended linear address record
			ihex.compaddr = ihex.baseaddr + ihex.addr;
			break;
		case IHEX_RT_SLADDR:	// start linear address record (MDK-ARM)
			break;
		default:
			printf("Error %02X: '%s'\n",(unsigned)rtype,buf);
			continue;
		}
		++count;
		
		if ( rtype == IHEX_RT_EOF )
			break;
		if ( strchr(buf,0x1A) || strchr(buf,0x04) )
			break;			// EOF from ascii-xfr
	}
}
*/
uint16_t w25_manuf_device(uint32_t spi) {
   uint16_t info;

   w25_wait(spi);
   gpio_clear(GPIOB, GPIO0);
   spi_xfer(spi,W25_CMD_MANUF_DEVICE);   // Byte 1
   spi_xfer(spi,DUMMY);         // Dummy1 (2)
   spi_xfer(spi,DUMMY);         // Dummy2 (3)
   spi_xfer(spi,0x00);         // Byte 4
   info = spi_xfer(spi,DUMMY) << 8;   // Byte 5
   info |= spi_xfer(spi,DUMMY);      // Byte 6
   gpio_set(GPIOB, GPIO0);

   return info;
}


uint32_t w25_JEDEC_ID(uint32_t spi) {
   uint32_t info;
   gpio_clear(GPIOB, GPIO0);
   w25_wait(spi);
//   gpio_clear(GPIOB, GPIO0);
   spi_xfer(spi,W25_CMD_JEDEC_ID);
	info = spi_xfer(spi,DUMMY);		 // Manuf.
	info = (info << 8) | spi_xfer(spi,DUMMY);// Memory Type
	info = (info << 8) | spi_xfer(spi,DUMMY);// Capacity
//   gpio_set(GPIOB, GPIO0);
   gpio_set(GPIOB, GPIO0);


   return info;
}


void w25_read_uid(uint32_t spi,void *buf,uint16_t bytes) {
   uint8_t *udata = (uint8_t*)buf;

   if ( bytes > 8 )
      bytes = 8;
   else if ( bytes <= 0 )
      return;

   w25_wait(spi);
   gpio_clear(GPIOB, GPIO0);
   spi_xfer(spi,W25_CMD_READ_UID);
   for ( uint8_t ux=0; ux<4; ++ux )
      spi_xfer(spi,DUMMY);
   for ( uint8_t ux=0; ux<bytes; ++ux )
      udata[ux] = spi_xfer(spi,DUMMY);
   gpio_set(GPIOB, GPIO0);

}

void spiflash_setup(void) {

static char dups[] = "000000";
static char mani[] = "000000";

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_CRC);

       gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
        GPIO3		
        | GPIO4	 
        | GPIO5	 

    );

    gpio_set_af(GPIOB, GPIO_AF6,
        GPIO3      
        | GPIO4   
        | GPIO5   

    );

gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO4 | GPIO5 | GPIO3);
		gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT,
						GPIO_PUPD_NONE, GPIO0);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
							GPIO0);
	gpio_set(GPIOB, GPIO0);

    rcc_periph_clock_enable(RCC_SPI3);

    spi_init_master(
        SPI3,
        SPI_CR1_BAUDRATE_FPCLK_DIV_4,
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,   
        SPI_CR1_CPHA_CLK_TRANSITION_1,     
        SPI_CR1_DFF_8BIT,
        SPI_CR1_MSBFIRST);

    spi_set_full_duplex_mode(SPI3);
    spi_disable_crc(SPI3);

    spi_set_nss_high(SPI3);
    SPI_I2SCFGR(SPI3) &= ~SPI_I2SCFGR_I2SMOD;


    spi_enable_software_slave_management(SPI3);
    spi_set_nss_high(SPI3);
    spi_enable_ss_output(SPI3);

    gpio_set(GPIOB, GPIO0);
    spi_enable(SPI3);
    
    printf("after spi3 init\n");
//   while (1)
//   {

      printf("start of while 1\n");
      uint8_t index = 0;
      uint32_t result1;

      spi_enable(SPI3);
      gpio_clear(GPIOB, GPIO0);
      w25_wait(SPI3);
//      spi_xfer(SPI3, 0x9F);
      gpio_set(GPIOB, GPIO0);
      gpio_set(GPIOB, GPIO0);
      gpio_clear(GPIOB, GPIO0);
      gpio_clear(GPIOB, GPIO0);
 
      result1 = w25_JEDEC_ID(SPI3);
      uint8_t result2 = (result1 << 8);
      uint8_t result3 = (result1 << 8);
      printf("result1 = %ld\n", result1);
      printf("result2 = %d\n", result2);
      printf("result3 = %d\n", result3);
      gpio_set(GPIOB, GPIO0);
      gpio_set(GPIOB, GPIO0);
//      st_draw_string_withbg(10, 25, (char *)result1, ST_COLOR_RED, ST_COLOR_PURPLE, &font_ubuntu_48);
//      st_draw_string_withbg(10, 45, (char *)result2, ST_COLOR_RED, ST_COLOR_PURPLE, &font_ubuntu_48);
//      st_draw_string_withbg(10, 65, (char *)result3, ST_COLOR_RED, ST_COLOR_PURPLE, &font_ubuntu_48);
      uint32_t poop = w25_JEDEC_ID(SPI3);
      sprintf(dups, "%02lx", poop);
//      (char *)dups = (char *)poop;
    
      printf("w25 jedec id = %02lx\n", poop);
      if (dups != 000000) {
     st_draw_string(10, 73, "FLASH FOUND jedec id;", ST_COLOR_PINK, &font_fixedsys_mono_24);
     st_draw_string(10, 94, dups, ST_COLOR_PURPLE, &font_fixedsys_mono_24);
     }
     uint16_t manu = w25_manuf_device(SPI3);
     printf("manufacturer = %02x\n", manu);
     printf("manufacturer = %d\n", manu);
//     char buf[8];
     uint16_t test3 = w25_manuf_device(SPI3);
     sprintf(mani, "%02lx", test3);
     if (test3) {
     st_draw_string(10, 114, mani, ST_COLOR_PURPLE, &font_fixedsys_mono_24);
     }
//      spi_disable(SPI3);
//      for (unsigned i = 0; i < 100; i++)
//	  {
//		__asm__("nop");
//	  }
//   }


}


