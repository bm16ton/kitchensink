#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/stm32/spi.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <pwm.h>
#ifdef ENABLE_NEOPIXEL
#include "ws2812_spi.h"
#endif
#include "usb.h"
#ifdef BLACKPILLV2
#define GPIO1_PORT   GPIOC      //LED
#define GPIO1_PIN    GPIO13     //LED
#define GPIO2_PORT   GPIOB
#define GPIO2_PIN    GPIO2
#define GPIO3_PORT   GPIOB       //BTN
#define GPIO3_PIN    GPIO14       //BTN
#define GPIO4_PORT   GPIOD
#define GPIO4_PIN    GPIO1
#define GPIO5_PORT   GPIOA
#define GPIO5_PIN    GPIO4
#define GPIO6_PORT
#define GPIO6_PIN
#define GPIO7_PORT
#define GPIO7_PIN
#define GPIO8_PORT
#define GPIO8_PIN


#define PWM0_PORT    GPIOC
#define PWM0_PIN     GPIO6
#define PWM1_PORT    GPIOC
#define PWM1_PIN     GPIO7
#define PWM2_PORT    GPIOC
#define PWM2_PIN     GPIO8

#define PWM3_PORT    GPIOC
#define PWM3_PIN     GPIO0
#endif
#ifdef F103
#define GPIO1_PORT   GPIOC      //LED
#define GPIO1_PIN    GPIO0     //LED
#define GPIO2_PORT   GPIOB
#define GPIO2_PIN    GPIO13
#define GPIO3_PORT   GPIOC       //BTN
#define GPIO3_PIN    GPIO14       //BTN
#define GPIO4_PORT   GPIOC
#define GPIO4_PIN    GPIO15
#define GPIO5_PORT   GPIOA
#define GPIO5_PIN    GPIO3
#endif
#define LED1_PORT
#define LED1_PIN

#define IRQ_TYPE_NONE		0
#define IRQ_TYPE_EDGE_RISING	0x00000001
#define IRQ_TYPE_EDGE_FALLING	0x00000002
#define IRQ_TYPE_EDGE_BOTH	IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING
#define IRQ_TYPE_LEVEL_HIGH	0x00000004
#define IRQ_TYPE_LEVEL_LOW	0x00000008
	
int irqtype = EXTI_TRIGGER_FALLING;

static void my_delay_1( void )
{
   for (unsigned i = 0; i < 800000; i++)
     {
        __asm__("nop");
     }
}

static void my_delay_2(void)
{
	for (unsigned i = 0; i < 20000; i++)
	  {
		__asm__("nop");
	  }
}

#ifdef BLACKPILLV2
static void irq_pin_init(void)
{
//    nvic_disable_irq(NVIC_EXTI0_IRQ);
    #ifdef ENABLE_NEOPIXEL
    ws2812_write_rgb(SPI1, 220, 180, 0);
    #endif
	my_delay_2();
    nvic_enable_irq(NVIC_EXTI4_IRQ);					
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	my_delay_2();
	exti_select_source(EXTI4, GPIO5_PORT);
    exti_set_trigger(EXTI4, irqtype);
	exti_enable_request(EXTI4);
}
/*
static void irq_none(void)
{
    nvic_disable_irq(NVIC_EXTI4_IRQ);
	my_delay_2();				
	exti_disable_request(EXTI4);
	my_delay_2();
}
*/
#endif

#ifdef F103
static void irq_pin_init(void)
{
//    nvic_disable_irq(NVIC_EXTI0_IRQ);
#ifdef ENABLE_NEOPIXEL
    ws2812_write_rgb(SPI1, 220, 180, 0);
#endif
	my_delay_2();
    nvic_enable_irq(NVIC_EXTI3_IRQ);					
	gpio_set_mode(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5_PIN);

	my_delay_2();
	exti_select_source(EXTI3, GPIO5_PORT);
//	state.falling = false;
//	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    exti_set_trigger(EXTI3, irqtype);
	exti_enable_request(EXTI3);
}

#endif

#ifdef BLACKPILLV2
static void pwm_probe(void)
{
    pwm_init();
    pwm_set_frequency(1000000);
	pwm_set_dc(PWM_CH1, 0);
	pwm_set_dc(PWM_CH2, 0);
	pwm_set_dc(PWM_CH3, 0);

//	pwm_set_dc(PWM_CH4, 0);

	pwm_set_dc(PWM_CH4, 0);

	pwm_start();
	my_delay_2();
    pwm_set_frequency(1000000);
	pwm_set_dc(PWM_CH1, 100);
	pwm_set_dc(PWM_CH2, 200);
	pwm_set_dc(PWM_CH3, 300);

//	pwm_set_dc(PWM_CH4, 500);

	pwm_set_dc(PWM_CH4, 500);

	my_delay_2();

}	


static void pwm_disable(void)
{
	timer_set_oc_value(TIM8, TIM_OC1, 0);
	timer_set_oc_value(TIM8, TIM_OC2, 0);
	timer_set_oc_value(TIM8, TIM_OC3, 0);

//	timer_set_oc_value(TIM8, TIM_OC4, 0);
	pwm_set_dc(PWM_CH1, 0);
	pwm_set_dc(PWM_CH2, 0);
	pwm_set_dc(PWM_CH3, 0);
//	pwm_set_dc(PWM_CH4, 0);
	timer_disable_oc_output(TIM8, TIM_OC1);
	timer_disable_oc_output(TIM8, TIM_OC2);
	timer_disable_oc_output(TIM8, TIM_OC3);
//	timer_disable_oc_output(TIM8, TIM_OC4);


	my_delay_2();
}	
#endif

#ifdef F103
static void pwm_probe(void)
{
//    pwm_setup_timer(RCC_TIM3, TIM3, 2, 1000);
//    pwm_setup_output_channel(TIM3, TIM_OC1, RCC_APB2ENR, GPIOA, GPIO8);
//    pwm_set_pulse_width(TIM3, 0, 800);
//    pwm_setup_output_channel(TIM3, TIM_OC2, RCC_APB2ENR, GPIOA, GPIO9);
//    pwm_set_pulse_width(TIM3, 1, 400);
//    pwm_setup_output_channel(TIM3, TIM_OC3, RCC_APB2ENR, GPIOA, GPIO10);
//    pwm_set_pulse_width(TIM3, 2, 200);
//    pwm_start_timer(TIM3);
/*    pwm_init();
    pwm_set_frequency(1000);
	pwm_set_dc(PWM_CH1, 0);
	pwm_set_dc(PWM_CH2, 0);
	pwm_set_dc(PWM_CH3, 0);
	pwm_start();
	my_delay_2();
    pwm_set_frequency(1000);
	pwm_set_dc(PWM_CH1, 500);
	pwm_set_dc(PWM_CH2, 500);
	pwm_set_dc(PWM_CH3, 500);
	my_delay_2();
	*/

}	
	
static void pwm_disable(void)
{
/*	timer_set_oc_value(TIM1, TIM_OC2, 0);
	timer_set_oc_value(TIM1, TIM_OC3, 0);
	pwm_set_dc(PWM_CH1, 0);
	pwm_set_dc(PWM_CH2, 0);
	pwm_set_dc(PWM_CH3, 0);
	timer_disable_oc_output(TIM1, TIM_OC1);
	timer_disable_oc_output(TIM1, TIM_OC2);
	timer_disable_oc_output(TIM1, TIM_OC3);
	my_delay_2();
	* */
}	
#endif

#ifdef BLACKPILLV2
static void usbgpio_output(int gpio)
{
	if (gpio == 1) {
	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1_PIN);
	gpio_set_output_options(GPIO1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
    } else if (gpio == 2) {
	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	gpio_set_output_options(GPIO2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_set_output_options(GPIO3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set_output_options(GPIO4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	gpio_set_output_options(GPIO5_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}
	
    my_delay_1();
}

static void usbgpio_input(int gpio)
{

	if (gpio == 1) {
	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
	} else if (gpio == 2) {
	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}

	my_delay_1();
}
#endif

#ifdef F103
static void usbgpio_output(int gpio)
{
	if (gpio == 1) {
	gpio_set_mode(GPIO1_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
    } else if (gpio == 2) {
	gpio_set_mode(GPIO2_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_set_mode(GPIO3_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_set_mode(GPIO4_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_set_mode(GPIO5_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}
	
    my_delay_1();
}

static void usbgpio_input(int gpio)
{

	if (gpio == 1) {
	gpio_set_mode(GPIO1_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
	} else if (gpio == 2) {
	gpio_set_mode(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_set_mode(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_set_mode(GPIO4_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_set_mode(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}

	my_delay_1();
}
#endif

static enum usbd_request_return_codes usb_control_gpio_request(
    usbd_device *dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *, struct usb_setup_data *req))
{
    (void)complete;
	(void)dev;

   if ((req->bmRequestType & 0x7F) != USB_REQ_TYPE_VENDOR)
     return USBD_REQ_NEXT_CALLBACK;

    
//    printf("bmrequesttype %d\n", req->bmRequestType);
    
   (*len) = 1;
   (*buf)[0] = 1; //success

   if (req->wValue == 1)
     {
        if ( req->wIndex == 0 )
			{
				usbgpio_input(1);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				usbgpio_input(2);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 2 )
			{
				usbgpio_input(3);
				return USBD_REQ_HANDLED;
			}	
	    else if ( req->wIndex == 3 )
			{
				usbgpio_input(4);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				usbgpio_input(5);
				return USBD_REQ_HANDLED;
			}	
      }
   else if (req->wValue == 2)
     {
     if ( req->wIndex == 0 )
			{
				usbgpio_output(1);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				usbgpio_output(2);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 2 )
			{
				usbgpio_output(3);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 3 )
			{
				usbgpio_output(4);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				usbgpio_output(5);
				return USBD_REQ_HANDLED;
			}
      }
   else if (req->wValue == 3)
     {
     if ( req->wIndex == 0 )
			{
            if (gpio_get(GPIO1_PORT, GPIO1_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
            if (gpio_get(GPIO2_PORT, GPIO2_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
		    return USBD_REQ_HANDLED;
		   }
	    else if ( req->wIndex == 2 )
			{
            if (gpio_get(GPIO3_PORT, GPIO3_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
		 else if ( req->wIndex == 3 )
			{
            if (gpio_get(GPIO4_PORT, GPIO4_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
            if (gpio_get(GPIO5_PORT, GPIO5_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = 4;
			    return 1;
			} else {
				(*buf)[0] = 0; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = 4;
			    return 1;
			}
			return USBD_REQ_HANDLED;
			}
         }
  else if (req->wValue == 0)
     { 
	 if (req->bRequest == 1)
        {
        if ( req->wIndex == 0 )
			{
				gpio_set(GPIO1_PORT, GPIO1_PIN);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				gpio_set(GPIO2_PORT, GPIO2_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 2 )
			{
				gpio_set(GPIO3_PORT, GPIO3_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 3 )
			{
				gpio_set(GPIO4_PORT, GPIO4_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				gpio_set(GPIO5_PORT, GPIO5_PIN);
				return USBD_REQ_HANDLED;
			}
        }
   else if (req->bRequest == 0)
     {
     if (req->wIndex == 0)
			{
				gpio_clear(GPIO1_PORT, GPIO1_PIN);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				gpio_clear(GPIO2_PORT, GPIO2_PIN);
				return USBD_REQ_HANDLED;
			}	
		else if ( req->wIndex == 2 )
			{
				gpio_clear(GPIO3_PORT, GPIO3_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 3 )
			{
				gpio_clear(GPIO4_PORT, GPIO4_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				gpio_clear(GPIO5_PORT, GPIO5_PIN);
				return USBD_REQ_HANDLED;
			}	
	  }
    }
       else if (req->wValue == 4)
     { 
     if (req->wIndex == 1) {
          pwm_probe();
          return USBD_REQ_HANDLED;
     } else if (req->wIndex == 2) {
          pwm_disable();
          return USBD_REQ_HANDLED;
       }
     }
    else if (req->wValue == 5)
     { 
      if ( req->wIndex == 0 ) {
//        pwm_set_dc(PWM_CH1, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }	    
      else if ( req->wIndex == 104 ) {
//        pwm_set_dc(PWM_CH2, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }
      else if ( req->wIndex == 208 ) {
//        pwm_set_dc(PWM_CH3, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }
      else if ( req->wIndex == 56 ) {
//        pwm_set_dc(PWM_CH4, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }
        return USBD_REQ_HANDLED;
     }
    else if (req->wValue == 9)
     {
     if ( req->wIndex == 1 ) {
        irqtype = IRQ_TYPE_NONE;
//        irq_none();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 2 ) {
        irqtype = IRQ_TYPE_LEVEL_HIGH;
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 3 ) {
        irqtype = IRQ_TYPE_LEVEL_LOW;
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 4 ) {
//        irqtype = IRQ_TYPE_EDGE_BOTH;
        irqtype = EXTI_TRIGGER_BOTH;   
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 5 ) {
//        irqtype = IRQ_TYPE_EDGE_RISING;
        irqtype = EXTI_TRIGGER_RISING;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 6 ) {
//        irqtype = IRQ_TYPE_EDGE_FALLING;
        irqtype = EXTI_TRIGGER_FALLING;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     }
     else
     {
   return USBD_REQ_NEXT_CALLBACK;
//      (*buf)[0] = -1; // FAILURE
     }
//    return 1;
//     return 1;
   /*
 {
    return USBD_REQ_NEXT_CALLBACK;
    }
    */
  return USBD_REQ_NEXT_CALLBACK;
}

void gpio_set_config(usbd_device *dev, uint16_t wValue)
{
    (void)wValue;
    
    usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
    
	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_VENDOR,
				USB_REQ_TYPE_TYPE,
				usb_control_gpio_request);
}

#ifdef F103
//usbd_device *usbd_dev;
void exti3_isr(void)
{
    // char buf2[64] __attribute__ ((aligned(4)));
//    uint8_t buft[4] = {3, 3, 3, 3};
    static uint8_t buft[4];
    buft[0] = 1; 
	buft[1] = 3;
	buft[2] = 3;
	buft[3] = 3;
	exti_reset_request(EXTI3);
//	usbd_ep_write_packet(usbd_device usbd_dev, 0x83, buf2, 64);
    usbd_ep_write_packet(usbdev, 0x82, buft, sizeof(buft));
    exti_set_trigger(EXTI3, irqtype);
}
#endif

#ifdef BLACKPILLV2
void exti4_isr(void)
{
    // char buf2[64] __attribute__ ((aligned(4)));
    uint8_t buft[4] __attribute__ ((aligned(2))) = {3, 3, 3, 3};
    #ifdef ENABLE_NEOPIXEL
    ws2812_write_rgb(SPI1, 220, 0, 220);
    #endif
	exti_reset_request(EXTI4);
//	usbd_ep_write_packet(usbd_device usbd_dev, 0x83, buf2, 64);
    usbd_ep_write_packet(usbdev, 0x82, buft, 4);
    gpio_toggle(GPIOC, GPIO13);
    exti_set_trigger(EXTI4, irqtype);
}
#endif

#ifdef F103
void usbgpio_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
	
	gpio_set_mode(GPIO1_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1_PIN);
	gpio_set_mode(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2_PIN);
	gpio_set_mode(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3_PIN);
	gpio_set_mode(GPIO4_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4_PIN);
	gpio_set_mode(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5_PIN);

    my_delay_1();
	gpio_clear(GPIO1_PORT, GPIO1_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);

}
#endif

#ifdef BLACKPILLV2
void usbgpio_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_SYSCFG);
	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1_PIN);
	gpio_set_output_options(GPIO1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO1_PIN);

	gpio_set(GPIO1_PORT, GPIO1_PIN);



	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set_output_options(GPIO4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO4_PIN);
		
				
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	
    my_delay_1();
	gpio_clear(GPIO1_PORT, GPIO1_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);

}
#endif
