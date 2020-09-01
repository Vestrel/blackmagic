/*
 * This file is part of the Black Magic Debug project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <libopencm3/lm4f/gpio.h>

#include "timing.h"
#include "version.h"

#define PLATFORM_HAS_TRACESWO

#define BOARD_IDENT         "Black Magic Probe (Launchpad ICDI), (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_DFU		"Black Magic (Upgrade) for Launchpad, (Firmware " FIRMWARE_VERSION ")"
#define DFU_IDENT           "Black Magic Firmware Upgrade (Launchpad)"
#define DFU_IFACE_STRING	"lolwut"

#define TDI_PORT	GPIOA
#define TMS_PORT	GPIOA
#define TCK_PORT	GPIOA
#define TDO_PORT	GPIOA
#define TDI_PIN		GPIO5
#define TMS_PIN		GPIO3
#define TCK_PIN		GPIO2
#define TDO_PIN		GPIO4

#define SWO_PORT	GPIOD
#define SWO_PIN		GPIO6

#define SWDIO_PORT	TMS_PORT
#define SWCLK_PORT	TCK_PORT
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

#define SRST_PORT	GPIOA
#define SRST_PIN	GPIO6

#define TMS_SET_MODE() do {								\
	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TMS_PIN);		\
	gpio_set_output_config(TMS_PORT, GPIO_OTYPE_PP, GPIO_DRIVE_2MA, TMS_PIN);	\
} while (0)

#define SWDIO_MODE_FLOAT() do {								\
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SWDIO_PIN);	\
} while (0)

#define SWDIO_MODE_DRIVE() do {									\
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SWDIO_PIN);		\
	gpio_set_output_config(SWDIO_PORT, GPIO_OTYPE_PP, GPIO_DRIVE_2MA, SWDIO_PIN);		\
} while (0)

#define UART_PIN_SETUP() do {								\
	periph_clock_enable(USBUSART_PORT_CLK);							\
	__asm__("nop"); __asm__("nop"); __asm__("nop");					\
	gpio_set_af(USBUSART_PORT, 0x1, USBUSART_RX_PIN | USBUSART_TX_PIN);				\
	gpio_mode_setup(USBUSART_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, USBUSART_RX_PIN);	\
	gpio_mode_setup(USBUSART_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, USBUSART_TX_PIN);	\
} while (0)

#define USB_DRIVER	lm4f_usb_driver
#define USB_IRQ		NVIC_USB0_IRQ
#define USB_ISR		usb0_isr
/* Interrupt priorities.  Low numbers are high priority. */
#define IRQ_PRI_USB		        (1 << 4)
#define IRQ_PRI_USBUSART	    (2 << 4)

#define USBUSART			UART0
#define USBUSART_IRQ		NVIC_UART0_IRQ
#define USBUSART_CLK		RCC_UART0
#define USBUSART_PORT_CLK	RCC_GPIOA
#define USBUSART_PORT 		GPIOA
#define USBUSART_RX_PIN 	GPIO0
#define USBUSART_TX_PIN 	GPIO1
#define USBSUART_ISR		uart0_isr

#define TRACEUART	UART2
#define TRACEUART_CLK	RCC_UART2
#define TRACEUART_IRQ	NVIC_UART2_IRQ
#define TRACEUART_ISR	uart2_isr

extern uint8_t running_status;
#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{}
#define SET_ERROR_STATE(state)	SET_IDLE_STATE(state)

inline static void gpio_set_val(uint32_t port, uint8_t pin, uint8_t val) {
	gpio_write(port, pin, val == 0 ? 0 : 0xff);
}

inline static uint8_t gpio_get(uint32_t port, uint8_t pin) {
	return !(gpio_read(port, pin) == 0);
}

#define disconnect_usb() do { usbd_disconnect(usbdev,1); nvic_disable_irq(USB_IRQ);} while(0)

static inline int platform_hwversion(void)
{
	return 0;
}

/*
 * Use newlib provided integer only stdio functions
 */

/* sscanf */
#ifdef sscanf
#undef sscanf
#define sscanf siscanf
#else
#define sscanf siscanf
#endif
/* sprintf */
#ifdef sprintf
#undef sprintf
#define sprintf siprintf
#else
#define sprintf siprintf
#endif
/* vasprintf */
#ifdef vasprintf
#undef vasprintf
#define vasprintf vasiprintf
#else
#define vasprintf vasiprintf
#endif
/* snprintf */
#ifdef snprintf
#undef snprintf
#define snprintf sniprintf
#else
#define snprintf sniprintf
#endif

#endif
