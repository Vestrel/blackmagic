/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2012  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include "general.h"
#include "cdcacm.h"

#define USBUART_TIMER_FREQ_HZ 1000000U /* 1us per tick */
#define USBUART_RUN_FREQ_HZ 5000U /* 200us (or 100 characters at 2Mbps) */

#define RX_FIFO_SIZE 128

#define TX_FIFO_SIZE 128

/* TX FIFO buffer */
static uint8_t buf_tx[TX_FIFO_SIZE];
/* FIFO in pointer, writes assumed to be atomic, should be only incremented outside TX ISR */
static uint8_t buf_tx_in;
/* FIFO out pointer, writes assumed to be atomic, should be only incremented within TX ISR */
static uint8_t buf_tx_out;
/* RX Fifo buffer */
static uint8_t buf_rx[RX_FIFO_SIZE];
/* Fifo in pointer, writes assumed to be atomic, should be only incremented within RX ISR */
static uint8_t buf_rx_in;
/* Fifo out pointer, writes assumed to be atomic, should be only incremented outside RX ISR */
static uint8_t buf_rx_out;

static void usbuart_run(void);

void usbuart_init(void)
{
	rcc_periph_clock_enable(USBUSART_CLK);

	UART_PIN_SETUP();

	/* Setup UART parameters. */
	usart_set_baudrate(USBUSART, 38400);
	usart_set_databits(USBUSART, 8);
	usart_set_stopbits(USBUSART, USART_STOPBITS_1);
	usart_set_mode(USBUSART, USART_MODE_TX_RX);
	usart_set_parity(USBUSART, USART_PARITY_NONE);
	usart_set_flow_control(USBUSART, USART_FLOWCONTROL_NONE);

	/* Enable interrupts */
	USBUSART_CR1 |= USART_CR1_RXNEIE;
	nvic_set_priority(USBUSART_IRQ, IRQ_PRI_USBUSART);
	nvic_enable_irq(USBUSART_IRQ);

	/* Finally enable the USART. */
	usart_enable(USBUSART);

	/* Setup timer for running deferred FIFO processing */
	USBUSART_TIM_CLK_EN();
	timer_set_mode(USBUSART_TIM, TIM_CR1_CKD_CK_INT,
			TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(USBUSART_TIM,
			rcc_apb2_frequency / USBUART_TIMER_FREQ_HZ * 2 - 1);
	timer_set_period(USBUSART_TIM,
			USBUART_TIMER_FREQ_HZ / USBUART_RUN_FREQ_HZ - 1);

	/* Setup update interrupt in NVIC */
	nvic_set_priority(USBUSART_TIM_IRQ, IRQ_PRI_USBUSART_TIM);
	nvic_enable_irq(USBUSART_TIM_IRQ);

	/* turn the timer on */
	timer_enable_counter(USBUSART_TIM);
}

/*
 * Runs deferred processing for usb uart rx, draining RX FIFO by sending
 * characters to host PC via CDCACM.  Allowed to read from FIFO in pointer,
 * but not write to it. Allowed to write to FIFO out pointer.
 */
static void usbuart_run(void)
{
	/* Forcibly empty fifo if no USB endpoint.
	 * If fifo empty, nothing further to do. */
	cm_disable_interrupts();
	if (cdcacm_get_config() != 1 || buf_rx_in == buf_rx_out) {
		buf_rx_out = buf_rx_in;
		cm_enable_interrupts();
		/* turn off LED, disable IRQ */
		timer_disable_irq(USBUSART_TIM, TIM_DIER_UIE);
		gpio_clear(LED_PORT_UART, LED_UART);
	}
	else
	{
		cm_enable_interrupts();

		uint8_t packet_buf[CDCACM_PACKET_SIZE];
		uint8_t packet_size = 0;
		uint8_t buf_out = buf_rx_out;

		/* copy from uart FIFO into local usb packet buffer */
		while (buf_rx_in != buf_out && packet_size < CDCACM_PACKET_SIZE)
		{
			packet_buf[packet_size++] = buf_rx[buf_out++];

			/* wrap out pointer */
			if (buf_out >= RX_FIFO_SIZE)
			{
				buf_out = 0;
			}
		}

		/* advance fifo out pointer by amount written */
		const uint16_t written = usbd_ep_write_packet(usbdev,
				CDCACM_UART_ENDPOINT, packet_buf, packet_size);
		cm_disable_interrupts();
		buf_rx_out = (buf_rx_out + written) % RX_FIFO_SIZE;
		cm_enable_interrupts();
	}
}

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{
	usart_set_baudrate(USBUSART, coding->dwDTERate);

	if (coding->bParityType)
		usart_set_databits(USBUSART, coding->bDataBits + 1);
	else
		usart_set_databits(USBUSART, coding->bDataBits);

	switch(coding->bCharFormat) {
	case 0:
		usart_set_stopbits(USBUSART, USART_STOPBITS_1);
		break;
	case 1:
		usart_set_stopbits(USBUSART, USART_STOPBITS_1_5);
		break;
	case 2:
		usart_set_stopbits(USBUSART, USART_STOPBITS_2);
		break;
	}

	switch(coding->bParityType) {
	case 0:
		usart_set_parity(USBUSART, USART_PARITY_NONE);
		break;
	case 1:
		usart_set_parity(USBUSART, USART_PARITY_ODD);
		break;
	case 2:
		usart_set_parity(USBUSART, USART_PARITY_EVEN);
		break;
	}
}

void usbuart_usb_out_cb(usbd_device *dev, uint8_t ep)
{
	(void)ep;

	usbd_ep_nak_set(dev, CDCACM_UART_ENDPOINT, 1);
	char buf[CDCACM_PACKET_SIZE];
	const uint16_t len = usbd_ep_read_packet(dev, CDCACM_UART_ENDPOINT,
						buf, CDCACM_PACKET_SIZE);

#if defined(BLACKMAGIC)
	/* Don't bother if uart is disabled.
	 * This will be the case on mini while we're being debugged.
	 */
	if(!(RCC_APB2ENR & RCC_APB2ENR_USART1EN))
	{
		usbd_ep_nak_set(dev, CDCACM_UART_ENDPOINT, 0);
		return;
	}
#endif

	if (len)
	{
		uint16_t copy_idx = 0;

		/* copy data to TX FIFO */
		while (copy_idx < len)
		{
			buf_tx[buf_tx_in++] = buf[copy_idx++];

			if (buf_tx_in >= TX_FIFO_SIZE)
				buf_tx_in = 0;
		}

		/* enable UART TX IRQ and LED */
		USBUSART_CR1 |= USART_CR1_TXEIE;
		gpio_set(LED_PORT_UART, LED_UART);
	}

	/* enable USBUART TX packet reception if FIFO has enough space */
	if (!len || (buf_tx_out - buf_tx_in) % TX_FIFO_SIZE > CDCACM_PACKET_SIZE)
		usbd_ep_nak_set(dev, CDCACM_UART_ENDPOINT, 0);
}

#ifdef USBUART_DEBUG
int usbuart_debug_write(const char *buf, size_t len)
{
	cm_disable_interrupts();
	for (size_t i = 0; i < len || (buf_rx_in + 1) % RX_FIFO_SIZE != buf_rx_out; i++) {
		if (buf[i] == '\n') {
			buf_rx[buf_rx_in++] = '\r';
			buf_rx_in %= RX_FIFO_SIZE;
		}
		buf_rx[buf_rx_in++] = buf[i];
		buf_rx_in %= RX_FIFO_SIZE;
	}
	cm_enable_interrupts();
	/* enable deferred processing if we put data in the FIFO */
	timer_enable_irq(USBUSART_TIM, TIM_DIER_UIE);
	return len;
}
#endif

void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep)
{
	(void) dev;
	(void) ep;
}

/*
 * Read a character from the UART RX and stuff it in a software RX FIFO.
 * Allowed to read from RX FIFO out pointer, but not write to it.
 * Allowed to write to RX FIFO in pointer.
 * Read a character from the TX FIFO and stuff it in a UART TX.
 * Allowed to read from TX FIFO in pointer, but not write to it.
 * Allowed to write to TX FIFO out pointer.
 */
static void usbuart_service()
{
	const uint32_t status = USART_SR(USBUSART);

	/* Send data to UART from TX FIFO */
	if (status & USART_FLAG_TXE)
	{
		if (buf_tx_out != buf_tx_in)
		{
			usart_send(USBUSART, buf_tx[buf_tx_out]);
			buf_tx_out = (buf_tx_out + 1) % TX_FIFO_SIZE;
		}
		else
		{
			/* no data left, disable IRQ and LED */
			USBUSART_CR1 &= ~USART_CR1_TXEIE;
			gpio_clear(LED_PORT_UART, LED_UART);
		}
		
		/* enable USBUART TX packet reception if FIFO has enough space */
		if (buf_tx_out == buf_tx_in || (buf_tx_out - buf_tx_in) % TX_FIFO_SIZE > CDCACM_PACKET_SIZE)
			usbd_ep_nak_set(usbdev, CDCACM_UART_ENDPOINT, 0);
	}

	const char c = usart_recv(USBUSART);
#if !defined(USART_SR_NE) && defined(USART_ISR_NF)
# define USART_SR_NE USART_ISR_NF
#endif
	if (status & (USART_FLAG_ORE | USART_FLAG_FE | USART_SR_NE | USART_FLAG_PE) || !(status & USART_FLAG_RXNE))
		return;

	/* Turn on LED */
	gpio_set(LED_PORT_UART, LED_UART);

	/* If the next increment of rx_in would put it at the same point
	* as rx_out, the FIFO is considered full.
	*/
	if (((buf_rx_in + 1) % RX_FIFO_SIZE) != buf_rx_out)
	{
		/* insert into FIFO */
		buf_rx[buf_rx_in++] = c;

		/* wrap out pointer */
		if (buf_rx_in >= RX_FIFO_SIZE)
		{
			buf_rx_in = 0;
		}

		/* enable deferred processing if we put data in the FIFO */
		timer_enable_irq(USBUSART_TIM, TIM_DIER_UIE);
	}
}

void USBUSART_ISR(void)
{
	/* receive/send data over UART */
	cm_disable_interrupts();
	usbuart_service();
	cm_enable_interrupts();
}

void USBUSART_TIM_ISR(void)
{
	/* need to clear timer update event */
	timer_clear_flag(USBUSART_TIM, TIM_SR_UIF);

	/* process FIFO */
	usbuart_run();
}

#ifdef ENABLE_DEBUG
enum {
	RDI_SYS_OPEN = 0x01,
	RDI_SYS_WRITE = 0x05,
	RDI_SYS_ISTTY = 0x09,
};

int rdi_write(int fn, const char *buf, size_t len)
{
	(void)fn;
#if defined(PLATFORM_HAS_DEBUG)
	if (debug_bmp)
		return len - usbuart_debug_write(buf, len);
#else
	(void)buf;
	(void)len;
#endif
	return 0;
}

struct ex_frame {
	union {
		int syscall;
		int retval;
	};
	const int *params;
	uint32_t r2, r3, r12, lr, pc;
};

void debug_monitor_handler_c(struct ex_frame *sp)
{
	/* Return to after breakpoint instruction */
	sp->pc += 2;

	switch (sp->syscall) {
	case RDI_SYS_OPEN:
		sp->retval = 1;
		break;
	case RDI_SYS_WRITE:
		sp->retval = rdi_write(sp->params[0], (void*)sp->params[1], sp->params[2]);
		break;
	case RDI_SYS_ISTTY:
		sp->retval = 1;
		break;
	default:
		sp->retval = -1;
	}

}

asm(".globl debug_monitor_handler\n"
    ".thumb_func\n"
    "debug_monitor_handler: \n"
    "    mov r0, sp\n"
    "    b debug_monitor_handler_c\n");

#endif
