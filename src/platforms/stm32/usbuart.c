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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include <libopencm3/cm3/dwt.h>
#include <libopencm3/stm32/st_usbfs.h>

#include "general.h"
#include "cdcacm.h"

#define RX_FIFO_SIZE 128
#define TX_BUF_SIZE 128

/* TX double buffer */
static uint8_t buf_tx[TX_BUF_SIZE * 2];
/* Active buffer part idx */
static uint8_t buf_tx_act_idx;
/* Active buffer part used capacity */
static uint8_t buf_tx_act_sz;
/* TX transfer complete */
static bool tx_trfr_cplt = true;
/* RX Fifo buffer */
static uint8_t buf_rx[RX_FIFO_SIZE];
/* RX Fifo out pointer, writes assumed to be atomic */
static uint8_t buf_rx_out;
/* RX usb transfer complete */
static bool rx_usb_trfr_cplt = true;

void usbuart_init(void)
{
	/* Enable clocks */
	rcc_periph_clock_enable(USBUSART_CLK);
	rcc_periph_clock_enable(USBUSART_DMA_CLK);

	/* Setup UART parameters. */
	UART_PIN_SETUP();
	usart_set_baudrate(USBUSART, 38400);
	usart_set_databits(USBUSART, 8);
	usart_set_stopbits(USBUSART, USART_STOPBITS_1);
	usart_set_mode(USBUSART, USART_MODE_TX_RX);
	usart_set_parity(USBUSART, USART_PARITY_NONE);
	usart_set_flow_control(USBUSART, USART_FLOWCONTROL_NONE);
	USBUSART_CR1 |= USART_CR1_IDLEIE;

	/* Setup USART TX DMA */
	dma_channel_reset(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN);
	dma_set_peripheral_address(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN, (uint32_t)&USBUSART_DR);
	dma_set_read_from_memory(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN);
	dma_enable_memory_increment_mode(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN);
	dma_set_peripheral_size(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN, DMA_CCR_PL_HIGH);
	dma_enable_transfer_complete_interrupt(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN);

	/* Setup USART RX DMA */
	dma_channel_reset(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN);
	dma_set_peripheral_address(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN, (uint32_t)&USBUSART_DR);
	dma_set_memory_address(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN, (uint32_t)buf_rx);
	dma_set_number_of_data(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN, RX_FIFO_SIZE);
	dma_set_read_from_peripheral(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN);
	dma_enable_memory_increment_mode(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN);
	dma_enable_circular_mode(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN);
	dma_set_peripheral_size(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN, DMA_CCR_PL_HIGH);
	dma_enable_half_transfer_interrupt(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN);
	dma_enable_transfer_complete_interrupt(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN);
	dma_enable_channel(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN);

	/* Enable interrupts */
	nvic_set_priority(USBUSART_IRQ, IRQ_PRI_USBUSART);
	nvic_set_priority(USBUSART_DMA_TX_IRQ, IRQ_PRI_USBUSART_DMA);
	nvic_set_priority(USBUSART_DMA_RX_IRQ, IRQ_PRI_USBUSART_DMA);
	nvic_enable_irq(USBUSART_IRQ);
	nvic_enable_irq(USBUSART_DMA_TX_IRQ);
	nvic_enable_irq(USBUSART_DMA_RX_IRQ);

	/* Finally enable the USART. */
	usart_enable(USBUSART);
	usart_enable_tx_dma(USBUSART);
	usart_enable_rx_dma(USBUSART);

	dwt_enable_cycle_counter();
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

/* 
 * Changes USBUSART TX buffer in which data is accumulated from USB.
 * Filled buffer is submitted to DMA for transfer.
 */
static void usbusart_change_dma_tx_buf(void)
{
	/* Select buffer for transmission */
	uint8_t *const tx_buf_ptr = &buf_tx[buf_tx_act_idx * TX_BUF_SIZE];

	/* Configure DMA */
	dma_set_memory_address(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN, (uint32_t)tx_buf_ptr);
	dma_set_number_of_data(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN, buf_tx_act_sz);
	dma_enable_channel(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN);

	/* Change active buffer */
	buf_tx_act_sz = 0;
	buf_tx_act_idx ^= 1;
}

void usbuart_usb_out_cb(usbd_device *dev, uint8_t ep)
{
	(void)ep;

	usbd_ep_nak_set(dev, CDCACM_UART_ENDPOINT, 1);
	
	/* Read new packet directly into TX buffer */
	uint8_t *const tx_buf_ptr = &buf_tx[buf_tx_act_idx * TX_BUF_SIZE];
	const uint16_t len = usbd_ep_read_packet(dev, CDCACM_UART_ENDPOINT,
						tx_buf_ptr + buf_tx_act_sz, CDCACM_PACKET_SIZE);

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
		buf_tx_act_sz += len;

		/* If DMA is idle, schedule new transfer */
		if (tx_trfr_cplt)
		{
			tx_trfr_cplt = false;
			usbusart_change_dma_tx_buf();

			/* Enable LED */
			gpio_set(LED_PORT_UART, LED_UART);
		}
	}

	/* Enable USBUART TX packet reception if buffer has enough space */
	if (TX_BUF_SIZE - buf_tx_act_sz >= CDCACM_PACKET_SIZE)
		usbd_ep_nak_set(dev, CDCACM_UART_ENDPOINT, 0);
}

#ifdef USBUART_DEBUG
int usbuart_debug_write(const char *buf, size_t len)
{
	cm_disable_interrupts();
	for (size_t i = 0; i < len && (buf_rx_in + 1) % RX_FIFO_SIZE != buf_rx_out; i++) {
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

uint32_t lastCycCnt = 0;
uint32_t canTX = 0;

/*
 * Runs deferred processing for USBUSART RX, draining RX FIFO by sending
 * characters to host PC via CDCACM. Allowed to write to FIFO OUT pointer.
 */
static void usbusart_send_rx_packet(void)
{
	rx_usb_trfr_cplt = false;
	/* Calculate writing position in the FIFO */
	const uint32_t buf_rx_in = (RX_FIFO_SIZE - dma_get_number_of_data(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN)) % RX_FIFO_SIZE;

	/* Forcibly empty fifo if no USB endpoint.
	 * If fifo empty, nothing further to do. */
	if (cdcacm_get_config() != 1 || buf_rx_in == buf_rx_out)
	{
		buf_rx_out = buf_rx_in;
		/* Turn off LED */
		gpio_clear(LED_PORT_UART, LED_UART); // TODO: make more complex LED mngment system
		rx_usb_trfr_cplt = true;
	}
	else
	{
		lastCycCnt = dwt_read_cycle_counter();
		/* To avoid the need of sending ZLP don't transmit full packet */
		uint8_t packet_buf[CDCACM_PACKET_SIZE - 1];
		uint32_t packet_size = 0;

		/* Copy from uart RX FIFO into local usb packet buffer */
		for (uint32_t buf_out = buf_rx_out; buf_out != buf_rx_in && packet_size < sizeof(packet_buf); buf_out %= RX_FIFO_SIZE)
			packet_buf[packet_size++] = buf_rx[buf_out++];

		/* advance fifo out pointer by amount written */
		const uint16_t written = usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT, packet_buf, packet_size);
		buf_rx_out = (buf_rx_out + written) % RX_FIFO_SIZE;
		// canTX += written;

		if (written) {
			canTX++;
		}
	}
}

void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep)
{
	(void) ep;
	(void) dev;
	
	usbusart_send_rx_packet();
}

static void usbuart_run(void)
{
	nvic_disable_irq(USB_IRQ);

	// if (dwt_read_cycle_counter() - lastCycCnt >= 30000) {
		// if ((*USB_EP_REG(CDCACM_UART_ENDPOINT) & USB_EP_TX_STAT) == USB_EP_TX_STAT_VALID) {
			// canTX++;
		// }
	// }

	/* Try to send a packet if usb is idle */
	if (rx_usb_trfr_cplt)
		usbusart_send_rx_packet();

	nvic_enable_irq(USB_IRQ);
}

void USBUSART_ISR(void)
{
	nvic_disable_irq(USBUSART_DMA_RX_IRQ);

	/* Get IDLE flag and reset interrupt flags */
	const bool isIdle = usart_get_flag(USBUSART, USART_FLAG_IDLE);
	usart_recv(USBUSART);

	/* If line is now idle, then transmit a packet */
	if (isIdle)
		usbuart_run();

	nvic_enable_irq(USBUSART_DMA_RX_IRQ);
}

void USBUSART_DMA_TX_ISR(void)
{
	nvic_disable_irq(USB_IRQ);

	/* Stop DMA */
	dma_disable_channel(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN);
	dma_clear_interrupt_flags(USBUSART_DMA_BUS, USBUSART_DMA_TX_CHAN, DMA_IFCR_CGIF_BIT);

	/* If new buffer is ready, continue transmission.
	 * Otherwise report transfer completion.
	 */
	if (buf_tx_act_sz)
	{
		usbusart_change_dma_tx_buf();
		usbd_ep_nak_set(usbdev, CDCACM_UART_ENDPOINT, 0);
	}
	else
	{
		gpio_clear(LED_PORT_UART, LED_UART);
		tx_trfr_cplt = true;
	}
	
	nvic_enable_irq(USB_IRQ);
}

void USBUSART_DMA_RX_ISR(void)
{
	nvic_disable_irq(USBUSART_IRQ);

	/* Clear flags */
	dma_clear_interrupt_flags(USBUSART_DMA_BUS, USBUSART_DMA_RX_CHAN, DMA_IFCR_CGIF_BIT);
	/* Transmit a packet */
	usbuart_run();

	nvic_enable_irq(USBUSART_IRQ);
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
