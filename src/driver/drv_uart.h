#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#include <stdint.h>
#include "mm32_device.h"
#include "hal_conf.h"
#include "drv_ring_buf.h"

#if 0

#define DEBUG_MSG(...)   uart_printf(UART0_PORT, __VA_ARGS__); uart_printf(UART1_PORT, __VA_ARGS__) 
#else
#define DEBUG_MSG(...)
#endif

void uart_ringbuf_init(void);
void drv_uart1_init(uint32_t baud_rate);
void drv_uart2_init(uint32_t baud_rate);
void drv_uart_write_byte(UART_TypeDef* uart, uint8_t inputData);
void send_string(UART_TypeDef* uart, uint8_t *str, uint8_t len);
void drv_uart_printf(UART_TypeDef* uart, char *fmt,...);

RING_BUF_DEF_STRUCT s_tx_ring_buf;

#endif
