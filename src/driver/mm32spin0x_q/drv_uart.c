#include <stdio.h>
#include "string.h"
#include <stdarg.h>
#include "stdlib.h"
#include "drv_ring_buf.h"
#include "drv_uart.h"
#include "hal_conf.h"

#define DATA_TX_BUFSIZE      64
#define DATA_RX_BUFSIZE      64

static RING_BUF_DEF_STRUCT s_tx_ring_buf;
volatile uint8_t txcount = 0; 
static uint8_t s_link_tx_buf[DATA_TX_BUFSIZE];

RING_BUF_DEF_STRUCT s_rx_ring_buf;
static uint8_t s_link_rx_buf[DATA_RX_BUFSIZE];

void uart_ringbuf_init(void)
{
    drv_ringbuf_init((RING_BUF_DEF_STRUCT*)&s_tx_ring_buf, s_link_tx_buf, DATA_TX_BUFSIZE);
    drv_ringbuf_init((RING_BUF_DEF_STRUCT*)&s_rx_ring_buf, s_link_rx_buf, DATA_RX_BUFSIZE);
}

void drv_uart1_init(uint32_t baud_rate)
{   
    /* GPIO port set */
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART1, ENABLE);      //enable UART1
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);        //enable GPIOB

    /* UART initial set */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_3);

    //UART1 NVIC
    NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    UART_StructInit(&UART_InitStructure);
    UART_InitStructure.UART_BaudRate = baud_rate;
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;                              //one stopbit
    UART_InitStructure.UART_Parity = UART_Parity_No;                                 //none odd-even  verify bit
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;     //No hardware flow control
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;                      //receive and sent mode

    UART_Init(UART1, &UART_InitStructure); //initial uart 1
    UART_ITConfig(UART1, UART_IT_TXIEN | UART_IT_RXIEN | UART_OVER_ERR, ENABLE);
    UART_Cmd(UART1, ENABLE);               //enable uart 1

    /* Enable Half duplex*/
    UART_HalfDuplexCmd(UART1, ENABLE);
    // UART_HalfDuplexCmd(UART1, DISABLE);

    /* GPIO initial set */
    //UART1_TX   GPIOB.3
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //UART1_RX    GPIOB.4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void drv_uart2_init(uint32_t baud_rate)
{   
    /* GPIO port set */
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_UART2, ENABLE);      //enable UART2
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);        //enable GPIOA
    RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOB, ENABLE);        //enable GPIOB

    /* UART initial set */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);

    //UART2 NVIC
    NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    UART_StructInit(&UART_InitStructure);
    UART_InitStructure.UART_BaudRate = baud_rate;
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;                              //one stopbit
    UART_InitStructure.UART_Parity = UART_Parity_No;                                 //none odd-even  verify bit
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;     //No hardware flow control
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;                      //receive and sent mode

    UART_Init(UART2, &UART_InitStructure); //initial uart 2
    // UART_ITConfig(UART2, UART_IT_TXIEN | UART_IT_RXIEN | UART_OVER_ERR, ENABLE);
    UART_Cmd(UART2, ENABLE);               //enable uart 2

    /* GPIO initial set */
    //UART2_TX   GPIOB.7
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //UART2_RX    GPIOA.6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void drv_uart_write_byte(UART_TypeDef* uart, uint8_t inputData)
{
    UART_SendData(uart, inputData & (u16)0x00FF);
    while(!UART_GetFlagStatus(uart, UART_FLAG_TXEPT));
}

void uart_send_bytes(UART_TypeDef* uart, uint8_t *bytes, uint8_t len)
{
    drv_ringbuf_write(&s_tx_ring_buf, bytes, len);
    UART_ITConfig(uart, UART_IT_TXIEN, ENABLE);
}

void drv_uart_printf(UART_TypeDef* uart, char *fmt,...)
{
    va_list ap;
    char string[64]; 
    va_start(ap,fmt);
    vsprintf(string,fmt,ap);
    uart_send_bytes(uart, (uint8_t*)string, strlen(string));
    va_end(ap);
}

void send_log(UART_TypeDef* uart, char *str)
{
    while(*str)
    {
        drv_uart_write_byte(uart, *str++);
    }
}

void log_uart_printf(UART_TypeDef* uart, char *fmt,...)
{
    va_list ap;
    char string[64];
    UART_ITConfig(uart, UART_IT_TXIEN, DISABLE);
    va_start(ap,fmt);
    vsprintf(string,fmt,ap);
    send_log(uart, string);
    va_end(ap);
}

void UART1_IRQHandler(void)
{
    uint8_t receive_data = 0xff;
    if(RESET != UART_GetITStatus(UART1, UART_ISR_RX))
    {
        UART_ClearITPendingBit(UART1, UART_ISR_RX);
        receive_data = UART_ReceiveData(UART1);
        drv_ringbuf_write((RING_BUF_DEF_STRUCT*)&s_rx_ring_buf, &receive_data, 1);
    }
    else if(RESET != UART_GetITStatus(UART1, UART_ISR_RXOERR))
    {
        UART_ClearITPendingBit(UART1, UART_ISR_RXOERR);
    }

    if(RESET != UART_GetITStatus(UART1, UART_IT_TXIEN))
    {
        UART_ClearITPendingBit(UART1, UART_IT_TXIEN);
        if(drv_ringbuf_count((RING_BUF_DEF_STRUCT*)&s_tx_ring_buf) == 0)
        {
            txcount = 0;
            UART_ITConfig(UART1, UART_IT_TXIEN, DISABLE);
            while (!UART_GetFlagStatus(UART1, UART_CSR_TXC));
            drv_ringbuf_flush((RING_BUF_DEF_STRUCT*)&s_tx_ring_buf);
        }
        else
        {
            uint8_t tx_buf[1] = {0};
            drv_ringbuf_read((RING_BUF_DEF_STRUCT*)&s_tx_ring_buf, 1, tx_buf);
            drv_uart_write_byte(UART1, tx_buf[0]);
        }
    }
}

void UART2_IRQHandler(void)
{
    uint8_t receive_data = 0xff;
    if(RESET != UART_GetITStatus(UART2, UART_ISR_RX))
    {
        UART_ClearITPendingBit(UART2, UART_ISR_RX);
        receive_data = UART_ReceiveData(UART2);
        drv_ringbuf_write((RING_BUF_DEF_STRUCT*)&s_rx_ring_buf, &receive_data, 1);
    }
    else if(RESET != UART_GetITStatus(UART2, UART_ISR_RXOERR))
    {
        UART_ClearITPendingBit(UART2, UART_ISR_RXOERR);
    }

    if(RESET != UART_GetITStatus(UART2, UART_IT_TXIEN))
    {
        UART_ClearITPendingBit(UART2, UART_IT_TXIEN);
        if(drv_ringbuf_count((RING_BUF_DEF_STRUCT*)&s_tx_ring_buf) == 0)
        {
            txcount = 0;
            UART_ITConfig(UART2, UART_IT_TXIEN, DISABLE);
            while (!UART_GetFlagStatus(UART2, UART_CSR_TXC));
            drv_ringbuf_flush((RING_BUF_DEF_STRUCT*)&s_tx_ring_buf);
        }
        else
        {
            uint8_t tx_buf[1] = {0};
            drv_ringbuf_read((RING_BUF_DEF_STRUCT*)&s_tx_ring_buf, 1, tx_buf);
            drv_uart_write_byte(UART2, tx_buf[0]);
        }
    }
}

