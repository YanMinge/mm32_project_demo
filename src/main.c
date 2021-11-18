#include "stdio.h"
#include "stdint.h"
#include "drv_gpio.h"
#include "drv_uart.h"
#include "drv_ring_buf.h"
#include "drv_timer.h"

// GPIOＡ.13 GPIOA.14 is used for JTAG, Please use it cautiously!!!


void message_process()
{
    if(drv_ringbuf_count((RING_BUF_DEF_STRUCT*)&s_rx_ring_buf) != 0)
    {
        uint8_t rx_buf[1] = {0};
        drv_ringbuf_read((RING_BUF_DEF_STRUCT*)&s_rx_ring_buf, 1, rx_buf);
        if((tx_buf[0] == 'G') || (tx_buf[0] == 'g'))
        drv_uart_write_byte(UART2, rx_buf[0]);
    }
}

int main(void)
{
    systick_init();
    uart_ringbuf_init();
    delay_ms(2000);
    drv_uart1_init(115200);
    drv_uart2_init(115200);

    drv_gpio_pin_mode(PB_1, GPIO_MODE_OUTPUT);

    // drv_gpio_pin_mode(PB_7, GPIO_MODE_OUTPUT);
    // drv_gpio_digital_write(PB_7, HIGH);

    drv_gpio_pin_mode(PA_0, GPIO_MODE_INPUT_ANALOG);
    drv_gpio_pin_mode(PA_5, GPIO_MODE_INPUT_ANALOG);
    drv_gpio_pin_mode(PB_0, GPIO_MODE_INPUT_ANALOG);

    // drv_gpio_pin_mode(PB_6, GPIO_MODE_OUTPUT);
    // drv_gpio_digital_write(PB_6, HIGH);
    // drv_gpio_pwm_mode(PA_6, 5000);
    // drv_gpio_analog_write(PA_6, 500);

    // drv_gpio_pin_mode(PA_6, GPIO_MODE_OUTPUT);
    // drv_gpio_digital_write(PA_6, HIGH);
    // drv_gpio_pwm_mode(PB_6, 5000);
    // drv_gpio_analog_write(PB_6, 500);

    log_uart_printf(UART2, "hello yanminge\r\n");
    // uint16_t count = 0;
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    unsigned long current_time = millis();
    while(1)
    {
        // drv_gpio_digital_write(PB_1, HIGH);
        // drv_uart_printf(UART1, "PB_1 HIGH %d\n", drv_gpio_analog_read(PB_0));
        // delay_ms(1000);
        // drv_gpio_digital_write(PB_1, LOW);
        // drv_uart_printf(UART1, "PB_1 LOW %d\r\n", drv_gpio_analog_read(PA_5));
        // delay_ms(1000);

        // count++;
        // if(count == 100)
        // {
        //     log_uart_printf(UART1, "yanminge test v=%d,t=%d,i=%d\r\n", drv_gpio_analog_read(PA_0), drv_gpio_analog_read(PA_5), drv_gpio_analog_read(PB_0));
        //     count = 0;
        // }
        if(millis() - current_time > 10)
        {
            adc_update();
        }
    }
    return 0;
}
