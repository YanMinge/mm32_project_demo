#include "stdio.h"
#include "stdint.h"
#include "drv_gpio.h"
#include "drv_uart.h"
#include "drv_ring_buf.h"
#include "drv_timer.h"

int main(void)
{
    systick_init();
    uart_ringbuf_init();
    delay_ms(2000);
    drv_uart1_init(115200);
    // drv_uart2_init(115200);

    drv_gpio_pin_mode(PB_5, GPIO_MODE_OUTPUT);

    // drv_gpio_pin_mode(PB_7, GPIO_MODE_OUTPUT);
    // drv_gpio_digital_write(PB_7, HIGH);

    drv_gpio_pin_mode(PA_0, GPIO_MODE_INPUT_ANALOG);
    drv_gpio_pin_mode(PA_1, GPIO_MODE_INPUT_ANALOG);
    drv_gpio_pin_mode(PA_4, GPIO_MODE_INPUT_ANALOG);
    drv_gpio_pin_mode(PA_5, GPIO_MODE_INPUT_ANALOG);

    // drv_gpio_pin_mode(PB_6, GPIO_MODE_OUTPUT);
    // drv_gpio_digital_write(PB_6, HIGH);
    // drv_gpio_pwm_mode(PA_6, 5000);
    // drv_gpio_analog_write(PA_6, 500);

    // drv_gpio_pin_mode(PA_6, GPIO_MODE_OUTPUT);
    // drv_gpio_digital_write(PA_6, HIGH);
    // drv_gpio_pwm_mode(PB_6, 5000);
    // drv_gpio_analog_write(PB_6, 500);

    drv_uart_printf(UART1, "hello yanminge\r\n");
    // uint16_t count = 0;
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    unsigned long current_time = millis();
    while(1)
    {
        if(millis() - current_time > 1000)
        {
            static bool led_state = LOW;
            led_state = !led_state;
            drv_gpio_digital_write(PB_5, led_state);
            drv_uart_printf(UART1, "PB_5 led_state: %d, adc read: %d, %d, %d, %d\n", led_state, drv_gpio_analog_read(PA_0), drv_gpio_analog_read(PA_1), drv_gpio_analog_read(PA_4), drv_gpio_analog_read(PA_5));
            current_time = millis();
        }
        adc_update();
        delay_ms(5);
    }
    return 0;
}
