#ifndef __DRV_GPIO_H__
#define __DRV_GPIO_H__

#include <stdint.h>
#include "hal_conf.h"
#include "mm32_device.h"
#include "drv_common_def.h"

#define HIGH        1
#define LOW         0

//Reference voltage, the unit is: V
#define ADC_REFVOLATGE            3.3f

// Several sets of channel data are collected
#define ADC_SCANNUM               3

#define ADC_AVERAGE_LEN           10

#define GPIO_PORT_ADDRESS(port)     (GPIO_TypeDef*)(GPIO_BASE + (port << 10))

typedef enum
{
    PA_0   =   0x00,
    PA_1   =   0x01,
    PA_2   =   0x02,
    PA_3   =   0x03,
    PA_4   =   0x04,
    PA_5   =   0x05,
    PA_6   =   0x06,
    PA_7   =   0x07,
    PA_8   =   0x08,
    PA_9   =   0x09,
    PA_10  =   0x0a,
    PA_11  =   0x0b,
    PA_12  =   0x0c,
    PA_13  =   0x0d,
    PA_14  =   0x0e,
    PA_15  =   0x0f, 
    PB_0   =   0x10,
    PB_1   =   0x11,
    PB_2   =   0x12,
    PB_3   =   0x13,
    PB_4   =   0x14,
    PB_5   =   0x15,
    PB_6   =   0x16,
    PB_7   =   0x17,
    PB_8   =   0x18,
    PB_9   =   0x19,
    PB_10  =   0x1a,
    PB_11  =   0x1b,
    PB_12  =   0x1c,
    PB_13  =   0x1d,
    PB_14  =   0x1e,
    PB_15  =   0x1f,
}gpio_pin_type;

typedef enum
{
    PORTA   =   0x00,
    PORTB   =   0x01,
}gpio_port_type;

typedef enum
{
    GPIO_MODE_OUTPUT = GPIO_Mode_Out_PP,
    GPIO_MODE_INPUT_ANALOG =  GPIO_Mode_AIN,
    GPIO_MODE_INPUT_NONE = GPIO_Mode_IN_FLOATING,
    GPIO_MODE_INPUT_PULLUP = GPIO_Mode_IPU,
    GPIO_MODE_INPUT_PULLDOWN = GPIO_Mode_IPD,
}gpio_mode_type;

void drv_gpio_pin_mode(gpio_pin_type pin,gpio_mode_type mode);
void drv_gpio_digital_write(gpio_pin_type  pin, uint8_t val);
int drv_gpio_digital_read(gpio_pin_type pin);
int drv_gpio_pwm_mode(gpio_pin_type pin, uint16_t frequency);
int drv_gpio_analog_write(gpio_pin_type pin, uint16_t analog_val);
void adc_update(void);
int drv_gpio_analog_read(gpio_pin_type pin);
#endif
