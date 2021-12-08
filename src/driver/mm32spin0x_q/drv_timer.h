// Define to prevent recursive inclusion
#ifndef __DRV_TIMER_H__
#define __DRV_TIMER_H__

#include "mm32_device.h"

void systick_init(void);
unsigned long millis(void);
void delay_us(uint32_t us);
void delay_ms(unsigned long ms);

#endif