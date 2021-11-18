#include "drv_timer.h"
#include "hal_conf.h"

volatile unsigned long system_time = 0;

void systick_init(void)
{
    system_time = 0;
    if(SysTick_Config(RCC_GetSysClockFreq() / 1000))
    {
        while(1);
    }
    NVIC_SetPriority(SysTick_IRQn, 3);
}


void SysTick_Handler(void)
{
     system_time++;
}

unsigned long millis(void)
{
    return system_time;
}

void delay_us(uint32_t us)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;
    
    reload = SysTick->LOAD;
    ticks = us * (RCC_GetSysClockFreq() / 1000000);
    tcnt = 0;
    told = SysTick->VAL;
    while(1)
    {
        tnow = SysTick->VAL;
        if(tnow != told)
        {
            if(tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if(tcnt >= ticks)
            {
                break;
            }
        }
    }  
}

void delay_ms(unsigned long ms)
{
    unsigned long start_time = millis();
    while((millis() - start_time) <= ms) 
    {
        //do nothing
    }
}