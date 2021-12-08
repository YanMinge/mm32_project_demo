#include "drv_gpio.h"
#include "drv_common_def.h"
#include "hal_conf.h"

static uint32_t pwm_divination = 0;
static bool ADCflag = false;
static bool ADCTrigFilterflag = false;
static bool dma_init_flag = false;
static uint16_t ADCValue[ADC_AVERAGE_LEN];
static uint16_t varADCavarage[ADC_AVERAGE_LEN][ADC_SCANNUM];
static uint16_t ADCFilterValue[ADC_SCANNUM];
static float ADCVolatge[ADC_SCANNUM];

static void adc_single_channel_init(uint32_t channel)
{
    ADC_InitTypeDef  ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);

    //Initialize PA1 to analog input mode
    //Enable ADC clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    //ADC prescale factor
    ADC_InitStructure.ADC_PRESCARE = ADC_PCLK2_PRESCARE_16;
    //Set ADC mode to continuous conversion mode
    ADC_InitStructure.ADC_Mode = ADC_Mode_Continue;
    //AD data right-justified
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_Init(ADC1, &ADC_InitStructure);

    //Enable the channel
    ADC_RegularChannelConfig(ADC1, channel, 0, ADC_Samctl_239_5);

    //Enable ADCDMA
    ADC_DMACmd(ADC1, ENABLE);
    //Enable AD conversion
    ADC_Cmd(ADC1, ENABLE);                                                    //Enable AD conversion
}

static void adc_filter(void)
{
    uint16_t channel;
    uint16_t cntFilter;
    uint32_t lADCFilterValue[ADC_AVERAGE_LEN] = {0, 0, 0};
    for(channel = 0; channel < ADC_SCANNUM; channel++)
    {
        for(cntFilter = 0; cntFilter < ADC_AVERAGE_LEN; cntFilter++)
        {
            lADCFilterValue[channel] += varADCavarage[cntFilter][channel];
        }
        ADCFilterValue[channel] = (lADCFilterValue[channel] * 30 / ADC_AVERAGE_LEN + ADCFilterValue[channel] * 70) / 100;
    }
}

void get_adc_volatge(void)
{
    uint16_t channel;
    for(channel = 0; channel < ADC_SCANNUM; channel++)
    {
        ADCVolatge[channel] = ((float)ADCFilterValue[channel] / 4095) * ADC_REFVOLATGE;
    }
}

static void dma_init(void)
{
    if(dma_init_flag == true)
    {
        return;
    }
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel1);
    DMA_StructInit(&DMA_InitStructure);
    //DMA transfer peripheral address
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (ADC1->DR);
    //DMA transfer memory address
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCValue;
    //DMA transfer direction from peripheral to memory
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    //DMA cache size
    DMA_InitStructure.DMA_BufferSize = ADC_SCANNUM;
    //After receiving the data, the peripheral address is forbidden to move
    //backward
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //After receiving the data, the memory address is shifted backward
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //Define the peripheral data width to 16 bits
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    //Define the memory data width to 16 bits
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //Cycle conversion mode
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //DMA priority is high
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //M2M mode is disabled
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_Auto_reload = DMA_Auto_Reload_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    //DMA interrupt initialization
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    dma_init_flag = true;
}

void DMA1_Channel1_IRQHandler(void)
{
    //Stop Conversion
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);
    //Clear interrupt flag
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    //Erected transmission complete flag
    ADCflag = true;

}

void drv_gpio_pin_mode(gpio_pin_type pin, gpio_mode_type mode)
{
    uint32_t port_num, pin_num;
    GPIO_InitTypeDef  GPIO_InitStructure;

    port_num = pin >> 4;
    pin_num = BIT(pin & 0x0f);

    /* enable the GPIO port */
    if(port_num == PORTA)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    }
    else
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    }

    /* configure led GPIO port */
    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  =  pin_num;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_Init(GPIO_PORT_ADDRESS(port_num), &GPIO_InitStructure);

    if(mode == GPIO_MODE_INPUT_ANALOG)
    {
        dma_init();
        switch(pin)
        {
            case PA_0:
                adc_single_channel_init(ADC_Channel_0);
                break;
            case PA_1:
                adc_single_channel_init(ADC_Channel_1);
                break;
            case PA_2:
                adc_single_channel_init(ADC_Channel_2);
                break;
            case PA_3:
                adc_single_channel_init(ADC_Channel_3);
                break;
            case PA_4:
                adc_single_channel_init(ADC_Channel_4);
                break;
            case PA_5:
                adc_single_channel_init(ADC_Channel_5);
                break;
            case PA_6:
                adc_single_channel_init(ADC_Channel_6);
                break;
            case PA_7:
                adc_single_channel_init(ADC_Channel_7);
                break;
            case PB_0:
                adc_single_channel_init(ADC_Channel_8);
                break;
            case PB_1:
                adc_single_channel_init(ADC_Channel_9);
                break;
            default:
                break;        
        }
    }
}

void drv_gpio_digital_write(gpio_pin_type pin, uint8_t val)
{
    uint32_t port_num, pin_num;

    port_num = pin >> 4;
    pin_num = BIT(pin & 0x0f);

    if(val == HIGH)
    {
        GPIO_SetBits(GPIO_PORT_ADDRESS(port_num), pin_num);
    }
    else
    {
        GPIO_ResetBits(GPIO_PORT_ADDRESS(port_num), pin_num);
    }
}

int drv_gpio_digital_read(gpio_pin_type pin)
{
    uint32_t port_num, pin_num;

    port_num = pin >> 4;
    pin_num = BIT(pin & 0x0f);
    return GPIO_ReadInputDataBit(GPIO_PORT_ADDRESS(port_num), pin_num);
}

int drv_gpio_pwm_mode(gpio_pin_type pin, uint16_t frequency)
{
    uint32_t port_num, pin_num;

    port_num = pin >> 4;
    pin_num = BIT(pin & 0x0f);
    GPIO_InitTypeDef GPIO_InitStructure;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* enable the GPIO port */
    if(port_num == PORTA)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    }
    else
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    }

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    switch(pin)
    {
        case PA_0:
             break;
        case PA_4:
             break;
        case PA_5:
             break;
        case PA_6:       //TIM3_CH1
             /* GPIO configure */
             GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
             GPIO_InitStructure.GPIO_Pin  =  pin_num;
             GPIO_Init(GPIOA, &GPIO_InitStructure);
             
             /* Timer configure */
             RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

             TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
             TIM_TimeBaseStructure.TIM_Period = (9000000 / frequency) - 1;
             pwm_divination = TIM_TimeBaseStructure.TIM_Period;
             TIM_TimeBaseStructure.TIM_Prescaler = 7;
             //Setting Clock Segmentation
             TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
             TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
             ///TIM Upward Counting Mode
             TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
             TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

             TIM_OCStructInit(&TIM_OCInitStructure);
             //Select Timer Mode: TIM Pulse Width Modulation Mode 2
             TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
             TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
             //Setting the Pulse Value of the Capture Comparison Register to be Loaded
             TIM_OCInitStructure.TIM_Pulse = 0;
             //Output polarity: TIM output is more polar
             TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
             TIM_OC1Init(TIM3, &TIM_OCInitStructure);

             TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
             TIM_ARRPreloadConfig(TIM3, ENABLE);
             TIM_CtrlPWMOutputs(TIM3, ENABLE);

             TIM_Cmd(TIM3, ENABLE);
             break;
        case PB_0:
             break;
        case PB_1:
             break;
        case PB_3:
             break;
        case PB_4:
             break;
        case PB_6:       //TIM2_CH1
             /* GPIO configure */
             GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_4);
             GPIO_InitStructure.GPIO_Pin  =  pin_num;
             GPIO_Init(GPIOB, &GPIO_InitStructure);
             
             /* Timer configure */
             RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

             TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
             TIM_TimeBaseStructure.TIM_Period = (9000000 / frequency) - 1;
             pwm_divination = TIM_TimeBaseStructure.TIM_Period;
             TIM_TimeBaseStructure.TIM_Prescaler = 7;
             //Setting Clock Segmentation
             TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
             TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
             ///TIM Upward Counting Mode
             TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
             TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

             TIM_OCStructInit(&TIM_OCInitStructure);
             //Select Timer Mode: TIM Pulse Width Modulation Mode 2
             TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
             TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
             //Setting the Pulse Value of the Capture Comparison Register to be Loaded
             TIM_OCInitStructure.TIM_Pulse = 0;
             //Output polarity: TIM output is more polar
             TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
             TIM_OC1Init(TIM2, &TIM_OCInitStructure);

             TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
             TIM_ARRPreloadConfig(TIM2, ENABLE);
             TIM_CtrlPWMOutputs(TIM2, ENABLE);

             TIM_Cmd(TIM2, ENABLE);
             break;
        case PB_7:
             break;
        case PB_13:
             break;
        case PB_14:
             break;
        default:
             return -1;

    }
    return 0;
}

int drv_gpio_analog_write(gpio_pin_type pin, uint16_t analog_val)
{
    uint16_t capture_compare_val = 0;
    switch(pin)
    {
        case PA_0:
             break;
        case PA_4:
             break;
        case PA_5:
             break;
        case PA_6:       //TIM3_CH1
             capture_compare_val = line_map(analog_val, 0, pwm_divination, 0, 1000);
             TIM_SetCompare1(TIM3, capture_compare_val);
             break;
        case PB_0:
             break;
        case PB_1:
             break;
        case PB_3:
             break;
        case PB_4:
             break;
        case PB_6:       //TIM2_CH1
             capture_compare_val = line_map(analog_val, 0, pwm_divination, 0, 1000);
             TIM_SetCompare1(TIM2, capture_compare_val);
        case PB_7:
             break;
        case PB_13:
             break;
        case PB_14:
             break;
        default:
             return -1;

    }
    return 0;
}

void adc_update(void)
{
    static uint16_t channel;
    static uint16_t averagenumber;
    if(ADCflag == true)
    {
          ADCflag = false;
          for(channel = 0; channel < ADC_SCANNUM; channel++)
          {
               varADCavarage[averagenumber][channel] = ADCValue[channel];
          }
          ADC_SoftwareStartConvCmd(ADC1, ENABLE);
          averagenumber++;
          if(averagenumber >= ADC_AVERAGE_LEN) 
          {
               averagenumber = 0;
               ADCTrigFilterflag = true;
          }
     }
     //Determine whether the second-order filter is over
     if(ADCTrigFilterflag)
     {
          adc_filter();
          //Clear the filter end flag
          ADCTrigFilterflag = false;
          //Convert the filtered value to voltage
          get_adc_volatge();
     }
}

int drv_gpio_analog_read(gpio_pin_type pin)
{
    uint16_t analog_val;
    switch(pin)
    {
        case PA_0:
             //adc_channel_enable(ADC1, ADC_Channel_0);
             analog_val = ADCFilterValue[0];
             break;
        case PA_5:
             //adc_channel_enable(ADC1, ADC_Channel_5);
             analog_val = ADCFilterValue[1];
             break;
        case PB_0:
            // adc_channel_enable(ADC1, ADC_Channel_8);
             analog_val = ADCFilterValue[2];
             break;
        default:
             return -1;
    }
    return analog_val;
}
