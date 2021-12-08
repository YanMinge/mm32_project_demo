#ifndef __DRV_COMMON_DEF_H__
#define __DRV_COMMON_DEF_H__

#include <stdint.h>


#define GPIO_BASE                    (0x48000000UL)

#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
#define BITS(start, end)             ((0xFFFFFFFFUL << (start)) & (0xFFFFFFFFUL >> (31U - (uint32_t)(end)))) 
#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))


#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs_user(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define line_map(value, target_min, target_max, source_min, source_max) constrain((target_min + (value - source_min)*(target_max - target_min)/(source_max - source_min)), target_min, target_max)

#endif //__DRV_COMMON_DEF_H__
