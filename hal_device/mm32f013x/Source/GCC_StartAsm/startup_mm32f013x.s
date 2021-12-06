/**************************************************************************//**
 * @file     startup_mm32f013x.s
 * @brief    CMSIS-Core(M) Device Startup File for Cortex-M0 Device
 * @version  V2.0.1
 * @date     23. July 2019
 ******************************************************************************/
/*
 * Copyright (c) 2009-2019 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

                .syntax  unified
                .arch    armv6-m

                .section .vectors
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __StackTop                      //  Top of Stack
                .long    Reset_Handler                   //  Reset Handler
                .long    NMI_Handler                     //  NMI Handler
                .long    HardFault_Handler               //  Hard Fault Handler
                .long    0                               //  Reserved
                .long    0                               //  Reserved
                .long    0                               //  Reserved
                .long    0                               //  Reserved
                .long    0                               //  Reserved
                .long    0                               //  Reserved
                .long    0                               //  Reserved
                .long    SVC_Handler                     //  SVCall Handler
                .long    0                               //  Reserved
                .long    0                               //  Reserved
                .long    PendSV_Handler                  //  PendSV Handler
                .long    SysTick_Handler                 //  SysTick Handler

                /* Interrupts */
                .long     WWDG_IRQHandler                //  Window Watchdog
                .long     PVD_IRQHandler                 //  PVD through EXTI Line detect
                .long     RTC_BKP_IRQHandler             //  RTC through EXTI Line
                .long     FLASH_IRQHandler               //  FLASH
                .long     RCC_CRS_IRQHandler             //  RCC & CRS
                .long     EXTI0_1_IRQHandler             //  EXTI Line 0 and 1
                .long     EXTI2_3_IRQHandler             //  EXTI Line 2 and 3
                .long     EXTI4_15_IRQHandler            //  EXTI Line 4 to 15
                .long     HWDIV_IRQHandler               //  HWDIV
                .long     DMA1_Channel1_IRQHandler       //  DMA Channel 1
                .long     DMA1_Channel2_3_IRQHandler     //  DMA Channel 2 and Channel 3
                .long     DMA1_Channel4_5_IRQHandler     //  DMA Channel 4 and Channel 5
                .long     ADC1_COMP_IRQHandler           //  ADC and COMPARE
                .long     TIM1_BRK_UP_TRG_COM_IRQHandler //  TIM1 Break, Update, Trigger and Commutation
                .long     TIM1_CC_IRQHandler             //  TIM1 Capture Compare
                .long     TIM2_IRQHandler                //  TIM2
                .long     TIM3_IRQHandler                //  TIM3
                .long     0                              //  Reserved
                .long     0                              //  Reserved
                .long     TIM14_IRQHandler               //  TIM14
                .long     0                              //  Reserved
                .long     TIM16_IRQHandler               //  TIM16
                .long     TIM17_IRQHandler               //  TIM17
                .long     I2C1_IRQHandler                //  I2C
                .long     0                              //  Reserved
                .long     SPI1_IRQHandler                //  SPI1
                .long     SPI2_IRQHandler                //  SPI2
                .long     UART1_IRQHandler               //  UART1
                .long     UART2_IRQHandler               //  UART2
                .long     0                              //  Reserved
                .long     CAN_IRQHandler                 //  CAN
                .long     USB_IRQHandler                 //  USB
   
                // .space   ( 32 * 4)
__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors


                .thumb
                .section .text
                .align   2

                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
                .fnstart
Reset_Handler:
                bl       SystemInit
            
               // ldr     r0, =SystemInit
                //blx     r0

                ldr      r4, =__copy_table_start__
                ldr      r5, =__copy_table_end__

.L_loop0:
                cmp      r4, r5
                bge      .L_loop0_done
                ldr      r1, [r4]
                ldr      r2, [r4, #4]
                ldr      r3, [r4, #8]

.L_loop0_0:
                subs     r3, #4
                blt      .L_loop0_0_done
                ldr      r0, [r1, r3]
                str      r0, [r2, r3]
                b        .L_loop0_0

.L_loop0_0_done:
                adds     r4, #12
                b        .L_loop0

.L_loop0_done:

                ldr      r3, =__zero_table_start__
                ldr      r4, =__zero_table_end__

.L_loop2:
                cmp      r3, r4
                bge      .L_loop2_done
                ldr      r1, [r3]
                ldr      r2, [r3, #4]
                movs     r0, 0

.L_loop2_0:
                subs     r2, #4
                blt      .L_loop2_0_done
                str      r0, [r1, r2]
                b        .L_loop2_0
.L_loop2_0_done:

                adds     r3, #8
                b        .L_loop2
.L_loop2_done:

                bl       _start

                .fnend
                .size    Reset_Handler, . - Reset_Handler

/* The default macro is not used for HardFault_Handler
 * because this results in a poor debug illusion.
 */
                .thumb_func
                .type    HardFault_Handler, %function
                .weak    HardFault_Handler
                .fnstart
HardFault_Handler:
                b        .
                .fnend
                .size    HardFault_Handler, . - HardFault_Handler

                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
Default_Handler:
                b        .
                .fnend
                .size    Default_Handler, . - Default_Handler

/* Macro to define default exception/interrupt handlers.
 * Default handler are weak symbols with an endless loop.
 * They can be overwritten by real handlers.
 */
                .macro   Set_Default_Handler  Handler_Name
                .weak    \Handler_Name
                .set     \Handler_Name, Default_Handler
                .endm


/* Default exception/interrupt handler */

                Set_Default_Handler  NMI_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler
                Set_Default_Handler  WWDG_IRQHandler
                Set_Default_Handler  PVD_IRQHandler
                Set_Default_Handler  RTC_BKP_IRQHandler
                Set_Default_Handler  FLASH_IRQHandler
                Set_Default_Handler  RCC_CRS_IRQHandler
                Set_Default_Handler  EXTI0_1_IRQHandler
                Set_Default_Handler  EXTI2_3_IRQHandler
                Set_Default_Handler  EXTI4_15_IRQHandler
                Set_Default_Handler  HWDIV_IRQHandler
                Set_Default_Handler  DMA1_Channel1_IRQHandler
                Set_Default_Handler  DMA1_Channel2_3_IRQHandler
                Set_Default_Handler  DMA1_Channel4_5_IRQHandler
                Set_Default_Handler  ADC1_COMP_IRQHandler
                Set_Default_Handler  TIM1_BRK_UP_TRG_COM_IRQHandler
                Set_Default_Handler  TIM1_CC_IRQHandler
                Set_Default_Handler  TIM2_IRQHandler
                Set_Default_Handler  TIM3_IRQHandler
                Set_Default_Handler  TIM14_IRQHandler
                Set_Default_Handler  TIM16_IRQHandler
                Set_Default_Handler  TIM17_IRQHandler
                Set_Default_Handler  I2C1_IRQHandler
                Set_Default_Handler  SPI1_IRQHandler
                Set_Default_Handler  SPI2_IRQHandler
                Set_Default_Handler  UART1_IRQHandler
                Set_Default_Handler  UART2_IRQHandler
                Set_Default_Handler  CAN_IRQHandler
                Set_Default_Handler  USB_IRQHandler
                .end
