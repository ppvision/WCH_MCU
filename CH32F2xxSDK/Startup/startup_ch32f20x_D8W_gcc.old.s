/**************************************************************************//**
 * @file     startup_ARMCM3.S
 * @brief    CMSIS-Core(M) Device Startup File for Cortex-M3 Device
 * @version  V2.2.0
 * @date     26. May 2021
 ******************************************************************************/
/*
 * Copyright (c) 2009-2021 Arm Limited. All rights reserved.
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
                .arch    armv7-m

                .section .vectors
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __StackTop                         /*     Top of Stack */
                .long    Reset_Handler                      /*     Reset Handler */
                .long    NMI_Handler                        /* -14 NMI Handler */
                .long    HardFault_Handler                  /* -13 Hard Fault Handler */
                .long    MemManage_Handler                  /* -12 MPU Fault Handler */
                .long    BusFault_Handler                   /* -11 Bus Fault Handler */
                .long    UsageFault_Handler                 /* -10 Usage Fault Handler */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    SVC_Handler                        /*  -5 SVC Handler */
                .long    DebugMon_Handler                   /*  -4 Debug Monitor Handler */
                .long    0                                  /*     Reserved */
                .long    PendSV_Handler                     /*  -2 PendSV Handler */
                .long    SysTick_Handler                    /*  -1 SysTick Handler */

/*******************************************************************************
 External Interrupts
*******************************************************************************/
                .long    WWDG_IRQHandler            // Window Watchdog
                .long    PVD_IRQHandler             //PVD through EXTI Line detect
                .long    TAMPER_IRQHandler          //TAMPER
                .long    RTC_IRQHandler             //RTC
                .long    FLASH_IRQHandler           //FLASH
                .long    RCC_IRQHandler             //RCC
                .long    EXTI0_IRQHandler           //EXTI Line 0
                .long    EXTI1_IRQHandler           //EXTI Line 1
                .long    EXTI2_IRQHandler           //EXTI Line 2
                .long    EXTI3_IRQHandler           //EXTI Line 3
                .long    EXTI4_IRQHandler           //EXTI Line 4
                .long    DMA1_Channel1_IRQHandler   //DMA1 Channel 1
                .long    DMA1_Channel2_IRQHandler   //DMA1 Channel 2
                .long    DMA1_Channel3_IRQHandler   //DMA1 Channel 3
                .long    DMA1_Channel4_IRQHandler   //DMA1 Channel 4
                .long    DMA1_Channel5_IRQHandler   //DMA1 Channel 5
                .long    DMA1_Channel6_IRQHandler   //DMA1 Channel 6
                .long    DMA1_Channel7_IRQHandler   //DMA1 Channel 7
                .long    ADC1_2_IRQHandler          //ADC1_2
                .long    USB_HP_CAN1_TX_IRQHandler  //USB High Priority or CAN1 TX
                .long    USB_LP_CAN1_RX0_IRQHandler //USB Low  Priority or CAN1 RX0
                .long    CAN1_RX1_IRQHandler        //CAN1 RX1
                .long    CAN1_SCE_IRQHandler        //CAN1 SCE
                .long    EXTI9_5_IRQHandler         //EXTI Line 9..5
                .long    TIM1_BRK_IRQHandler        //TIM1 Break
                .long    TIM1_UP_IRQHandler         //TIM1 Update
                .long    TIM1_TRG_COM_IRQHandler    //TIM1 Trigger and Commutation
                .long    TIM1_CC_IRQHandler         //TIM1 Capture Compare
                .long    TIM2_IRQHandler            //TIM2
                .long    TIM3_IRQHandler            //TIM3
                .long    TIM4_IRQHandler            //TIM4
                .long    I2C1_EV_IRQHandler         //I2C1 Event
                .long    I2C1_ER_IRQHandler         //I2C1 Error
                .long    I2C2_EV_IRQHandler         //I2C2 Event
                .long    I2C2_ER_IRQHandler         //I2C2 Error
                .long    SPI1_IRQHandler            //SPI1
                .long    SPI2_IRQHandler            //SPI2
                .long    USART1_IRQHandler          //USART1
                .long    USART2_IRQHandler          //USART2
                .long    USART3_IRQHandler          //USART3
                .long    EXTI15_10_IRQHandler       //EXTI Line 15..10
                .long    RTCAlarm_IRQHandler        //RTC Alarm through EXTI Line
                .long    USBWakeUp_IRQHandler       //USB Wakeup
                .long    USBHD_IRQHandler           //USB Host/Device 
                .long    USBHDWakeUp_IRQHandler     //USB Host/Device Wakeup
                .long    ETH_IRQHandler             //ETH 
                .long    ETHWakeUp_IRQHandler       //ETH Wakeup
                .long    BB_IRQHandler              //BLE BB
                .long    LLE_IRQHandler             //BLE LLE
                .long    TIM5_IRQHandler            //TIM5
                .long    UART4_IRQHandler           //UART4
                .long    DMA1_Channel8_IRQHandler   //DMA1 Channel8
                .long    OSC32KCal_IRQHandler       //OSC32KCal
                .long    OSCWakeUp_IRQHandler       //OSC WakeUp     

                .space   ((224-54) * 4)                          /* Interrupts 10 .. 224 are left out */


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

                ldr      r4, =__copy_table_start__
                ldr      r5, =__copy_table_end__

.L_loop0:
                cmp      r4, r5
                bge      .L_loop0_done
                ldr      r1, [r4]                /* source address */
                ldr      r2, [r4, #4]            /* destination address */
                ldr      r3, [r4, #8]            /* word count */
                lsls     r3, r3, #2              /* byte count */

.L_loop0_0:
                subs     r3, #4                  /* decrement byte count */
                ittt     ge
                ldrge    r0, [r1, r3]
                strge    r0, [r2, r3]
                bge      .L_loop0_0

                adds     r4, #12
                b        .L_loop0
.L_loop0_done:

                ldr      r3, =__zero_table_start__
                ldr      r4, =__zero_table_end__

.L_loop2:
                cmp      r3, r4
                bge      .L_loop2_done
                ldr      r1, [r3]                /* destination address */
                ldr      r2, [r3, #4]            /* word count */
                lsls     r2, r2, #2              /* byte count */
                movs     r0, 0

.L_loop2_0:
                subs     r2, #4                  /* decrement byte count */
                itt      ge
                strge    r0, [r1, r2]
                bge      .L_loop2_0

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
                Set_Default_Handler  MemManage_Handler
                Set_Default_Handler  BusFault_Handler
                Set_Default_Handler  UsageFault_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  DebugMon_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler
/*******************************************************************************
 External Interrupts
*******************************************************************************/
                Set_Default_Handler  WWDG_IRQHandler            //Window Watchdog
                Set_Default_Handler  PVD_IRQHandler             //PVD through EXTI Line detect
                Set_Default_Handler  TAMPER_IRQHandler          //TAMPER
                Set_Default_Handler  RTC_IRQHandler             //RTC
                Set_Default_Handler  FLASH_IRQHandler           //FLASH
                Set_Default_Handler  RCC_IRQHandler             //RCC
                Set_Default_Handler  EXTI0_IRQHandler           //EXTI Line 0
                Set_Default_Handler  EXTI1_IRQHandler           //EXTI Line 1
                Set_Default_Handler  EXTI2_IRQHandler           //EXTI Line 2
                Set_Default_Handler  EXTI3_IRQHandler           //EXTI Line 3
                Set_Default_Handler  EXTI4_IRQHandler           //EXTI Line 4
                Set_Default_Handler  DMA1_Channel1_IRQHandler   //DMA1 Channel 1
                Set_Default_Handler  DMA1_Channel2_IRQHandler   //DMA1 Channel 2
                Set_Default_Handler  DMA1_Channel3_IRQHandler   //DMA1 Channel 3
                Set_Default_Handler  DMA1_Channel4_IRQHandler   //DMA1 Channel 4
                Set_Default_Handler  DMA1_Channel5_IRQHandler   //DMA1 Channel 5
                Set_Default_Handler  DMA1_Channel6_IRQHandler   //DMA1 Channel 6
                Set_Default_Handler  DMA1_Channel7_IRQHandler   //DMA1 Channel 7
                Set_Default_Handler  ADC1_2_IRQHandler          //ADC1_2
                Set_Default_Handler  USB_HP_CAN1_TX_IRQHandler  //USB High Priority or CAN1 TX
                Set_Default_Handler  USB_LP_CAN1_RX0_IRQHandler //USB Low  Priority or CAN1 RX0
                Set_Default_Handler  CAN1_RX1_IRQHandler        //CAN1 RX1
                Set_Default_Handler  CAN1_SCE_IRQHandler        //CAN1 SCE
                Set_Default_Handler  EXTI9_5_IRQHandler         //EXTI Line 9..5
                Set_Default_Handler  TIM1_BRK_IRQHandler        //TIM1 Break
                Set_Default_Handler  TIM1_UP_IRQHandler         //TIM1 Update
                Set_Default_Handler  TIM1_TRG_COM_IRQHandler    //TIM1 Trigger and Commutation
                Set_Default_Handler  TIM1_CC_IRQHandler         //TIM1 Capture Compare
                Set_Default_Handler  TIM2_IRQHandler            //TIM2
                Set_Default_Handler  TIM3_IRQHandler            //TIM3
                Set_Default_Handler  TIM4_IRQHandler            //TIM4
                Set_Default_Handler  I2C1_EV_IRQHandler         //I2C1 Event
                Set_Default_Handler  I2C1_ER_IRQHandler         //I2C1 Error
                Set_Default_Handler  I2C2_EV_IRQHandler         //I2C2 Event
                Set_Default_Handler  I2C2_ER_IRQHandler         //I2C2 Error
                Set_Default_Handler  SPI1_IRQHandler            //SPI1
                Set_Default_Handler  SPI2_IRQHandler            //SPI2
                Set_Default_Handler  USART1_IRQHandler          //USART1
                Set_Default_Handler  USART2_IRQHandler          //USART2
                Set_Default_Handler  USART3_IRQHandler          //USART3
                Set_Default_Handler  EXTI15_10_IRQHandler       //EXTI Line 15..10
                Set_Default_Handler  RTCAlarm_IRQHandler        //RTC Alarm through EXTI Line
                Set_Default_Handler  USBWakeUp_IRQHandler       //USB Wakeup
                Set_Default_Handler  USBHD_IRQHandler           //USB Host/Device 
                Set_Default_Handler  USBHDWakeUp_IRQHandler     //USB Host/Device Wakeup
                Set_Default_Handler  ETH_IRQHandler             //ETH 
                Set_Default_Handler  ETHWakeUp_IRQHandler       //ETH Wakeup
                Set_Default_Handler  BB_IRQHandler              //BLE BB
                Set_Default_Handler  LLE_IRQHandler             //BLE LLE
                Set_Default_Handler  TIM5_IRQHandler            //TIM5
                Set_Default_Handler  UART4_IRQHandler           //UART4
                Set_Default_Handler  DMA1_Channel8_IRQHandler   //DMA1 Channel8
                Set_Default_Handler  OSC32KCal_IRQHandler       //OSC32KCal
                Set_Default_Handler  OSCWakeUp_IRQHandler       //OSC WakeUp 


                .end
