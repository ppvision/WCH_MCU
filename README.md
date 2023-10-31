# Mac下开发沁恒CH32F2xx

南京沁恒的芯片大家应该都比较熟悉，特别是USB转串口，国内、国外市场占有率很大。
沁恒专注、开放、热情在USB接口领域做到了邻头羊的地位，在RISC- V也有较早的布局。很早就想玩玩他家的板子，手上也有一块他家ch32f20xEvt的开发版，但因其提供的工具链主要是windows下，官方提供的开发IDE[MounRiver Studio](http://www.mounriver.com/)
目前仅支持windows和LINUX。他们似乎忘记了，还有一些专业选手工作在MAC上。

Mac上的嵌入式老狗如何在喜欢的CLion & VSCode & Sublime调试开发沁恒CH32F2xx。

今天给大家介绍一下沁恒ARM Cortex-M3 在Mac上的开发流程`SOC为CH32F208WBU6`

<div class="bg-white  mx-auto">
  <img src="/ch32f208_func.png" class="max-h-full">
</div>

<div class="bg-white  mx-auto">
  <img src="/ch32f208_sys.png" class="max-h-full">
</div>

:::info 基本技能
嵌入式开发，你首先要掌握、使用下面的工具
- 交叉编译器。
- 构建工具
- 调试工具
    Jlink, Dap-link,ST-linke及沁恒自家的ws-link等等
:::


## 工具链。

交叉编译器有很多种，Keil只能工作在windows上，
Mac 下可以选择的工具链有主要gcc、clang。我们主要讲解在Mac上使用GNU gcc编译工具
ARM gcc编译器可以在ARM的官网址下载。[https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads]

## 构建工具
远古时代大家都写Makefile,Makefile的语法学习曲线比较陡峭，一下两下真掌握不透。现在主流的构建工具普遍为cmake，它的配置文件CMakeList.txt 就比较简单易用。各大厂NXP、TI等都在使用。
当然用好它还是要花点功夫。现在你不用担心这些，我已经帮你准备好了针对arm的开发环境设置、编译器的参数配置已经帮你准备好了。你只需要从github上check out出来开箱食用。[MCU_Builder](https://github.com/ppvision/MCU_Builder)


## 移植

针对于某一款MCU的移植，首先我们要知道:
- 它采用什么架构，目前常见的架构有Arm，RISC—V等。
- 存储资源
    > - 了解其存储(Flash、RAM)映射地址及大小。
    > - FLAH的使用规划,RAM堆栈、Heap的分配

**这些配置需要GCC链接脚本`chf2xx.ld`描述**



## 修改启动初始代码&配置向量表


::: tip 上电后的向量表
    ARM的Soc一般可以有多种启动方式。 
 可以在内置的ROM中启动，也可以从内、外部的flash或者RAM启动，这种启动方式的选择一般是通过boot引脚的上拉或者下拉的排列组合来确定。因此地址 0x0000_0000 处应该存储上电后的向量表.  

 >M3 向量表格式 

| 地址   |     向量编号      |  说明  |
|----------|:-------------:|------:|
| 0x0000_0000 | 0 | MSP初始值 |
| 0x0000_0004 | 1 | 复位向量(PC 初始值) |

:::

 - **MSP值** 我们规划为RAM的最高址也就是 __RAM_BASE + __RAM_SIZE 
    > 上面的ld文件我们预留了256字节做为它用
 - **PC**  复位后第一条指令地址 


    SOC启动初始化代码一般是采用汇编语言编写。 沁恒只提供了windows下Keil的示例工程，Keil汇编语言和gcc的汇编语言语法有些不同，我们需要依葫芦画瓢重按gcc 汇编语法改写它的启动代码。


## ch32f20x启动代码



::: code-group
```asm [keil 汇编]
;/********************************** (C) COPYRIGHT *******************************
;* File Name          : startup_ch32f20x_D8W.s
;* Author             : WCH
;* Version            : V1.0.0
;* Date               : 2021/08/08
;* Description        : CH32F208x vector table for MDK-ARM toolchain.
;*********************************************************************************
;* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
;* Attention: This software (modified or not) and binary are used for 
;* microcontroller manufactured by Nanjing Qinheng Microelectronics.
;*******************************************************************************/

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp

;/*******************************************************************************
;*******************************************************************************/
Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB

;/*******************************************************************************
; Vector Table Mapped to Address 0 at Reset
;*******************************************************************************/
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD     Reset_Handler              ; Reset Handler
                DCD     NMI_Handler                ; NMI Handler
                DCD     HardFault_Handler          ; Hard Fault Handler
                DCD     MemManage_Handler          ; MPU Fault Handler
                DCD     BusFault_Handler           ; Bus Fault Handler
                DCD     UsageFault_Handler         ; Usage Fault Handler
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     SVC_Handler                ; SVCall Handler
                DCD     DebugMon_Handler           ; Debug Monitor Handler
                DCD     0                          ; Reserved
                DCD     PendSV_Handler             ; PendSV Handler
                DCD     SysTick_Handler            ; SysTick Handler
                                    
;/*******************************************************************************
; External Interrupts
;*******************************************************************************/
                DCD     WWDG_IRQHandler            ; Window Watchdog
                DCD     PVD_IRQHandler             ; PVD through EXTI Line detect
                DCD     TAMPER_IRQHandler          ; TAMPER
                DCD     RTC_IRQHandler             ; RTC
                DCD     FLASH_IRQHandler           ; FLASH
                DCD     RCC_IRQHandler             ; RCC
                DCD     EXTI0_IRQHandler           ; EXTI Line 0
                DCD     EXTI1_IRQHandler           ; EXTI Line 1
                DCD     EXTI2_IRQHandler           ; EXTI Line 2
                DCD     EXTI3_IRQHandler           ; EXTI Line 3
                DCD     EXTI4_IRQHandler           ; EXTI Line 4
                DCD     DMA1_Channel1_IRQHandler   ; DMA1 Channel 1
                DCD     DMA1_Channel2_IRQHandler   ; DMA1 Channel 2
                DCD     DMA1_Channel3_IRQHandler   ; DMA1 Channel 3
                DCD     DMA1_Channel4_IRQHandler   ; DMA1 Channel 4
                DCD     DMA1_Channel5_IRQHandler   ; DMA1 Channel 5
                DCD     DMA1_Channel6_IRQHandler   ; DMA1 Channel 6
                DCD     DMA1_Channel7_IRQHandler   ; DMA1 Channel 7
                DCD     ADC1_2_IRQHandler          ; ADC1_2
                DCD     USB_HP_CAN1_TX_IRQHandler  ; USB High Priority or CAN1 TX
                DCD     USB_LP_CAN1_RX0_IRQHandler ; USB Low  Priority or CAN1 RX0
                DCD     CAN1_RX1_IRQHandler        ; CAN1 RX1
                DCD     CAN1_SCE_IRQHandler        ; CAN1 SCE
                DCD     EXTI9_5_IRQHandler         ; EXTI Line 9..5
                DCD     TIM1_BRK_IRQHandler        ; TIM1 Break
                DCD     TIM1_UP_IRQHandler         ; TIM1 Update
                DCD     TIM1_TRG_COM_IRQHandler    ; TIM1 Trigger and Commutation
                DCD     TIM1_CC_IRQHandler         ; TIM1 Capture Compare
                DCD     TIM2_IRQHandler            ; TIM2
                DCD     TIM3_IRQHandler            ; TIM3
                DCD     TIM4_IRQHandler            ; TIM4
                DCD     I2C1_EV_IRQHandler         ; I2C1 Event
                DCD     I2C1_ER_IRQHandler         ; I2C1 Error
                DCD     I2C2_EV_IRQHandler         ; I2C2 Event
                DCD     I2C2_ER_IRQHandler         ; I2C2 Error
                DCD     SPI1_IRQHandler            ; SPI1
                DCD     SPI2_IRQHandler            ; SPI2
                DCD     USART1_IRQHandler          ; USART1
                DCD     USART2_IRQHandler          ; USART2
                DCD     USART3_IRQHandler          ; USART3
                DCD     EXTI15_10_IRQHandler       ; EXTI Line 15..10
                DCD     RTCAlarm_IRQHandler        ; RTC Alarm through EXTI Line
                DCD     USBWakeUp_IRQHandler       ; USB Wakeup
                DCD     USBHD_IRQHandler           ; USB Host/Device 
                DCD     USBHDWakeUp_IRQHandler     ; USB Host/Device Wakeup
                DCD     ETH_IRQHandler             ; ETH 
                DCD     ETHWakeUp_IRQHandler       ; ETH Wakeup
                DCD     BB_IRQHandler              ; BLE BB
                DCD     LLE_IRQHandler             ; BLE LLE
                DCD     TIM5_IRQHandler            ; TIM5
                DCD     UART4_IRQHandler           ; UART4
                DCD     DMA1_Channel8_IRQHandler   ; DMA1 Channel8
                DCD     OSC32KCal_IRQHandler       ; OSC32KCal
                DCD     OSCWakeUp_IRQHandler       ; OSC WakeUp                             
                
__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY
;/*******************************************************************************
; Reset handler
;*******************************************************************************/
Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK]
     IMPORT  __main
     IMPORT  SystemInit
                 LDR     R0, =SystemInit
                 BLX     R0
                 LDR     R0, =__main
                 BX      R0
                 ENDP

;/*******************************************************************************
; Dummy Exception Handlers (infinite loops which can be modified)
;*******************************************************************************/
NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler          [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler           [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler           [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WWDG_IRQHandler            [WEAK]
                EXPORT  PVD_IRQHandler             [WEAK]
                EXPORT  TAMPER_IRQHandler          [WEAK]
                EXPORT  RTC_IRQHandler             [WEAK]
                EXPORT  FLASH_IRQHandler           [WEAK]
                EXPORT  RCC_IRQHandler             [WEAK]
                EXPORT  EXTI0_IRQHandler           [WEAK]
                EXPORT  EXTI1_IRQHandler           [WEAK]
                EXPORT  EXTI2_IRQHandler           [WEAK]
                EXPORT  EXTI3_IRQHandler           [WEAK]
                EXPORT  EXTI4_IRQHandler           [WEAK]
                EXPORT  DMA1_Channel1_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel2_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel3_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel4_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel5_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel6_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel7_IRQHandler   [WEAK]
                EXPORT  ADC1_2_IRQHandler          [WEAK]
                EXPORT  USB_HP_CAN1_TX_IRQHandler  [WEAK]
                EXPORT  USB_LP_CAN1_RX0_IRQHandler [WEAK]
                EXPORT  CAN1_RX1_IRQHandler        [WEAK]
                EXPORT  CAN1_SCE_IRQHandler        [WEAK]
                EXPORT  EXTI9_5_IRQHandler         [WEAK]
                EXPORT  TIM1_BRK_IRQHandler        [WEAK]
                EXPORT  TIM1_UP_IRQHandler         [WEAK]
                EXPORT  TIM1_TRG_COM_IRQHandler    [WEAK]
                EXPORT  TIM1_CC_IRQHandler         [WEAK]
                EXPORT  TIM2_IRQHandler            [WEAK]
                EXPORT  TIM3_IRQHandler            [WEAK]
                EXPORT  TIM4_IRQHandler            [WEAK]
                EXPORT  I2C1_EV_IRQHandler         [WEAK]
                EXPORT  I2C1_ER_IRQHandler         [WEAK]
                EXPORT  I2C2_EV_IRQHandler         [WEAK]
                EXPORT  I2C2_ER_IRQHandler         [WEAK]
                EXPORT  SPI1_IRQHandler            [WEAK]
                EXPORT  SPI2_IRQHandler            [WEAK]
                EXPORT  USART1_IRQHandler          [WEAK]
                EXPORT  USART2_IRQHandler          [WEAK]
                EXPORT  USART3_IRQHandler          [WEAK]
                EXPORT  EXTI15_10_IRQHandler       [WEAK]
                EXPORT  RTCAlarm_IRQHandler        [WEAK]
                EXPORT  USBWakeUp_IRQHandler       [WEAK]
                EXPORT  USBHD_IRQHandler           [WEAK]
                EXPORT  USBHDWakeUp_IRQHandler     [WEAK]
                EXPORT  ETH_IRQHandler             [WEAK]
                EXPORT  ETHWakeUp_IRQHandler       [WEAK]
                EXPORT  BB_IRQHandler              [WEAK]
                EXPORT  LLE_IRQHandler             [WEAK]
                EXPORT  TIM5_IRQHandler            [WEAK]
                EXPORT  UART4_IRQHandler           [WEAK]
                EXPORT  DMA1_Channel8_IRQHandler   [WEAK]
                EXPORT  OSC32KCal_IRQHandler       [WEAK]
                EXPORT  OSCWakeUp_IRQHandler       [WEAK]
 

WWDG_IRQHandler
PVD_IRQHandler
TAMPER_IRQHandler
RTC_IRQHandler
FLASH_IRQHandler
RCC_IRQHandler
EXTI0_IRQHandler
EXTI1_IRQHandler
EXTI2_IRQHandler
EXTI3_IRQHandler
EXTI4_IRQHandler
DMA1_Channel1_IRQHandler
DMA1_Channel2_IRQHandler
DMA1_Channel3_IRQHandler
DMA1_Channel4_IRQHandler
DMA1_Channel5_IRQHandler
DMA1_Channel6_IRQHandler
DMA1_Channel7_IRQHandler
ADC1_2_IRQHandler
USB_HP_CAN1_TX_IRQHandler
USB_LP_CAN1_RX0_IRQHandler
CAN1_RX1_IRQHandler
CAN1_SCE_IRQHandler
EXTI9_5_IRQHandler
TIM1_BRK_IRQHandler
TIM1_UP_IRQHandler
TIM1_TRG_COM_IRQHandler
TIM1_CC_IRQHandler
TIM2_IRQHandler
TIM3_IRQHandler
TIM4_IRQHandler
I2C1_EV_IRQHandler
I2C1_ER_IRQHandler
I2C2_EV_IRQHandler
I2C2_ER_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
USART1_IRQHandler
USART2_IRQHandler
USART3_IRQHandler
EXTI15_10_IRQHandler
RTCAlarm_IRQHandler
USBWakeUp_IRQHandler
USBHD_IRQHandler
USBHDWakeUp_IRQHandler 
ETH_IRQHandler
ETHWakeUp_IRQHandler
BB_IRQHandler
LLE_IRQHandler
TIM5_IRQHandler
UART4_IRQHandler
DMA1_Channel8_IRQHandler
OSC32KCal_IRQHandler
OSCWakeUp_IRQHandler

                B       .

                ENDP

                ALIGN

;/*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************/
                 IF      :DEF:__MICROLIB           
                
                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit
                
                 ELSE
                
                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN

                 ENDIF

                 END


```

```asm [gcc 汇编]
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
                bl       main

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
******************************************************************************/
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


```

```c [全C 实现的启动代码 ]


#define Def_Week_Handler(nm) void  __attribute__((weak)) nm(void) { \
    while(1);\
}

//    SYSTEM WILL HELP US AUTO COPY IT
__attribute__ ((section(".after_vectors.init_data")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int *pulSrc = (unsigned int*) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors.init_bss")))
void bss_init(unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;

extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;
extern void SystemInit();
extern void main();


void SystemInitHook (void) {
    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;
    // Load base address of Global Section Table
    SectionTableAddr = &__data_section_table;
    // Copy the data sections from flash to SRAM.
    while (SectionTableAddr < &__data_section_table_end) {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        if(LoadAddr != ExeAddr) data_init(LoadAddr, ExeAddr, SectionLen);
    }
    // At this point, SectionTableAddr = &__bss_section_table;
    // Zero fill the bss segment
    while (SectionTableAddr < &__bss_section_table_end) {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        bss_init(ExeAddr, SectionLen);
    }
    SectionLen = 0;
}


void Reset_Handler(void){
     SystemInitHook();
     SystemInit();
     main();
}



Def_Week_Handler(NMI_Handler)
Def_Week_Handler(HardFault_Handler)
Def_Week_Handler(MemManage_Handler)
Def_Week_Handler(BusFault_Handler)
Def_Week_Handler(UsageFault_Handler)
Def_Week_Handler(SVC_Handler)
Def_Week_Handler(DebugMon_Handler)
Def_Week_Handler(PendSV_Handler)
Def_Week_Handler(SysTick_Handler)

/*******************************************************************************
 External Interrupts
*******************************************************************************/
Def_Week_Handler(WWDG_IRQHandler           )
Def_Week_Handler(PVD_IRQHandler            )
Def_Week_Handler(TAMPER_IRQHandler         )
Def_Week_Handler(RTC_IRQHandler            )
Def_Week_Handler(FLASH_IRQHandler          )
Def_Week_Handler(RCC_IRQHandler            )
Def_Week_Handler(EXTI0_IRQHandler          )
Def_Week_Handler(EXTI1_IRQHandler          )
Def_Week_Handler(EXTI2_IRQHandler          )
Def_Week_Handler(EXTI3_IRQHandler          )
Def_Week_Handler(EXTI4_IRQHandler          )
Def_Week_Handler(DMA1_Channel1_IRQHandler  )
Def_Week_Handler(DMA1_Channel2_IRQHandler  )
Def_Week_Handler(DMA1_Channel3_IRQHandler  )
Def_Week_Handler(DMA1_Channel4_IRQHandler  )
Def_Week_Handler(DMA1_Channel5_IRQHandler  )
Def_Week_Handler(DMA1_Channel6_IRQHandler  )
Def_Week_Handler(DMA1_Channel7_IRQHandler  )
Def_Week_Handler(ADC1_2_IRQHandler         )
Def_Week_Handler(USB_HP_CAN1_TX_IRQHandler )
Def_Week_Handler(USB_LP_CAN1_RX0_IRQHandler)
Def_Week_Handler(CAN1_RX1_IRQHandler       )
Def_Week_Handler(CAN1_SCE_IRQHandler       )
Def_Week_Handler(EXTI9_5_IRQHandler        )
Def_Week_Handler(TIM1_BRK_IRQHandler       )
Def_Week_Handler(TIM1_UP_IRQHandler        )
Def_Week_Handler(TIM1_TRG_COM_IRQHandler   )
Def_Week_Handler(TIM1_CC_IRQHandler        )
Def_Week_Handler(TIM2_IRQHandler           )
Def_Week_Handler(TIM3_IRQHandler           )
Def_Week_Handler(TIM4_IRQHandler           )
Def_Week_Handler(I2C1_EV_IRQHandler        )
Def_Week_Handler(I2C1_ER_IRQHandler        )
Def_Week_Handler(I2C2_EV_IRQHandler        )
Def_Week_Handler(I2C2_ER_IRQHandler        )
Def_Week_Handler(SPI1_IRQHandler           )
Def_Week_Handler(SPI2_IRQHandler           )
Def_Week_Handler(USART1_IRQHandler         )
Def_Week_Handler(USART2_IRQHandler         )
Def_Week_Handler(USART3_IRQHandler         )
Def_Week_Handler(EXTI15_10_IRQHandler      )
Def_Week_Handler(RTCAlarm_IRQHandler       )
Def_Week_Handler(USBWakeUp_IRQHandler      )
Def_Week_Handler(USBHD_IRQHandler          )
Def_Week_Handler(USBHDWakeUp_IRQHandler    )
Def_Week_Handler(ETH_IRQHandler            )
Def_Week_Handler(ETHWakeUp_IRQHandler      )
Def_Week_Handler(BB_IRQHandler             )
Def_Week_Handler(LLE_IRQHandler            )
Def_Week_Handler(TIM5_IRQHandler           )
Def_Week_Handler(UART4_IRQHandler          )
Def_Week_Handler(DMA1_Channel8_IRQHandler  )
Def_Week_Handler(OSC32KCal_IRQHandler      )
Def_Week_Handler(OSCWakeUp_IRQHandler      )

/*
    我们规划为RAM的最高址也就是 __RAM_BASE + __RAM_SIZE 为stack的底。
    （上面的ld文件我们在RAM预留了256字节做为它用）
*/

#define STACK_TOP (void (*)(void))(0x20000000 + 0x00010000 -256)

__attribute__ (( section(".vectors") )) void (* const VectorArray[])(void) = {
    STACK_TOP,
    Reset_Handler,
    NMI_Handler,        
    HardFault_Handler,  
    MemManage_Handler,  
    BusFault_Handler,   
    UsageFault_Handler,
    0,                  
    0,                  
    0,                  
    0,                  
    SVC_Handler,       
    DebugMon_Handler,   
    0,                  
    PendSV_Handler ,    
    SysTick_Handler,

/*******************************************************************************
 External Interrupts
*******************************************************************************/

    WWDG_IRQHandler           ,
    PVD_IRQHandler            ,
    TAMPER_IRQHandler         ,
    RTC_IRQHandler            ,
    FLASH_IRQHandler          ,
    RCC_IRQHandler            ,
    EXTI0_IRQHandler          ,
    EXTI1_IRQHandler          ,
    EXTI2_IRQHandler          ,
    EXTI3_IRQHandler          ,
    EXTI4_IRQHandler          ,
    DMA1_Channel1_IRQHandler  ,
    DMA1_Channel2_IRQHandler  ,
    DMA1_Channel3_IRQHandler  ,
    DMA1_Channel4_IRQHandler  ,
    DMA1_Channel5_IRQHandler  ,
    DMA1_Channel6_IRQHandler  ,
    DMA1_Channel7_IRQHandler  ,
    ADC1_2_IRQHandler         ,
    USB_HP_CAN1_TX_IRQHandler ,
    USB_LP_CAN1_RX0_IRQHandler,
    CAN1_RX1_IRQHandler       ,
    CAN1_SCE_IRQHandler       ,
    EXTI9_5_IRQHandler        ,
    TIM1_BRK_IRQHandler       ,
    TIM1_UP_IRQHandler        ,
    TIM1_TRG_COM_IRQHandler   ,
    TIM1_CC_IRQHandler        ,
    TIM2_IRQHandler           ,
    TIM3_IRQHandler           ,
    TIM4_IRQHandler           ,
    I2C1_EV_IRQHandler        ,
    I2C1_ER_IRQHandler        ,
    I2C2_EV_IRQHandler        ,
    I2C2_ER_IRQHandler        ,
    SPI1_IRQHandler           ,
    SPI2_IRQHandler           ,
    USART1_IRQHandler         ,
    USART2_IRQHandler         ,
    USART3_IRQHandler         ,
    EXTI15_10_IRQHandler      ,
    RTCAlarm_IRQHandler       ,
    USBWakeUp_IRQHandler      ,
    USBHD_IRQHandler          ,
    USBHDWakeUp_IRQHandler    ,
    ETH_IRQHandler            ,
    ETHWakeUp_IRQHandler      ,
    BB_IRQHandler             ,
    LLE_IRQHandler            ,
    TIM5_IRQHandler           ,
    UART4_IRQHandler          ,
    DMA1_Channel8_IRQHandler  ,
    OSC32KCal_IRQHandler      ,
    OSCWakeUp_IRQHandler      ,
    0

};


```


```txt [ gcc 链接文件 chf2xx.ld ]

__ROM_BASE = 0x08000000;
__ROM_SIZE = 0x00078000;

/*--------------------- Embedded RAM Configuration ----------------------------*/
__RAM_BASE = 0x20000000;
__RAM_SIZE = 0x00010000;


MEMORY
{
    FLASH (rx)      : ORIGIN = __ROM_BASE, LENGTH = __ROM_SIZE
    RAM   (rwx)     : ORIGIN = __RAM_BASE, LENGTH = __RAM_SIZE - 256
    SCRATCH_Y(rwx)  : ORIGIN = __RAM_BASE + __RAM_SIZE - 256, LENGTH = 256

}

ENTRY(Reset_Handler)

SECTIONS
{
  .text :
  {
    KEEP(*(.vectors))
    *(.text*)

    KEEP(*(.init))
    KEEP(*(.fini))

    /* .ctors */
    *crtbegin.o(.ctors)
    *crtbegin?.o(.ctors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
    *(SORT(.ctors.*))
    *(.ctors)

    /* .dtors */
    *crtbegin.o(.dtors)
    *crtbegin?.o(.dtors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
    *(SORT(.dtors.*))
    *(.dtors)

    *(.rodata*)

    KEEP(*(.eh_frame*))
  } > FLASH


  .flash_range : ALIGN(4)
  {
      FILL(0xff)
      __section_table_start = .;
      __data_section_table = .;


      LONG(LOADADDR(.data));
      LONG(    ADDR(.data));
      LONG(  SIZEOF(.data));

      __data_section_table_end = .;

      __bss_section_table = .;
      LONG(    ADDR(.bss));
      LONG(  SIZEOF(.bss));
      __bss_section_table_end = .;
      __section_table_end = . ;
      /* End of Global Section Table */
      *(.after_vectors*)

  } > FLASH


  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  __exidx_start = .;
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > FLASH
  __exidx_end = .;


  __etext = ALIGN (4);

  .data : AT (__etext)
  {
    __data_start__ = .;
    *(vtable)
    *(.data)
    *(.data.*)

    . = ALIGN(4);
    /* preinit data */
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP(*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);

    . = ALIGN(4);
    /* init data */
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    PROVIDE_HIDDEN (__init_array_end = .);

    . = ALIGN(4);
    /* finit data */
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array))
    PROVIDE_HIDDEN (__fini_array_end = .);

    KEEP(*(.jcr*))
    . = ALIGN(4);
    /* All data end */
    __data_end__ = .;

  } > RAM



  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > RAM AT > RAM

    .heap (COPY):
    {
        __end__ = .;
        end = __end__;
        *(.heap*)
        __HeapLimit = .;
    } > RAM


    .flash_end : {
        __flash_binary_end = .;
    } > FLASH



    .stack_dummy (COPY):
    {
        *(.stack*)
    } > SCRATCH_Y

    __StackTop       = ORIGIN(RAM) + LENGTH(RAM) ;/*  */
    __StackLimit     = ORIGIN(RAM) + LENGTH(RAM) - __STACK_SIZE; /*  */
    __StackBottom    = __StackTop; /* init value*/

    PROVIDE(__stack = __StackTop);
    PROVIDE(_sstack = __StackTop);
    PROVIDE(_estack = __StackBottom);


    /* Check if data + heap + stack exceeds RAM limit */
  ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

    /* assign unused never call function to panic. 
    _close_r  = panic;
    _kill     = panic;
    _getpid   = panic;
    _fstat_r  = panic;
    _isatty_r = panic;
    _lseek_r  = panic;*/
}

```



:::


## openocd 配置

 - 启动openocd
```sh
openocd_wch -f wch-arm.cfg
```

::: code-group

```txt [启动openocd]
openocd_wch -f wch-arm.cfg
```

```txt [openocd结果]
Open On-Chip Debugger 0.11.0+dev-02415-gfad123a16-dirty (2023-06-12-19:28)
Licensed under GNU GPL v2
For bug reports, read
    http://openocd.org/doc/doxygen/bugs.html
Info : DEPRECATED target event trace-config; use TPIU events {pre,post}-{enable,disable}
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : Using CMSIS-DAPv2 interface with VID:PID=0x2e8a:0x000c, serial=0000
Info : CMSIS-DAP: SWD supported
Info : CMSIS-DAP: Atomic commands supported
Info : CMSIS-DAP: Test domain timer supported
Info : CMSIS-DAP: FW Version = 2.1.2
Info : CMSIS-DAP: Serial# = E46258A0E3472F2A
Info : CMSIS-DAP: Interface Initialised (SWD)
Info : SWCLK/TCK = 0 SWDIO/TMS = 0 TDI = 0 TDO = 0 nTRST = 0 nRESET = 0
Info : CMSIS-DAP: Interface ready
Info : clock speed 8000 kHz
Info : SWD DPIDR 0x2ba01477
Info : [ch32f1x.cpu] Cortex-M3 r2p1 processor detected
Info : [ch32f1x.cpu] target has 6 breakpoints, 4 watchpoints
[ch32f1x.cpu] Target successfully examined.
Info : starting gdb server for ch32f1x.cpu on 3333
Info : Listening on port 3333 for gdb connections
```


```txt [wcn_arm.cfg]
adapter driver cmsis-dap
transport select swd
adapter speed 8000
source [find target/ch32f1x.cfg]
```

```txt [target/ch32f1x.cfg]
# script for ch32f1x family

#
#ch32 devices support both JTAG and SWD transports.
#
source [find target/swj-dp.tcl]
source [find mem_helper.tcl]

if { [info exists CHIPNAME] } {
   set _CHIPNAME $CHIPNAME
} else {
   set _CHIPNAME ch32f1x
}

set _ENDIAN little

# Work-area is a space in RAM used for flash programming

if { [info exists WORKAREASIZE] } {
   set _WORKAREASIZE $WORKAREASIZE
} else {
   set _WORKAREASIZE 0x1000
}

# Allow overriding the Flash bank size
if { [info exists FLASH_SIZE] } {
    set _FLASH_SIZE $FLASH_SIZE
} else {
    # autodetect size
    set _FLASH_SIZE 0
}

#jtag scan chain
if { [info exists CPUTAPID] } {
   set _CPUTAPID $CPUTAPID
} else {
   if { [using_jtag] } {
      
      set _CPUTAPID 0x3ba00477
   } {
      # this is the SW-DP tap id not the jtag tap id
      set _CPUTAPID 0x1ba01477
   }
}

swj_newdap $_CHIPNAME cpu -irlen 4 -ircapture 0x1 -irmask 0xf -expected-id $_CPUTAPID
dap create $_CHIPNAME.dap -chain-position $_CHIPNAME.cpu

if {[using_jtag]} {
   jtag newtap $_CHIPNAME bs -irlen 5
}

set _TARGETNAME $_CHIPNAME.cpu
target create $_TARGETNAME cortex_m -endian $_ENDIAN -dap $_CHIPNAME.dap

$_TARGETNAME configure -work-area-phys 0x20000000 -work-area-size $_WORKAREASIZE -work-area-backup 0

# flash size will be probed
set _FLASHNAME $_CHIPNAME.flash
flash bank $_FLASHNAME wch_arm 0x08000000 $_FLASH_SIZE 0 0 $_TARGETNAME

# JTAG speed should be <= F_CPU/6. F_CPU after reset is 8MHz, so use F_JTAG = 1MHz
#adapter speed 4000

adapter srst delay 100
if {[using_jtag]} {
 jtag_ntrst_delay 100
}

reset_config srst_nogate

if {![using_hla]} {
    # if srst is not fitted use SYSRESETREQ to
    # perform a soft reset
    cortex_m reset_config sysresetreq
}

$_TARGETNAME configure -event examine-end {
    # DBGMCU_CR |= DBG_WWDG_STOP | DBG_IWDG_STOP |
    #              DBG_STANDBY | DBG_STOP | DBG_SLEEP
    mmw 0xE0042004 0x00000307 0
}

$_TARGETNAME configure -event trace-config {
    # Set TRACE_IOEN; TRACE_MODE is set to async; when using sync
    # change this value accordingly to configure trace pins
    # assignment
    mmw 0xE0042004 0x00000020 0
}

```
:::


## JLINK 配置

::: tip 增加新SOC支持
  -   创建目录 '~/Library/Application Support/SEGGER/JLinkDevices/WCH/WH32F2xx'
  -   新目录下创建配置文件 Devices.xml
  ```  
  <Database>
      <Device>
        <ChipInfo Vendor="WCH" Name="WH32F2xx" WorkRAMAddr="0x20000000" WorkRAMSize="0x20000" Core="JLINK_CORE_CORTEX_M3" />
        <FlashBankInfo Name="Flash_128k" BaseAddr="0x08000000" MaxSize="0x20000" Loader="CH32F2xx.FLM" LoaderType="FLASH_ALGO_TYPE_OPEN" />
      </Device>
    </Database>
  ```
  - 将烧录算法(在沁恒提供的SDK内)CH32F2xx.FLM 拷贝到新创建的目录
  ```
  └── WH32F2xx
    ├── CH32F2xx.FLM
    └── Devices.xml
  ```  
:::





