/**************************************************************************//**
 * @file     startup_ARMCM3.S
 * @brief    CMSIS-Core(M) Device Startup File for Cortex-M3 Device
 * @version  V2.2.0
 * @date
 ******************************************************************************/

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

Def_Week_Handler(SystemInit)

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

