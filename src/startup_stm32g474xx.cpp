/**
 * @file: startup_stm32g474xx.cpp
 *
 * @date: 2020-06-22
 *
 * @author: Lukas Güldenstein
 *
 * @brief: Startup Code for STM32G474xx
 */

/* -------------------------------- includes -------------------------------- */
#include <algorithm>
#include <cstdint>

#include "stmlibxx/interrupt.hpp"

#define DEFINE_DEFAULT_ISR(name)                                               \
  extern "C" __attribute__((interrupt)) __attribute__((weak))                  \
  __attribute__((noreturn)) void                                               \
  name() {                                                                     \
    while (true)                                                               \
      ;                                                                        \
  }
DEFINE_DEFAULT_ISR(defaultISR)
DEFINE_DEFAULT_ISR(NMI_Handler)
DEFINE_DEFAULT_ISR(HardFault_Handler)
DEFINE_DEFAULT_ISR(MemManage_Handler)
DEFINE_DEFAULT_ISR(BusFault_Handler)
DEFINE_DEFAULT_ISR(UsageFault_Handler)
DEFINE_DEFAULT_ISR(SVC_Handler)
DEFINE_DEFAULT_ISR(DebugMon_Handler)
DEFINE_DEFAULT_ISR(PendSV_Handler)
DEFINE_DEFAULT_ISR(SysTick_Handler)
DEFINE_DEFAULT_ISR(WWDG_IRQHandler)
DEFINE_DEFAULT_ISR(PVD_PVM_IRQHandler)
DEFINE_DEFAULT_ISR(RTC_TAMP_LSECSS_IRQHandler)
DEFINE_DEFAULT_ISR(RTC_WKUP_IRQHandler)
DEFINE_DEFAULT_ISR(FLASH_IRQHandler)
DEFINE_DEFAULT_ISR(RCC_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI0_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI1_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI2_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI3_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI4_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel1_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel2_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel3_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel4_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel5_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel6_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel7_IRQHandler)
DEFINE_DEFAULT_ISR(ADC1_2_IRQHandler)
DEFINE_DEFAULT_ISR(USB_HP_IRQHandler)
DEFINE_DEFAULT_ISR(USB_LP_IRQHandler)
DEFINE_DEFAULT_ISR(FDCAN1_IT0_IRQHandler)
DEFINE_DEFAULT_ISR(FDCAN1_IT1_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI9_5_IRQHandler)
DEFINE_DEFAULT_ISR(TIM1_BRK_TIM15_IRQHandler)
DEFINE_DEFAULT_ISR(TIM1_UP_TIM16_IRQHandler)
DEFINE_DEFAULT_ISR(TIM1_TRG_COM_TIM17_IRQHandler)
DEFINE_DEFAULT_ISR(TIM1_CC_IRQHandler)
DEFINE_DEFAULT_ISR(TIM2_IRQHandler)
DEFINE_DEFAULT_ISR(TIM3_IRQHandler)
DEFINE_DEFAULT_ISR(TIM4_IRQHandler)
DEFINE_DEFAULT_ISR(I2C1_EV_IRQHandler)
DEFINE_DEFAULT_ISR(I2C1_ER_IRQHandler)
DEFINE_DEFAULT_ISR(I2C2_EV_IRQHandler)
DEFINE_DEFAULT_ISR(I2C2_ER_IRQHandler)
DEFINE_DEFAULT_ISR(SPI1_IRQHandler)
DEFINE_DEFAULT_ISR(SPI2_IRQHandler)
DEFINE_DEFAULT_ISR(USART1_IRQHandler)
DEFINE_DEFAULT_ISR(USART2_IRQHandler)
DEFINE_DEFAULT_ISR(USART3_IRQHandler)
DEFINE_DEFAULT_ISR(EXTI15_10_IRQHandler)
DEFINE_DEFAULT_ISR(RTC_Alarm_IRQHandler)
DEFINE_DEFAULT_ISR(USBWakeUp_IRQHandler)
DEFINE_DEFAULT_ISR(TIM8_BRK_IRQHandler)
DEFINE_DEFAULT_ISR(TIM8_UP_IRQHandler)
DEFINE_DEFAULT_ISR(TIM8_TRG_COM_IRQHandler)
DEFINE_DEFAULT_ISR(TIM8_CC_IRQHandler)
DEFINE_DEFAULT_ISR(ADC3_IRQHandler)
DEFINE_DEFAULT_ISR(FMC_IRQHandler)
DEFINE_DEFAULT_ISR(LPTIM1_IRQHandler)
DEFINE_DEFAULT_ISR(TIM5_IRQHandler)
DEFINE_DEFAULT_ISR(SPI3_IRQHandler)
DEFINE_DEFAULT_ISR(UART4_IRQHandler)
DEFINE_DEFAULT_ISR(UART5_IRQHandler)
DEFINE_DEFAULT_ISR(TIM6_DAC_IRQHandler)
DEFINE_DEFAULT_ISR(TIM7_DAC_IRQHandler)
DEFINE_DEFAULT_ISR(DMA2_Channel1_IRQHandler)
DEFINE_DEFAULT_ISR(DMA2_Channel2_IRQHandler)
DEFINE_DEFAULT_ISR(DMA2_Channel3_IRQHandler)
DEFINE_DEFAULT_ISR(DMA2_Channel4_IRQHandler)
DEFINE_DEFAULT_ISR(DMA2_Channel5_IRQHandler)
DEFINE_DEFAULT_ISR(ADC4_IRQHandler)
DEFINE_DEFAULT_ISR(ADC5_IRQHandler)
DEFINE_DEFAULT_ISR(UCPD1_IRQHandler)
DEFINE_DEFAULT_ISR(COMP1_2_3_IRQHandler)
DEFINE_DEFAULT_ISR(COMP4_5_6_IRQHandler)
DEFINE_DEFAULT_ISR(COMP7_IRQHandler)
DEFINE_DEFAULT_ISR(HRTIM1_Master_IRQHandler)
DEFINE_DEFAULT_ISR(HRTIM1_TIMA_IRQHandler)
DEFINE_DEFAULT_ISR(HRTIM1_TIMB_IRQHandler)
DEFINE_DEFAULT_ISR(HRTIM1_TIMC_IRQHandler)
DEFINE_DEFAULT_ISR(HRTIM1_TIMD_IRQHandler)
DEFINE_DEFAULT_ISR(HRTIM1_TIME_IRQHandler)
DEFINE_DEFAULT_ISR(HRTIM1_FLT_IRQHandler)
DEFINE_DEFAULT_ISR(HRTIM1_TIMF_IRQHandler)
DEFINE_DEFAULT_ISR(CRS_IRQHandler)
DEFINE_DEFAULT_ISR(SAI1_IRQHandler)
DEFINE_DEFAULT_ISR(TIM20_BRK_IRQHandler)
DEFINE_DEFAULT_ISR(TIM20_UP_IRQHandler)
DEFINE_DEFAULT_ISR(TIM20_TRG_COM_IRQHandler)
DEFINE_DEFAULT_ISR(TIM20_CC_IRQHandler)
DEFINE_DEFAULT_ISR(FPU_IRQHandler)
DEFINE_DEFAULT_ISR(I2C4_EV_IRQHandler)
DEFINE_DEFAULT_ISR(I2C4_ER_IRQHandler)
DEFINE_DEFAULT_ISR(SPI4_IRQHandler)
DEFINE_DEFAULT_ISR(FDCAN2_IT0_IRQHandler)
DEFINE_DEFAULT_ISR(FDCAN2_IT1_IRQHandler)
DEFINE_DEFAULT_ISR(FDCAN3_IT0_IRQHandler)
DEFINE_DEFAULT_ISR(FDCAN3_IT1_IRQHandler)
DEFINE_DEFAULT_ISR(RNG_IRQHandler)
DEFINE_DEFAULT_ISR(LPUART1_IRQHandler)
DEFINE_DEFAULT_ISR(I2C3_EV_IRQHandler)
DEFINE_DEFAULT_ISR(I2C3_ER_IRQHandler)
DEFINE_DEFAULT_ISR(DMAMUX_OVR_IRQHandler)
DEFINE_DEFAULT_ISR(QUADSPI_IRQHandler)
DEFINE_DEFAULT_ISR(DMA1_Channel8_IRQHandler)
DEFINE_DEFAULT_ISR(DMA2_Channel6_IRQHandler)
DEFINE_DEFAULT_ISR(DMA2_Channel7_IRQHandler)
DEFINE_DEFAULT_ISR(DMA2_Channel8_IRQHandler)
DEFINE_DEFAULT_ISR(CORDIC_IRQHandler)
DEFINE_DEFAULT_ISR(FMAC_IRQHandler)
extern std::uint32_t _estack;
extern "C" void Reset_Handler();

namespace stmlibxx {
std::uintptr_t g_pfnVectors[] __attribute__((section(".isr_vector"))){
    // Stack Ptr initialization
    reinterpret_cast<std::uintptr_t>(&_estack),
    // Entry point
    reinterpret_cast<std::uintptr_t>(Reset_Handler),
    // Exceptions
    reinterpret_cast<std::uintptr_t>(NMI_Handler),
    /* NMI_Handler
     */
    reinterpret_cast<std::uintptr_t>(HardFault_Handler), /* HardFault_Handler */
    reinterpret_cast<std::uintptr_t>(MemManage_Handler), /* MemManage_Handler */
    reinterpret_cast<std::uintptr_t>(BusFault_Handler),  /* BusFault_Handler */
    reinterpret_cast<std::uintptr_t>(UsageFault_Handler), /* UsageFault_Handler
                                                           */
    reinterpret_cast<std::uintptr_t>(nullptr),            /* 0 */
    reinterpret_cast<std::uintptr_t>(nullptr),            /* 0 */
    reinterpret_cast<std::uintptr_t>(nullptr),            /* 0 */
    reinterpret_cast<std::uintptr_t>(nullptr),            /* 0 */
    reinterpret_cast<std::uintptr_t>(SVC_Handler),
    /* SVC_Handler
     */
    reinterpret_cast<std::uintptr_t>(DebugMon_Handler), /* DebugMon_Handler */
    reinterpret_cast<std::uintptr_t>(nullptr),          /* 0 */
    reinterpret_cast<std::uintptr_t>(PendSV_Handler),
    /* PendSV_Handler
     */
    reinterpret_cast<std::uintptr_t>(SysTick_Handler), /* SysTick_Handler */

    reinterpret_cast<std::uintptr_t>(stmlibxx::Interrupt_Base::WWDG_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::PVD_PVM_IRQHandler),
    reinterpret_cast<std::uintptr_t>(
        Interrupt_Base::RTC_TAMP_LSECSS_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::RTC_WKUP_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FLASH_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::RCC_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::EXTI0_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::EXTI1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::EXTI2_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::EXTI3_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::EXTI4_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA1_Channel1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA1_Channel2_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA1_Channel3_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA1_Channel4_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA1_Channel5_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA1_Channel6_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA1_Channel7_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::ADC1_2_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::USB_HP_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::USB_LP_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FDCAN1_IT0_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FDCAN1_IT1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::EXTI9_5_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM1_BRK_TIM15_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM1_UP_TIM16_IRQHandler),
    reinterpret_cast<std::uintptr_t>(
        Interrupt_Base::TIM1_TRG_COM_TIM17_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM1_CC_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM2_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM3_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM4_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::I2C1_EV_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::I2C1_ER_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::I2C2_EV_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::I2C2_ER_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::SPI1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::SPI2_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::USART1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::USART2_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::USART3_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::EXTI15_10_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::RTC_Alarm_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::USBWakeUp_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM8_BRK_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM8_UP_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM8_TRG_COM_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM8_CC_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::ADC3_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FMC_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::LPTIM1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM5_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::SPI3_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::UART4_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::UART5_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM6_DAC_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM7_DAC_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA2_Channel1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA2_Channel2_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA2_Channel3_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA2_Channel4_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA2_Channel5_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::ADC4_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::ADC5_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::UCPD1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::COMP1_2_3_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::COMP4_5_6_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::COMP7_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::HRTIM1_Master_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::HRTIM1_TIMA_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::HRTIM1_TIMB_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::HRTIM1_TIMC_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::HRTIM1_TIMD_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::HRTIM1_TIME_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::HRTIM1_FLT_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::HRTIM1_TIMF_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::CRS_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::SAI1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM20_BRK_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM20_UP_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM20_TRG_COM_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::TIM20_CC_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FPU_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::I2C4_EV_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::I2C4_ER_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::SPI4_IRQHandler),
    reinterpret_cast<std::uintptr_t>(nullptr),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FDCAN2_IT0_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FDCAN2_IT1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FDCAN3_IT0_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FDCAN3_IT1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::RNG_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::LPUART1_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::I2C3_EV_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::I2C3_ER_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMAMUX_OVR_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::QUADSPI_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA1_Channel8_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA2_Channel6_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA2_Channel7_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::DMA2_Channel8_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::CORDIC_IRQHandler),
    reinterpret_cast<std::uintptr_t>(Interrupt_Base::FMAC_IRQHandler)};

Interrupt_Base *Interrupt_Base::ISRVectorTable[];
} // namespace stmlibxx
extern "C" void Reset_Handler() {
  // Initialize data section
  extern std::uint8_t _sdata;
  extern std::uint8_t _edata;
  extern std::uint8_t _etext;
  std::size_t size = static_cast<size_t>(&_edata - &_sdata);
  std::copy(&_etext, &_etext + size, &_sdata);

  // Initialize bss section
  extern std::uint8_t _sbss;
  extern std::uint8_t _ebss;
  std::fill(&_sbss, &_ebss, UINT8_C(0x00));

  /* Call the clock system intitialization function.*/
  asm("bl SystemInit");
  /* Call static constructors */
  asm("bl __libc_init_array");
  /* Call the application's entry point.*/
  asm("bl main");
}
