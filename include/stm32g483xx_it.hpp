/**
 * @file: stm32g483xx_it.hpp
 *
 * @date: 2020-06-22
 *
 * @author: Lukas Güldenstein
 */

#ifndef _STM32G483XX_IT_HPP
#define _STM32G483XX_IT_HPP

/* -------------------------------- includes -------------------------------- */
#define MAX_INTERRUPTS 118
#include "stm32g4xx.h"
class Interrupt {
 public:
  // Constructor
  Interrupt();
  // Register function
  void Interrupt::Register(IRQn_Type interrupt_number, Interrupt* intThisPtr) {
    ISRVectorTable[interrupt_number] = intThisPtr;
  }
  // ISR functions
  static void WWDG_IRQHandler(void) { ISRVectorTable[WWDG_IRQn]->ISR(); }
  static void PVD_PVM_IRQHandler(void) { ISRVectorTable[PVD_PVM_IRQn]->ISR(); }
  static void RTC_TAMP_LSECSS_IRQHandler(void) { ISRVectorTable[RTC_TAMP_LSECSS_IRQn]->ISR(); }
  static void RTC_WKUP_IRQHandler(void) { ISRVectorTable[RTC_WKUP_IRQn]->ISR(); }
  static void FLASH_IRQHandler(void) { ISRVectorTable[FLASH_IRQn]->ISR(); }
  static void RCC_IRQHandler(void) { ISRVectorTable[RCC_IRQn]->ISR(); }
  static void EXTI0_IRQHandler(void) { ISRVectorTable[EXTI0_IRQn]->ISR(); }
  static void EXTI1_IRQHandler(void) { ISRVectorTable[EXTI1_IRQn]->ISR(); }
  static void EXTI2_IRQHandler(void) { ISRVectorTable[EXTI2_IRQn]->ISR(); }
  static void EXTI3_IRQHandler(void) { ISRVectorTable[EXTI3_IRQn]->ISR(); }
  static void EXTI4_IRQHandler(void) { ISRVectorTable[EXTI4_IRQn]->ISR(); }
  static void DMA1_Channel1_IRQHandler(void) { ISRVectorTable[DMA1_Channel1_IRQn]->ISR(); }
  static void DMA1_Channel2_IRQHandler(void) { ISRVectorTable[DMA1_Channel2_IRQn]->ISR(); }
  static void DMA1_Channel3_IRQHandler(void) { ISRVectorTable[DMA1_Channel3_IRQn]->ISR(); }
  static void DMA1_Channel4_IRQHandler(void) { ISRVectorTable[DMA1_Channel4_IRQn]->ISR(); }
  static void DMA1_Channel5_IRQHandler(void) { ISRVectorTable[DMA1_Channel5_IRQn]->ISR(); }
  static void DMA1_Channel6_IRQHandler(void) { ISRVectorTable[DMA1_Channel6_IRQn]->ISR(); }
  static void DMA1_Channel7_IRQHandler(void) { ISRVectorTable[DMA1_Channel7_IRQn]->ISR(); }
  static void ADC1_2_IRQHandler(void) { ISRVectorTable[ADC1_2_IRQn]->ISR(); }
  static void USB_HP_IRQHandler(void) { ISRVectorTable[USB_HP_IRQn]->ISR(); }
  static void USB_LP_IRQHandler(void) { ISRVectorTable[USB_LP_IRQn]->ISR(); }
  static void FDCAN1_IT0_IRQHandler(void) { ISRVectorTable[FDCAN1_IT0_IRQn]->ISR(); }
  static void FDCAN1_IT1_IRQHandler(void) { ISRVectorTable[FDCAN1_IT1_IRQn]->ISR(); }
  static void EXTI9_5_IRQHandler(void) { ISRVectorTable[EXTI9_5_IRQn]->ISR(); }
  static void TIM1_BRK_TIM15_IRQHandler(void) { ISRVectorTable[TIM1_BRK_TIM15_IRQn]->ISR(); }
  static void TIM1_UP_TIM16_IRQHandler(void) { ISRVectorTable[TIM1_UP_TIM16_IRQn]->ISR(); }
  static void TIM1_TRG_COM_TIM17_IRQHandler(void) { ISRVectorTable[TIM1_TRG_COM_TIM17_IRQn]->ISR(); }
  static void TIM1_CC_IRQHandler(void) { ISRVectorTable[TIM1_CC_IRQn]->ISR(); }
  static void TIM2_IRQHandler(void) { ISRVectorTable[TIM2_IRQn]->ISR(); }
  static void TIM3_IRQHandler(void) { ISRVectorTable[TIM3_IRQn]->ISR(); }
  static void TIM4_IRQHandler(void) { ISRVectorTable[TIM4_IRQn]->ISR(); }
  static void I2C1_EV_IRQHandler(void) { ISRVectorTable[I2C1_EV_IRQn]->ISR(); }
  static void I2C1_ER_IRQHandler(void) { ISRVectorTable[I2C1_ER_IRQn]->ISR(); }
  static void I2C2_EV_IRQHandler(void) { ISRVectorTable[I2C2_EV_IRQn]->ISR(); }
  static void I2C2_ER_IRQHandler(void) { ISRVectorTable[I2C2_ER_IRQn]->ISR(); }
  static void SPI1_IRQHandler(void) { ISRVectorTable[SPI1_IRQn]->ISR(); }
  static void SPI2_IRQHandler(void) { ISRVectorTable[SPI2_IRQn]->ISR(); }
  static void USART1_IRQHandler(void) { ISRVectorTable[USART1_IRQn]->ISR(); }
  static void USART2_IRQHandler(void) { ISRVectorTable[USART2_IRQn]->ISR(); }
  static void USART3_IRQHandler(void) { ISRVectorTable[USART3_IRQn]->ISR(); }
  static void EXTI15_10_IRQHandler(void) { ISRVectorTable[EXTI15_10_IRQn]->ISR(); }
  static void RTC_Alarm_IRQHandler(void) { ISRVectorTable[RTC_Alarm_IRQn]->ISR(); }
  static void USBWakeUp_IRQHandler(void) { ISRVectorTable[USBWakeUp_IRQn]->ISR(); }
  static void TIM8_BRK_IRQHandler(void) { ISRVectorTable[TIM8_BRK_IRQn]->ISR(); }
  static void TIM8_UP_IRQHandler(void) { ISRVectorTable[TIM8_UP_IRQn]->ISR(); }
  static void TIM8_TRG_COM_IRQHandler(void) { ISRVectorTable[TIM8_TRG_COM_IRQn]->ISR(); }
  static void TIM8_CC_IRQHandler(void) { ISRVectorTable[TIM8_CC_IRQn]->ISR(); }
  static void ADC3_IRQHandler(void) { ISRVectorTable[ADC3_IRQn]->ISR(); }
  static void FMC_IRQHandler(void) { ISRVectorTable[FMC_IRQn]->ISR(); }
  static void LPTIM1_IRQHandler(void) { ISRVectorTable[LPTIM1_IRQn]->ISR(); }
  static void TIM5_IRQHandler(void) { ISRVectorTable[TIM5_IRQn]->ISR(); }
  static void SPI3_IRQHandler(void) { ISRVectorTable[SPI3_IRQn]->ISR(); }
  static void UART4_IRQHandler(void) { ISRVectorTable[UART4_IRQn]->ISR(); }
  static void UART5_IRQHandler(void) { ISRVectorTable[UART5_IRQn]->ISR(); }
  static void TIM6_DAC_IRQHandler(void) { ISRVectorTable[TIM6_DAC_IRQn]->ISR(); }
  static void TIM7_DAC_IRQHandler(void) { ISRVectorTable[TIM7_DAC_IRQn]->ISR(); }
  static void DMA2_Channel1_IRQHandler(void) { ISRVectorTable[DMA2_Channel1_IRQn]->ISR(); }
  static void DMA2_Channel2_IRQHandler(void) { ISRVectorTable[DMA2_Channel2_IRQn]->ISR(); }
  static void DMA2_Channel3_IRQHandler(void) { ISRVectorTable[DMA2_Channel3_IRQn]->ISR(); }
  static void DMA2_Channel4_IRQHandler(void) { ISRVectorTable[DMA2_Channel4_IRQn]->ISR(); }
  static void DMA2_Channel5_IRQHandler(void) { ISRVectorTable[DMA2_Channel5_IRQn]->ISR(); }
  static void ADC4_IRQHandler(void) { ISRVectorTable[ADC4_IRQn]->ISR(); }
  static void ADC5_IRQHandler(void) { ISRVectorTable[ADC5_IRQn]->ISR(); }
  static void UCPD1_IRQHandler(void) { ISRVectorTable[UCPD1_IRQn]->ISR(); }
  static void COMP1_2_3_IRQHandler(void) { ISRVectorTable[COMP1_2_3_IRQn]->ISR(); }
  static void COMP4_5_6_IRQHandler(void) { ISRVectorTable[COMP4_5_6_IRQn]->ISR(); }
  static void COMP7_IRQHandler(void) { ISRVectorTable[COMP7_IRQn]->ISR(); }
  static void CRS_IRQHandler(void) { ISRVectorTable[CRS_IRQn]->ISR(); }
  static void SAI1_IRQHandler(void) { ISRVectorTable[SAI1_IRQn]->ISR(); }
  static void TIM20_BRK_IRQHandler(void) { ISRVectorTable[TIM20_BRK_IRQn]->ISR(); }
  static void TIM20_UP_IRQHandler(void) { ISRVectorTable[TIM20_UP_IRQn]->ISR(); }
  static void TIM20_TRG_COM_IRQHandler(void) { ISRVectorTable[TIM20_TRG_COM_IRQn]->ISR(); }
  static void TIM20_CC_IRQHandler(void) { ISRVectorTable[TIM20_CC_IRQn]->ISR(); }
  static void FPU_IRQHandler(void) { ISRVectorTable[FPU_IRQn]->ISR(); }
  static void I2C4_EV_IRQHandler(void) { ISRVectorTable[I2C4_EV_IRQn]->ISR(); }
  static void I2C4_ER_IRQHandler(void) { ISRVectorTable[I2C4_ER_IRQn]->ISR(); }
  static void SPI4_IRQHandler(void) { ISRVectorTable[SPI4_IRQn]->ISR(); }
  static void AES_IRQHandler(void) { ISRVectorTable[AES_IRQn]->ISR(); }
  static void FDCAN2_IT0_IRQHandler(void) { ISRVectorTable[FDCAN2_IT0_IRQn]->ISR(); }
  static void FDCAN2_IT1_IRQHandler(void) { ISRVectorTable[FDCAN2_IT1_IRQn]->ISR(); }
  static void FDCAN3_IT0_IRQHandler(void) { ISRVectorTable[FDCAN3_IT0_IRQn]->ISR(); }
  static void FDCAN3_IT1_IRQHandler(void) { ISRVectorTable[FDCAN3_IT1_IRQn]->ISR(); }
  static void RNG_IRQHandler(void) { ISRVectorTable[RNG_IRQn]->ISR(); }
  static void LPUART1_IRQHandler(void) { ISRVectorTable[LPUART1_IRQn]->ISR(); }
  static void I2C3_EV_IRQHandler(void) { ISRVectorTable[I2C3_EV_IRQn]->ISR(); }
  static void I2C3_ER_IRQHandler(void) { ISRVectorTable[I2C3_ER_IRQn]->ISR(); }
  static void DMAMUX_OVR_IRQHandler(void) { ISRVectorTable[DMAMUX_OVR_IRQn]->ISR(); }
  static void QUADSPI_IRQHandler(void) { ISRVectorTable[QUADSPI_IRQn]->ISR(); }
  static void DMA1_Channel8_IRQHandler(void) { ISRVectorTable[DMA1_Channel8_IRQn]->ISR(); }
  static void DMA2_Channel6_IRQHandler(void) { ISRVectorTable[DMA2_Channel6_IRQn]->ISR(); }
  static void DMA2_Channel7_IRQHandler(void) { ISRVectorTable[DMA2_Channel7_IRQn]->ISR(); }
  static void DMA2_Channel8_IRQHandler(void) { ISRVectorTable[DMA2_Channel8_IRQn]->ISR(); }
  static void CORDIC_IRQHandler(void) { ISRVectorTable[CORDIC_IRQn]->ISR(); }
  static void FMAC_IRQHandler(void) { ISRVectorTable[FMAC_IRQn]->ISR(); }

  virtual void ISR(void) = 0;

 private:
  static Interrupt* ISRVectorTable[MAX_INTERRUPTS - NVIC_USER_IRQ_OFFSET];
};

#endif /* _STM32G483XX_IT_HPP */