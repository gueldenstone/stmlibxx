/**
 * @file: timer.hpp
 *
 * @date: 2020-05-02
 *
 * @author: Lukas Güldenstein
 */

#ifndef _TIMER_HPP
#define _TIMER_HPP

/* -------------------------------- includes -------------------------------- */
#include "stm32f303xe_it.hpp"
#include "stm32f3xx.h"

/* --------------------------- function prototypes -------------------------- */

class Timer;  // forward declaration

class TimerInterrupt : public Interrupt {
 public:
  TimerInterrupt(Timer* ownerptr);
  virtual void ISR(void);

 private:
  Timer* InterruptOwnerPtr;
};

class Timer {
  friend class TimerInterrupt;

 public:
  Timer(void);
  int GetCount(void) { return Count; }

 private:
  // Timer registers
  __IO uint32_t CR1;   /*!< TIM control register 1,              Address offset: 0x00 */
  __IO uint32_t CR2;   /*!< TIM control register 2,              Address offset: 0x04 */
  __IO uint32_t SMCR;  /*!< TIM slave mode control register,     Address offset: 0x08 */
  __IO uint32_t DIER;  /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __IO uint32_t SR;    /*!< TIM status register,                 Address offset: 0x10 */
  __IO uint32_t EGR;   /*!< TIM event generation register,       Address offset: 0x14 */
  __IO uint32_t CCMR1; /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __IO uint32_t CCMR2; /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __IO uint32_t CCER;  /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __IO uint32_t CNT;   /*!< TIM counter register,                Address offset: 0x24 */
  __IO uint32_t PSC;   /*!< TIM prescaler,                       Address offset: 0x28 */
  __IO uint32_t ARR;   /*!< TIM auto-reload register,            Address offset: 0x2C */
  __IO uint32_t RCR;   /*!< TIM repetition counter register,     Address offset: 0x30 */
  __IO uint32_t CCR1;  /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __IO uint32_t CCR2;  /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __IO uint32_t CCR3;  /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __IO uint32_t CCR4;  /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __IO uint32_t BDTR;  /*!< TIM break and dead-time register,    Address offset: 0x44 */
  __IO uint32_t DCR;   /*!< TIM DMA control register,            Address offset: 0x48 */
  __IO uint32_t DMAR;  /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  __IO uint32_t OR;    /*!< TIM option register,                 Address offset: 0x50 */
  __IO uint32_t CCMR3; /*!< TIM capture/compare mode register 3, Address offset: 0x54 */
  __IO uint32_t CCR5;  /*!< TIM capture/compare register5,       Address offset: 0x58 */
  __IO uint32_t CCR6;  /*!< TIM capture/compare register 4,      Address offset: 0x5C */
  TimerInterrupt* InterruptPtr;
  int Count;
};

#endif /* _TIMER_HPP */