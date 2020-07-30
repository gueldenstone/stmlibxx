/**
 * @file: gpio.hpp
 *
 * @date: 2020-07-29
 *
 * @author: Lukas Güldenstein
 */

#ifndef _GPIO_HPP
#define _GPIO_HPP

/* -------------------------------- includes -------------------------------- */
#ifdef STM32G474xx
#include "stm32g474xx.h"
#endif

typedef enum GPIO_Pin { P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15 } GPIO_Pin_Type;
typedef enum GPIO_Mode { Input = 0b00, Output = 0b01, Alternate = 0b10, Analog = 11 } GPIO_Mode_Type;
typedef enum GPIO_OutputType { PushPull = 0b0, OpenDrain = 0b1 } GPIO_OutputType_Type;
typedef enum GPIO_Speed {
  LowSpeed = 0b00,
  MediumSpeed = 0b01,
  HighSpeed = 0b10,
  VeryHighSpeed = 0b11
} GPIO_Speed_Type;
typedef enum GPIO_PullUpDown { noPUPD = 0b00, PullUp = 0b01, PullDown = 0b10 } GPIO_PullUpDown_Type;
typedef enum GPIO_State { Off = 0, On = 1 } GPIO_State_Type;
typedef enum GPIO_AlternateFunction {
  AF0 = 0b0000,
  AF1 = 0b0001,
  AF2 = 0b0010,
  AF3 = 0b0011,
  AF4 = 0b0100,
  AF5 = 0b0101,
  AF6 = 0b0110,
  AF7 = 0b0111,
  AF8 = 0b1000,
  AF9 = 0b1001,
  AF10 = 0b1010,
  AF11 = 0b1011,
  AF12 = 0b1100,
  AF13 = 0b1101,
  AF14 = 0b1110,
  AF15 = 0b1111
} GPIO_AlternateFunction_Type;

class GPIO {
 public:
  GPIO() {
    while (1) {
      __NOP();
    }
  }
  GPIO(GPIO_TypeDef* bank) : gpio(bank) {
    if (bank == GPIOA) {
      RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    } else if (bank == GPIOB) {
      RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    } else if (bank == GPIOC) {
      RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    } else if (bank == GPIOD) {
      RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
    } else if (bank == GPIOE) {
      RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
    } else if (bank == GPIOF) {
      RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
    } else if (bank == GPIOG) {
      RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
    }
  }
  ~GPIO() = default;

 public:
  GPIO_State_Type read_pin(const GPIO_Pin_Type& pin_num);
  void pin_on(const GPIO_Pin_Type& pin_num);
  void pin_off(const GPIO_Pin_Type& pin_num);

 public:
  void set_pin_config(const GPIO_Pin_Type& pin_num, const GPIO_Mode& mode, const GPIO_OutputType& otype,
                      const GPIO_Speed& ospeed, const GPIO_PullUpDown_Type& pupd,
                      const GPIO_AlternateFunction_Type& af) {
    set_pin_mode(pin_num, mode);
    set_pin_type(pin_num, otype);
    set_pin_speed(pin_num, ospeed);
    set_pin_pupd(pin_num, pupd);
    set_pin_af(pin_num, af);
  }

  void set_pin_config(const GPIO_Pin_Type& pin_num, const GPIO_Mode& mode,
                      const GPIO_AlternateFunction_Type& af) {
    set_pin_mode(pin_num, mode);
    set_pin_af(pin_num, af);
  }
  void set_pin_config(const GPIO_Pin_Type& pin_num, const GPIO_Mode& mode, const GPIO_OutputType& otype) {
    set_pin_mode(pin_num, mode);
    set_pin_type(pin_num, otype);
  }
  void set_pin_config(const GPIO_Pin_Type& pin_num, const GPIO_Mode& mode) { set_pin_mode(pin_num, mode); }

 private:
  void set_pin_mode(const GPIO_Pin_Type& pin_num, const GPIO_Mode& mode);
  void set_pin_type(const GPIO_Pin_Type& pin_num, const GPIO_OutputType& otype);
  void set_pin_speed(const GPIO_Pin_Type& pin_num, const GPIO_Speed& ospeed);
  void set_pin_pupd(const GPIO_Pin_Type& pin_num, const GPIO_PullUpDown_Type& pupd);
  void set_pin_af(const GPIO_Pin_Type& pin_num, const GPIO_AlternateFunction_Type& af);

 protected:
  GPIO_TypeDef* gpio = nullptr;

 private:
};

// declare GPIO Banks
extern GPIO GPIO_A;
extern GPIO GPIO_B;
extern GPIO GPIO_C;

#endif /* _GPIO_HPP */