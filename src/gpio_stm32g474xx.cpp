/**
 * @file: gpio_stm32g474xx.cpp
 *
 * @date: 2020-07-30
 *
 * @author: Lukas Güldenstein
 *
 * @brief: Implementation of GPIO driver functions
 */

/* -------------------------------- includes -------------------------------- */
#include "gpio.hpp"

// initialize GPIO
GPIO GPIO_A(GPIOA);
GPIO GPIO_B(GPIOB);
GPIO GPIO_C(GPIOC);

GPIO_State_Type GPIO::read_pin(const GPIO_Pin_Type& pin_num) {
  // read IDR Register
  return (GPIO_State_Type)((gpio->IDR & pin_num) >> pin_num);
}

void GPIO::pin_on(const GPIO_Pin_Type& pin_num) {
  // setting 1 pin to on state
  gpio->BSRR |= 0b1 << pin_num;
}

void GPIO::pin_off(const GPIO_Pin_Type& pin_num) {
  // setting 1 pin to off state
  gpio->BSRR |= 0b1 << (16 + pin_num);
}

void GPIO::set_pin_mode(const GPIO_Pin_Type& pin_num, const GPIO_Mode& mode) {
  const int offset = pin_num * 2;
  gpio->MODER &= ~(0b11 << offset);  // reset to 0
  gpio->MODER |= mode << offset;     // set to desired mode
}

void GPIO::set_pin_type(const GPIO_Pin_Type& pin_num, const GPIO_OutputType& otype) {
  const int offset = pin_num * 1;
  gpio->OTYPER &= ~(0b1 << offset);  // reset to 0
  gpio->OTYPER |= otype << offset;
}

void GPIO::set_pin_speed(const GPIO_Pin_Type& pin_num, const GPIO_Speed& ospeed) {
  const int offset = pin_num * 2;
  gpio->OSPEEDR &= ~(0b11 << offset);  // reset to 0
  gpio->OSPEEDR |= ospeed << offset;
}

void GPIO::set_pin_pupd(const GPIO_Pin_Type& pin_num, const GPIO_PullUpDown_Type& pupd) {
  const int offset = pin_num * 2;
  gpio->PUPDR &= ~(0b11 << offset);  // reset to 0
  gpio->PUPDR |= pupd << offset;
}

void GPIO::set_pin_af(const GPIO_Pin_Type& pin_num, const GPIO_AlternateFunction_Type& af) {
  const int offset1 = pin_num * 4, offset2 = (pin_num - 8) * 4;
  if (pin_num < 8) {
    gpio->AFR[0] &= ~(0b1111 << offset1);  // reset to 0
    gpio->AFR[0] |= af << offset1;
  } else {
    gpio->AFR[1] &= ~(0b1111 << offset2);  // reset to 0
    gpio->AFR[1] |= af << offset2;
  }
}