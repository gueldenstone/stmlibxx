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
#include <cstdarg>

#include "stmlibxx/gpio.hpp"

// initialize GPIO
stmlibxx::GPIO GPIO_A(GPIOA);
stmlibxx::GPIO GPIO_B(GPIOB);
stmlibxx::GPIO GPIO_C(GPIOC);
namespace stmlibxx {

GPIO_State GPIO::read_pin(const GPIO_Pin& pin_num) {
  // read IDR Register
  return (GPIO_State)((gpio->IDR & 1 << pin_num) >> pin_num);
}

void GPIO::pin_on(const GPIO_Pin& pin_num) {
  // setting 1 pin to on state
  gpio->BSRR |= 0b1 << pin_num;
}

void GPIO::pin_off(const GPIO_Pin& pin_num) {
  // setting 1 pin to off state
  gpio->BSRR |= 0b1 << (16 + pin_num);
}

// implementation functions of GPIO configuration, specific to device
void GPIO::pin_config_impl(const GPIO_Pin& pin_num, const GPIO_Mode& mode) {
  const int offset = pin_num * 2;
  gpio->MODER &= ~(0b11 << offset);  // reset to 0
  gpio->MODER |= mode << offset;     // set to desired mode
}

void GPIO::pin_config_impl(const GPIO_Pin& pin_num, const GPIO_OutputType& otype) {
  const int offset = pin_num * 1;
  gpio->OTYPER &= ~(0b1 << offset);  // reset to 0
  gpio->OTYPER |= otype << offset;
}

void GPIO::pin_config_impl(const GPIO_Pin& pin_num, const GPIO_Speed& ospeed) {
  const int offset = pin_num * 2;
  gpio->OSPEEDR &= ~(0b11 << offset);  // reset to 0
  gpio->OSPEEDR |= ospeed << offset;
}

void GPIO::pin_config_impl(const GPIO_Pin& pin_num, const GPIO_PullUpDown& pupd) {
  const int offset = pin_num * 2;
  gpio->PUPDR &= ~(0b11 << offset);  // reset to 0
  gpio->PUPDR |= pupd << offset;
}

void GPIO::pin_config_impl(const GPIO_Pin& pin_num, const GPIO_AlternateFunction& af) {
  const int offset1 = pin_num * 4, offset2 = (pin_num - 8) * 4;
  if (pin_num < 8) {
    gpio->AFR[0] &= ~(0b1111 << offset1);  // reset to 0
    gpio->AFR[0] |= af << offset1;
  } else {
    gpio->AFR[1] &= ~(0b1111 << offset2);  // reset to 0
    gpio->AFR[1] |= af << offset2;
  }
}
}  // namespace stmlibxx
