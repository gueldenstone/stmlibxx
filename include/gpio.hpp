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
#include "etl/vector.h"

enum GPIO_Pin { P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15 };
enum GPIO_Mode { Input = 0b00, Output = 0b01, Alternate = 0b10, Analog = 11 };
enum GPIO_OutputType { PushPull = 0b0, OpenDrain = 0b1 };
enum GPIO_Speed { LowSpeed = 0b00, MediumSpeed = 0b01, HighSpeed = 0b10, VeryHighSpeed = 0b11 };
enum GPIO_PullUpDown { noPUPD = 0b00, PullUp = 0b01, PullDown = 0b10 };
enum GPIO_State { Off = 0, On = 1 };
enum GPIO_AlternateFunction {
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
};

class GPIO {
 public:
  GPIO() = delete;
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
  GPIO_State read_pin(const GPIO_Pin& pin_num);
  void pin_on(const GPIO_Pin& pin_num);
  void pin_off(const GPIO_Pin& pin_num);

 private:
  void pin_config_impl(const GPIO_Pin& pin_num, const GPIO_Mode& mode);
  void pin_config_impl(const GPIO_Pin& pin_num, const GPIO_OutputType& otype);
  void pin_config_impl(const GPIO_Pin& pin_num, const GPIO_Speed& ospeed);
  void pin_config_impl(const GPIO_Pin& pin_num, const GPIO_PullUpDown& pupd);
  void pin_config_impl(const GPIO_Pin& pin_num, const GPIO_AlternateFunction& af);

 public:
  //  variadic template function which iterates over all given configs
  template <typename... Configs>
  void set_pin_config(const GPIO_Pin& pin_num, const Configs&... configs) {
    auto call = [&](auto&& config) { pin_config_impl(pin_num, config); };
    (call(configs), ...);
  }

 protected:
  GPIO_TypeDef* gpio = nullptr;
};

// declare GPIO Banks
extern GPIO GPIO_A;
extern GPIO GPIO_B;
extern GPIO GPIO_C;

struct GPIO_Pin_Config {
  GPIO_Pin_Config() = default;
  GPIO_Pin_Config(GPIO& bank, const etl::vector<GPIO_Pin, 16>& pin_nums)
      : m_bank(bank), m_pin_nums(pin_nums) {}
  GPIO_Pin_Config(const etl::vector<GPIO_Pin, 16>& pin_nums) : m_pin_nums(pin_nums) {}

  GPIO_Mode Mode = Analog;
  GPIO_OutputType OutputType = PushPull;
  GPIO_Speed OutputSpeed = LowSpeed;
  GPIO_PullUpDown PullUpPullDown = noPUPD;
  GPIO_AlternateFunction AlternateFunction = AF0;

 private:
  GPIO& m_bank = GPIO_A;
  etl::vector<GPIO_Pin, 16> m_pin_nums;

 public:
  void configure(void) {
    for (auto&& pin : m_pin_nums) {
      m_bank.set_pin_config(pin, Mode, OutputType, OutputSpeed, PullUpPullDown, AlternateFunction);
    }
  }
  void configure_bank(GPIO& bank, const etl::vector<GPIO_Pin, 16>& pin_nums) {
    m_pin_nums = pin_nums;
    m_bank = bank;
    configure();
  }
};

#endif /* _GPIO_HPP */