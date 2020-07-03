/**
 * @file: stm32g474xx_it.cpp
 *
 * @date: 2020-07-03
 *
 * @author: Lukas Güldenstein
 *
 * @brief: implementation of the interrupts on stm32g474xx
 */

/* -------------------------------- includes -------------------------------- */
#include "stm32g474xx_it.hpp"

Interrupt *Interrupt::ISRVectorTable[];