/**
 * @file hal_rp2040.h
 * @brief RP2040 (Raspberry Pi Pico SDK + FreeRTOS) implementation of hal.h.
 *
 * I2C0 at 400 kHz on GP4 (SDA) / GP5 (SCL); console on UART0 GP0/GP1.
 */
#ifndef HAL_RP2040_H
#define HAL_RP2040_H

#include "hal/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_RP2040_I2C_HZ (400000u)
#define HAL_RP2040_SDA_PIN (4u)
#define HAL_RP2040_SCL_PIN (5u)

/** Configure I2C0 and its pins. Call once before using the bus. */
void hal_rp2040_init(void);

/** Handle for I2C0. */
hal_i2c_bus_t *hal_rp2040_i2c0(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_RP2040_H */
