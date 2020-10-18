#ifndef SENSOR_ADE9153A_SPI_H
#define SENSOR_ADE9153A_SPI_H

#include "stdint.h"
#include "ade9153a_errors.h"

/**
 * SPI write data buffer to slave.
 * @param data - bytes to transmit, considering the first two bytes the command header (CMD_HDR).
 * @param length - total bytes count.
 */
ADE9153AStatus_t ade9153a_spi_write(uint8_t *data, uint8_t length);

/**
 * SPI read data buffer from slave.
 * @param cmd - command header (CMD_HDR).
 * @param data - bytes received from slave.
 * @param length - total bytes count.
 */
ADE9153AStatus_t ade9153a_spi_read(uint16_t cmd, uint8_t *data, uint8_t length);

/**
 * Delay in milliseconds.
 * @param delay
 */
void ade9153a_spi_delay_ms(uint32_t delay);

#endif
