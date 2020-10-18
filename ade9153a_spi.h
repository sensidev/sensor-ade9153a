#ifndef SENSOR_ADE9153A_SPI_H
#define SENSOR_ADE9153A_SPI_H

void ade9153a_spi_write(uint16_t cmd, uint8_t *data, uint16_t length);

void ade9153a_spi_read(uint16_t cmd, uint8_t *data, uint16_t length);

void ade9153a_spi_delay_ms(uint32_t delay);

#endif
