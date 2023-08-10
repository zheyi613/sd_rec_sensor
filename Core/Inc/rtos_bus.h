/**
 * @file rtos_bus.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief manage blocking mode and interrupt mode when RTOS is running
 * @date 2023-08-10
 */
#ifndef RTOS_BUS_H
#define RTOS_BUS_H

#include <stdint.h>

enum bus_mode {
        BUS_POLLING_MODE,
        BUS_INTERRUPT_MODE
};

void set_bus_mode(enum bus_mode mode);
int i2c2_write_multi(uint8_t address, uint8_t reg, uint8_t *pdata,
                     int16_t size);
int i2c2_read_multi(uint8_t address, uint8_t reg, uint8_t *pdata,
                    int16_t size);
int spi1_txrx(uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
int spi1_tx(uint8_t *data, uint16_t size);

#endif /* RTOS_BUS_H */