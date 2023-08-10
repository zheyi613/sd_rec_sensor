/**
 * @file rtos_bus.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief manage blocking mode and interrupt mode when RTOS is running
 * @date 2023-08-10
 */
#include "rtos_bus.h"
#include "i2c.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

uint8_t bus_int_mode;

/**
 * @brief set bus mode
 * 
 * @param mode 0: polling (blocking) / 1: interrupt (non-blocking)
 */
void set_bus_mode(enum bus_mode mode)
{
        bus_int_mode = (uint8_t)mode;
}

int i2c2_write_multi(uint8_t address, uint8_t reg, uint8_t *data,
                     int16_t size)
{
        int status = 0;

        if (bus_int_mode) {
                status = HAL_I2C_Mem_Write_DMA(&hi2c2, address, reg,
                                               I2C_MEMADD_SIZE_8BIT,
                                               data, size);
                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        } else {
                status = HAL_I2C_Mem_Write(&hi2c2, address, reg,
                                           I2C_MEMADD_SIZE_8BIT,
                                           data, size, 1);
        }
        return status;
}

int i2c2_read_multi(uint8_t address, uint8_t reg, uint8_t *data,
                    int16_t size)
{
        int status = 0;

        if (bus_int_mode) {
                status = HAL_I2C_Mem_Read_DMA(&hi2c2, address, reg,
                                              I2C_MEMADD_SIZE_8BIT,
                                              data, size);
                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        } else {
                status = HAL_I2C_Mem_Read(&hi2c2, address, reg,
                                          I2C_MEMADD_SIZE_8BIT,
                                          data, size, 1);
        }
        return status;
}

int spi1_txrx(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
        int status = 0;

        if (bus_int_mode) {
                status = HAL_SPI_TransmitReceive_DMA(&hspi1, tx_data, rx_data,
                                                     size);
                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        } else {
                status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data,
                                                 size, 100);
        }
        return status;
}

int spi1_tx(uint8_t *data, uint16_t size)
{
        int status = 0;

        if (bus_int_mode) {
                status = HAL_SPI_Transmit_DMA(&hspi1, data, size);
                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        } else {
                status = HAL_SPI_Transmit(&hspi1, data, size, 100);
        }
        return status;
}
