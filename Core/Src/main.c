/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dwt_delay.h"
#include "rtos_bus.h"
#include "icm20948.h"
#include "diskio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SD_BUFFER_SIZE  512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t sensor_handler;
TaskHandle_t sd_handler;
TaskHandle_t msg_handler;

struct imu {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} imu;

char *sd_idle_buf_ptr;
char sd_buffer[2][SD_BUFFER_SIZE];
/* variable of sd spi */
WORD Timer1, Timer2;
/* variable of file system */
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;

uint8_t rec_finish;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void sensor_task(void *param);
static void sd_task(void *param);
static void msg_task(void *param);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  BaseType_t status;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  HAL_Delay(100);
  /* Initialize SD double buffer */
  uint32_t free;

  sd_idle_buf_ptr = sd_buffer[0];
  memset(sd_buffer[0], 0, SD_BUFFER_SIZE);
  memset(sd_buffer[1], 0, SD_BUFFER_SIZE);

  rec_finish = 0;

  set_bus_mode(BUS_POLLING_MODE);

  if (icm20948_init(1125, GYRO_2000_DPS, ACCEL_16G, LP_BW_119HZ)) {
    while (1) {
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
      HAL_Delay(1000);
    }
  }

  fres = f_mount(&fs, "", 0);
  if (fres != FR_OK) {
    Error_Handler();
  }
  fres = f_open(&fil, "test1.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
  if (fres != FR_OK) {
    Error_Handler();
  }
  fres = f_getfree("", &fre_clust, &pfs);
  if (fres != FR_OK) {
    Error_Handler();
  }
  free = (uint32_t)(fre_clust * pfs->csize * 0.5);
  if (free < 1) {
    Error_Handler();
  }

  set_bus_mode(BUS_INTERRUPT_MODE);

  vSetVarulMaxPRIGROUPValue();
  SEGGER_UART_init(1500000);
  SEGGER_SYSVIEW_Conf();

  status = xTaskCreate(sensor_task, "sensor_task", 300, NULL, 3, &sensor_handler);
  configASSERT(status == pdPASS);

  status = xTaskCreate(sd_task, "sd_task", 300, NULL, 2, &sd_handler);
  configASSERT(status == pdPASS);

  status = xTaskCreate(msg_task, "msg_task", 500, NULL, 1, &msg_handler);
  configASSERT(status == pdPASS);

  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hi2c->Instance == hi2c2.Instance) {
    vTaskNotifyGiveFromISR(sensor_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hi2c->Instance == hi2c2.Instance) {
    vTaskNotifyGiveFromISR(sensor_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi->Instance == hspi1.Instance) {
    vTaskNotifyGiveFromISR(sd_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (hspi->Instance == hspi1.Instance) {
    vTaskNotifyGiveFromISR(sd_handler, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void sensor_task(void *param)
{
  struct {
    BaseType_t current_tick;
    float ax, ay, az, gx, gy, gz;
  } data;
  uint16_t msg_len;

  while (1) {
    if (!icm20948_read_axis6(&data.ax, &data.ay, &data.az,
                             &data.gx, &data.gy, &data.gz)) {
      data.current_tick = xTaskGetTickCount();
      imu.ax = data.az;
      imu.ay = data.ay;
      imu.az = data.az;
      imu.gx = data.gx;
      imu.gy = data.gy;
      imu.gz = data.gz;

      if (!rec_finish) {
        msg_len = snprintf(sd_idle_buf_ptr, 200, "tick: %ld, "
                          "ax: %.2f, ay: %.2f, az: %.2f, "
                          "gx: %.2f, gy: %.2f, gz: %.2f\n",
                          data.current_tick, data.ax, data.ay, data.az,
                          data.gx, data.gy, data.gz);
        sd_idle_buf_ptr += msg_len;
        // memcpy(sd_idle_buf_ptr, &data, sizeof(data));
        // sd_idle_buf_ptr += sizeof(data);
        // *(sd_idle_buf_ptr++) = '\n';
      }
    }  
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void sd_task(void *param)
{
  BaseType_t start_tick = xTaskGetTickCount();
  BaseType_t pass_tick;
  uint8_t wr_buf_id = 1;
  UINT wr_msg_size = 0;
  UINT remain_size;

  while (1) {
    pass_tick = xTaskGetTickCount() - start_tick;
    if (pass_tick < pdMS_TO_TICKS(10000)) {
      if (wr_msg_size > 0)
        f_write(&fil, sd_buffer[wr_buf_id], wr_msg_size, &remain_size);

      vTaskSuspendAll();
      wr_msg_size = sd_idle_buf_ptr - sd_buffer[!wr_buf_id];
      sd_idle_buf_ptr = sd_buffer[wr_buf_id];
      xTaskResumeAll();

      wr_buf_id = !wr_buf_id;
    } else if (rec_finish == 0) {
      f_close(&fil);
      f_mount(NULL, "", 0);
      rec_finish = 1;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void msg_task(void *param)
{
  char msg[300];
  uint16_t len;
  BaseType_t sensor_wm, sd_wm, msg_wm;
  BaseType_t min_remaining, remaining;
  float ax, ay, az, gx, gy, gz;

	while (1) {
    vTaskSuspendAll();
    ax = imu.ax;
    ay = imu.ay;
    az = imu.az;
    gx = imu.gx;
    gy = imu.gy;
    gz = imu.gz;
    xTaskResumeAll();

    sensor_wm = uxTaskGetStackHighWaterMark(sensor_handler);
    sd_wm = uxTaskGetStackHighWaterMark(sd_handler);
    msg_wm = uxTaskGetStackHighWaterMark(msg_handler);

    min_remaining = xPortGetMinimumEverFreeHeapSize();
    remaining = xPortGetFreeHeapSize();

		len = snprintf(msg, 300, "sensor data:\n"
                             "ax: %.2f, ay: %.2f, az: %.2f\n"
                             "gx: %.2f, gy: %.2f, gz: %.2f\n"
                             "stack watermark:\n"
                             "sensor: %ld, sd: %ld, msg: %ld\n"
                             "min remain: %ld, current remain: %ld\n",
                             ax, ay, az, gx, gy, gz,
                             sensor_wm, sd_wm, msg_wm,
                             min_remaining, remaining);
    CDC_Transmit_FS((uint8_t *)msg, len);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == htim9.Instance) {
    uint16_t count;

    count = Timer1;
    if (count)
      Timer1 = --count;
    count = Timer2;
    if (count)
      Timer2 = --count;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    CDC_Transmit_FS((uint8_t *)"error!\n", 7);
    HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
