/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t ax, ay, az;
    uint32_t timestamp_ms;
} IMU_Data_t;

typedef struct {
    uint32_t event_id;
    uint32_t timestamp_ms;
    uint16_t peak_mg;
    uint16_t rms_mg;
} PotholeEvent_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR              (0x68U << 1)
#define MPU6050_REG_WHO_AM_I      0x75U
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define IMU_SAMPLE_PERIOD_MS      10U
#define DETECT_WINDOW_SIZE       20U
#define POTHOLE_PEAK_THRESHOLD_MG 550U  /* Example only: calibrate experimentally */
#define POTHOLE_RMS_THRESHOLD_MG  180U  /* Example only: calibrate experimentally */
#define POTHOLE_COOLDOWN_MS       500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

osThreadId MPUTaskHandle;
osThreadId UartTaskHandle;
osThreadId AlarmTaskHandle;
QueueHandle_t uartQueueHandle;
/* USER CODE BEGIN PV */
static QueueHandle_t imuQueueHandle = NULL;
static volatile uint32_t imuReadErrorCount = 0;
static volatile uint32_t imuQueueDropCount = 0;
static volatile uint32_t eventQueueDropCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartMPUTask(void const * argument);
void StartUartTask(void const * argument);
void StartAlarmTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t AbsI32(int32_t v)
{
    return (uint32_t)(v < 0 ? -v : v);
}

static HAL_StatusTypeDef MPU6050_InitSensor(void)
{
    uint8_t who = 0, value = 0;
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3, 100) != HAL_OK)
        return HAL_ERROR;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who, 1, 100) != HAL_OK || who != 0x68U)
        return HAL_ERROR;
    /* Wake sensor; default accel range +/-2g (16384 LSB/g). */
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &value, 1, 100) != HAL_OK)
        return HAL_ERROR;
    value = 0x00U;
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &value, 1, 100) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
	HAL_UART_Transmit(&huart1, (uint8_t *)"Before RTOS\r\n", sizeof("Before RTOS\r\n") - 1U, 100);

    /* FreeRTOS native queues: preserve the original uartQueueHandle name. */
    imuQueueHandle = xQueueCreate(32, sizeof(IMU_Data_t));
    uartQueueHandle = xQueueCreate(10, sizeof(PotholeEvent_t));
    if (imuQueueHandle == NULL || uartQueueHandle == NULL)
        Error_Handler();

    osThreadDef(MPUTask, StartMPUTask, osPriorityAboveNormal, 0, 256);
    MPUTaskHandle = osThreadCreate(osThread(MPUTask), NULL);

    osThreadDef(UartTask, StartUartTask, osPriorityNormal, 0, 256);
    UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

    osThreadDef(AlarmTask, StartAlarmTask, osPriorityNormal, 0, 256);
    AlarmTaskHandle = osThreadCreate(osThread(AlarmTask), NULL);
    if (MPUTaskHandle == NULL || UartTaskHandle == NULL || AlarmTaskHandle == NULL)
        Error_Handler();

    // ?????
    osKernelStart();

    while (1)
    {
    }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMPUTask */
/**
  * @brief  Function implementing the MPUTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMPUTask */
void StartMPUTask(void const * argument)
{
    uint8_t accel_data[6];
    IMU_Data_t sample;

    while (MPU6050_InitSensor() != HAL_OK) {
        imuReadErrorCount++;
        osDelay(1000);
    }
    osDelay(100);

    TickType_t lastWakeTime = xTaskGetTickCount();
    for (;;) {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                             I2C_MEMADD_SIZE_8BIT, accel_data, 6, 20) == HAL_OK) {
            sample.ax = (int16_t)((uint16_t)accel_data[0] << 8 | accel_data[1]);
            sample.ay = (int16_t)((uint16_t)accel_data[2] << 8 | accel_data[3]);
            sample.az = (int16_t)((uint16_t)accel_data[4] << 8 | accel_data[5]);
            sample.timestamp_ms = HAL_GetTick();
            if (xQueueSend(imuQueueHandle, &sample, 0) != pdPASS)
                imuQueueDropCount++;
        } else {
            imuReadErrorCount++;
        }
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
    }
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
    PotholeEvent_t event;
    char msg[128];
    for (;;) {
        if (xQueueReceive(uartQueueHandle, &event, portMAX_DELAY) == pdPASS) {
            /* Simple line protocol for ESP32: no invented GPS or speed fields. */
            int len = snprintf(msg, sizeof(msg),
                "POTHOLE,%lu,%lu,%u,%u\r\n",
                (unsigned long)event.event_id,
                (unsigned long)event.timestamp_ms,
                (unsigned int)event.peak_mg,
                (unsigned int)event.rms_mg);
            if (len > 0 && len < (int)sizeof(msg))
                (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, 200);
        }
    }
}

/* USER CODE BEGIN Header_StartAlarmTask */
/**
* @brief Function implementing the AlarmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlarmTask */
void StartAlarmTask(void const * argument)
{
    IMU_Data_t sample;
    PotholeEvent_t event;
    uint32_t deviations[DETECT_WINDOW_SIZE] = {0};
    uint64_t sumSquares = 0;
    uint32_t peak = 0;
    uint32_t lastEventMs = 0, eventId = 0;
    uint32_t count = 0, index = 0;
    int32_t baseline = 0;
    uint8_t baselineReady = 0, armed = 1;

    for (;;) {
        if (xQueueReceive(imuQueueHandle, &sample, portMAX_DELAY) != pdPASS)
            continue;

        /* Low-pass baseline tracks gravity + slow tilt; subtract for shock signal.
           For production: calibrate mounting orientation and test on real roads. */
        int32_t z_mg = ((int32_t)sample.az * 1000) / 16384;
        if (!baselineReady) {
            baseline = z_mg;
            baselineReady = 1;
        }
        baseline += (z_mg - baseline) / 32;
        uint32_t deviation = AbsI32(z_mg - baseline);

        /* Rolling RMS, using 64-bit accumulation to avoid overflow. */
        uint32_t old = deviations[index];
        deviations[index] = deviation;
        index = (index + 1U) % DETECT_WINDOW_SIZE;
        if (count < DETECT_WINDOW_SIZE) count++;
        /* Sum of squared deviations in (mg)^2. */
        sumSquares -= (uint64_t)old * old;
        sumSquares += (uint64_t)deviation * deviation;
        peak = 0;
        for (uint32_t i = 0; i < count; i++)
            if (deviations[i] > peak) peak = deviations[i];
        uint32_t rms = (uint32_t)sqrt((double)sumSquares / count);

        if (count == DETECT_WINDOW_SIZE && armed &&
            peak >= POTHOLE_PEAK_THRESHOLD_MG &&
            rms >= POTHOLE_RMS_THRESHOLD_MG &&
            (eventId == 0 || (uint32_t)(sample.timestamp_ms - lastEventMs) >= POTHOLE_COOLDOWN_MS)) {
            event.event_id = ++eventId;
            event.timestamp_ms = sample.timestamp_ms;
            event.peak_mg = (uint16_t)(peak > 65535U ? 65535U : peak);
            event.rms_mg = (uint16_t)(rms > 65535U ? 65535U : rms);
            if (xQueueSend(uartQueueHandle, &event, 0) != pdPASS)
                eventQueueDropCount++;
            lastEventMs = sample.timestamp_ms;
            armed = 0;
        }
        /* Re-arm only after the whole window settles. */
        if (peak < POTHOLE_PEAK_THRESHOLD_MG / 2U &&
            rms < POTHOLE_RMS_THRESHOLD_MG / 2U)
            armed = 1;
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
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
