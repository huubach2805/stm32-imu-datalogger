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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float ax, ay, az;   /* acceleration, g        */
  float gx, gy, gz;   /* angular rate, deg/s    */
} IMU_Data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* MPU6050 (AD0 = GND -> 7-bit address 0x68) */
#define MPU6050_ADDR          (0x68 << 1)
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_I2C_TIMEOUT   100      /* ms */

#define ACC_LSB_PER_G         16384.0f /* +-2 g      */
#define GYRO_LSB_PER_DPS      131.0f   /* +-250 deg/s */

#define RAD_TO_DEG            57.29578f
#define FILTER_ALPHA          0.98f    /* complementary filter: weight of gyro */
#define LOOP_PERIOD_MS        20       /* 50 Hz output rate */
#define GYRO_CALIB_SAMPLES    200      /* keep the board still during start-up */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static IMU_Data imu;
static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

/* Angles in degrees
 *   roll  : rotation around X
 *   pitch : rotation around Y
 *   yaw   : rotation around Z (gyro only -> drifts over time)
 */
static float roll = 0, pitch = 0, yaw = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1, MPU6050_I2C_TIMEOUT);
}

static HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t who_am_i = 0;

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                       &who_am_i, 1, MPU6050_I2C_TIMEOUT) != HAL_OK)
    return HAL_ERROR;

  if (who_am_i != 0x68)
    return HAL_ERROR;

  if (MPU6050_WriteReg(MPU6050_PWR_MGMT_1,   0x00) != HAL_OK) return HAL_ERROR; /* wake up            */
  if (MPU6050_WriteReg(MPU6050_SMPLRT_DIV,   0x07) != HAL_OK) return HAL_ERROR; /* sample rate divider */
  if (MPU6050_WriteReg(MPU6050_CONFIG,       0x03) != HAL_OK) return HAL_ERROR; /* DLPF ~44 Hz         */
  if (MPU6050_WriteReg(MPU6050_GYRO_CONFIG,  0x00) != HAL_OK) return HAL_ERROR; /* +-250 deg/s         */
  if (MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00) != HAL_OK) return HAL_ERROR; /* +-2 g               */

  return HAL_OK;
}

/* Read accel + gyro in one 14-byte burst (accel, temperature, gyro),
 * so all six values come from the same sampling instant. */
static HAL_StatusTypeDef MPU6050_Read(IMU_Data *d)
{
  uint8_t buf[14];

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                       buf, sizeof(buf), MPU6050_I2C_TIMEOUT) != HAL_OK)
    return HAL_ERROR;

  int16_t ax = (int16_t)(buf[0]  << 8 | buf[1]);
  int16_t ay = (int16_t)(buf[2]  << 8 | buf[3]);
  int16_t az = (int16_t)(buf[4]  << 8 | buf[5]);
  /* buf[6..7] = temperature, not used */
  int16_t gx = (int16_t)(buf[8]  << 8 | buf[9]);
  int16_t gy = (int16_t)(buf[10] << 8 | buf[11]);
  int16_t gz = (int16_t)(buf[12] << 8 | buf[13]);

  d->ax = ax / ACC_LSB_PER_G;
  d->ay = ay / ACC_LSB_PER_G;
  d->az = az / ACC_LSB_PER_G;
  d->gx = gx / GYRO_LSB_PER_DPS - gyro_bias_x;
  d->gy = gy / GYRO_LSB_PER_DPS - gyro_bias_y;
  d->gz = gz / GYRO_LSB_PER_DPS - gyro_bias_z;

  return HAL_OK;
}

/* Average the gyro output while the board is still -> zero-rate offset */
static void MPU6050_CalibrateGyro(void)
{
  float sx = 0, sy = 0, sz = 0;
  int n = 0;

  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++)
  {
    if (MPU6050_Read(&imu) == HAL_OK)
    {
      sx += imu.gx; sy += imu.gy; sz += imu.gz;
      n++;
    }
    HAL_Delay(5);
  }

  if (n > 0)
  {
    gyro_bias_x = sx / n;
    gyro_bias_y = sy / n;
    gyro_bias_z = sz / n;
  }
}

static void Angles_FromAccel(const IMU_Data *d, float *roll_acc, float *pitch_acc)
{
  *roll_acc  = atan2f( d->ay, sqrtf(d->ax * d->ax + d->az * d->az)) * RAD_TO_DEG;
  *pitch_acc = atan2f(-d->ax, sqrtf(d->ay * d->ay + d->az * d->az)) * RAD_TO_DEG;
}

/* Complementary filter: gyro for fast changes, accelerometer to remove drift */
static void Filter_Update(const IMU_Data *d, float dt)
{
  float roll_acc, pitch_acc;
  Angles_FromAccel(d, &roll_acc, &pitch_acc);

  roll  = FILTER_ALPHA * (roll  + d->gx * dt) + (1.0f - FILTER_ALPHA) * roll_acc;
  pitch = FILTER_ALPHA * (pitch + d->gy * dt) + (1.0f - FILTER_ALPHA) * pitch_acc;
  yaw  += d->gz * dt;   /* no magnetometer -> yaw cannot be corrected */
}

static void UART_Send(const char *s, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)len, 100);
}

/* Output line (10 integer fields):
 *   tick_ms, ax, ay, az, roll, pitch, yaw, gx, gy, gz
 *   ax..az x1000 (g), roll..yaw x100 (deg), gx..gz x100 (deg/s)
 */
static void UART_SendSample(uint32_t tick, const IMU_Data *d)
{
  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                     (unsigned long)tick,
                     (long)(d->ax * 1000.0f), (long)(d->ay * 1000.0f), (long)(d->az * 1000.0f),
                     (long)(roll * 100.0f), (long)(pitch * 100.0f), (long)(yaw * 100.0f),
                     (long)(d->gx * 100.0f), (long)(d->gy * 100.0f), (long)(d->gz * 100.0f));

  if (len > 0 && len < (int)sizeof(buf))
    UART_Send(buf, len);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  if (MPU6050_Init() != HAL_OK)
  {
    /* Sensor not found: report it instead of sending zeros forever */
    const char msg[] = "ERR,MPU6050 not found\r\n";
    while (1)
    {
      UART_Send(msg, sizeof(msg) - 1);
      HAL_Delay(1000);
    }
  }

  MPU6050_CalibrateGyro();

  /* Start the filter from the accelerometer angles instead of 0 */
  if (MPU6050_Read(&imu) == HAL_OK)
    Angles_FromAccel(&imu, &roll, &pitch);

  uint32_t last_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Fixed-rate loop based on SysTick, so dt is known and stable */
    uint32_t now = HAL_GetTick();
    if (now - last_tick < LOOP_PERIOD_MS)
      continue;

    float dt = (now - last_tick) / 1000.0f;
    last_tick = now;

    if (MPU6050_Read(&imu) != HAL_OK)
    {
      const char msg[] = "ERR,I2C read failed\r\n";
      UART_Send(msg, sizeof(msg) - 1);
      continue;
    }

    Filter_Update(&imu, dt);
    UART_SendSample(now, &imu);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
