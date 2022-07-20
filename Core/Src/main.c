/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define deg90 128
#define Minute_Ones 0
#define Minute_Tens 1
#define Hour_Ones 2
#define Hour_Tens 3
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum step{ONE, TWO, THREE, FOUR};

typedef struct
{
  int seconds;
  int minutes;
  int hours;
  
} time;
typedef struct {
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_1;
  uint16_t GPIO_Pin_2;
  uint16_t GPIO_Pin_3;
  uint16_t GPIO_Pin_4;
  int move_flag;
  int steps_to_move;
  enum step curr_step;
} Motor_TypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
int flag = 0;
int StepCount;
time tm;
Motor_TypeDef Motors[4];
int milliseconds;
int stepFlag = 0;
GPIO_PinState lastPosition1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void initStep(int steps, int id) {
  Motors[id].steps_to_move = steps;
  Motors[id].move_flag = 1;
}

void step(void) {
  for (int i = 0; i < 4; i++) {
    if (Motors[i].move_flag == 1) {
       switch(Motors[i].curr_step) 
    {
      case ONE:
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_4, GPIO_PIN_RESET);
        Motors[i].curr_step = TWO;  
        break;
      
      case TWO:
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_4, GPIO_PIN_RESET);
        Motors[i].curr_step = THREE;
        break;
      
      case THREE:
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_4, GPIO_PIN_SET);
        Motors[i].curr_step = FOUR;
        break;
      
      case FOUR:
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motors[i].GPIOx, Motors[i].GPIO_Pin_4, GPIO_PIN_SET);
        Motors[i].curr_step = ONE;
        Motors[i].steps_to_move--;
        if(Motors[i].steps_to_move <= 0) {
          Motors[i].move_flag = 0;
        }
        break;
      }
    
    }
  }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  milliseconds = 0;
  tm.seconds = 0;
  tm.minutes = 0;
  tm.hours = 0;
  Motors[Minute_Ones].GPIOx = GPIOB;
  Motors[Minute_Ones].GPIO_Pin_1 = IN1_1_Pin;
  Motors[Minute_Ones].GPIO_Pin_2 = IN2_1_Pin;
  Motors[Minute_Ones].GPIO_Pin_3 = IN3_1_Pin;
  Motors[Minute_Ones].GPIO_Pin_4 = IN4_1_Pin;
  Motors[Minute_Ones].move_flag = 0;
  Motors[Minute_Ones].steps_to_move = 0;
  Motors[Minute_Ones].curr_step = ONE;
  
  Motors[Minute_Tens].GPIOx = GPIOB;
  Motors[Minute_Tens].GPIO_Pin_1 = IN1_2_Pin;
  Motors[Minute_Tens].GPIO_Pin_2 = IN2_2_Pin;
  Motors[Minute_Tens].GPIO_Pin_3 = IN3_2_Pin;
  Motors[Minute_Tens].GPIO_Pin_4 = IN4_2_Pin;
  Motors[Minute_Tens].move_flag = 0;
  Motors[Minute_Tens].steps_to_move = 0;
  Motors[Minute_Tens].curr_step = ONE;
  
  Motors[Hour_Ones].GPIOx = GPIOA;
  Motors[Hour_Ones].GPIO_Pin_1 = IN1_3_Pin;
  Motors[Hour_Ones].GPIO_Pin_2 = IN2_3_Pin;
  Motors[Hour_Ones].GPIO_Pin_3 = IN3_3_Pin;
  Motors[Hour_Ones].GPIO_Pin_4 = IN4_3_Pin;
  Motors[Hour_Ones].move_flag = 0;
  Motors[Hour_Ones].steps_to_move = 0;
  Motors[Hour_Ones].curr_step = ONE;
  
  Motors[Hour_Tens].GPIOx = GPIOA;
  Motors[Hour_Tens].GPIO_Pin_1 = IN1_4_Pin;
  Motors[Hour_Tens].GPIO_Pin_2 = IN2_4_Pin;
  Motors[Hour_Tens].GPIO_Pin_3 = IN3_4_Pin;
  Motors[Hour_Tens].GPIO_Pin_4 = IN4_4_Pin;
  Motors[Hour_Tens].move_flag = 0;    
  Motors[Hour_Tens].steps_to_move = 0;
  Motors[Hour_Tens].curr_step = ONE;
  
  
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  initStep(deg90/10 + 1, 0);
  initStep(deg90/10 + 1, 1);
  initStep(deg90/10 + 1, 2);
  initStep(deg90/10 + 1, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lastPosition1 = HAL_GPIO_ReadPin(GPIOC, MAG1_Pin);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (milliseconds >= 1000) {
      tm.seconds++;
      milliseconds = 0;
    }
    if (tm.seconds >= 5) {
      if (flag != 1) {
        initStep(deg90/10 + 1, 0);
        initStep(deg90/10 + 1, 1);
        initStep(deg90/10 + 1, 2);
        initStep(deg90/10 + 1, 3);
      }
      tm.seconds = 0;
    }
    step();
    HAL_Delay(1);
//    if(lastPosition1 != HAL_GPIO_ReadPin(GPIOC, MAG1_Pin)) {
//      initStep(deg90*3);
//      lastPosition1 = HAL_GPIO_ReadPin(GPIOC, MAG1_Pin);
//    }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|LD3_Pin|LD6_Pin|LD4_Pin
                          |LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_3_Pin|IN2_3_Pin|IN1_4_Pin|IN2_4_Pin
                          |IN3_3_Pin|IN4_3_Pin|IN3_4_Pin|IN4_4_Pin
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_1_Pin|IN2_1_Pin|IN3_1_Pin|IN4_1_Pin
                          |IN1_2_Pin|IN2_2_Pin|IN3_2_Pin|IN4_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin LD3_Pin LD6_Pin LD4_Pin
                           LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|LD3_Pin|LD6_Pin|LD4_Pin
                          |LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_3_Pin IN2_3_Pin IN1_4_Pin IN2_4_Pin
                           IN3_3_Pin IN4_3_Pin IN3_4_Pin IN4_4_Pin
                           PA12 */
  GPIO_InitStruct.Pin = IN1_3_Pin|IN2_3_Pin|IN1_4_Pin|IN2_4_Pin
                          |IN3_3_Pin|IN4_3_Pin|IN3_4_Pin|IN4_4_Pin
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MAG1_Pin */
  GPIO_InitStruct.Pin = MAG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MAG1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_1_Pin IN2_1_Pin IN3_1_Pin IN4_1_Pin
                           IN1_2_Pin IN2_2_Pin IN3_2_Pin IN4_2_Pin */
  GPIO_InitStruct.Pin = IN1_1_Pin|IN2_1_Pin|IN3_1_Pin|IN4_1_Pin
                          |IN1_2_Pin|IN2_2_Pin|IN3_2_Pin|IN4_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
