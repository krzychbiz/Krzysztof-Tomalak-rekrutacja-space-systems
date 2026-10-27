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
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE BEGIN PV */
/* --- Konfiguracja protokołu WS2812B (1 dioda na PA6 / TIM3_CH1) --- */
#define WS2812_LED_COUNT   1      /* ile diod w łańcuchu */
#define WS2812_BITS        24     /* 8 bitów G + 8 R + 8 B na diodę */
#define WS2812_RESET_SLOTS 80     /* zera na końcu = reset (>50 µs stanu niskiego) */

/* Wartości wypełnienia PWM dla bitu 0 i 1.
 * TIM3 przy 84 MHz, Period=104 → 1 bit ≈ 1.25 µs (800 kHz). */
#define WS2812_0           34     /* krótki impuls wysoki ≈ 0.40 µs */
#define WS2812_1           67     /* długi impuls wysoki ≈ 0.80 µs */

/* Bufor: 24 bity koloru + sloty resetu; DMA ładuje je kolejno do CCR1 */
#define WS2812_BUFFER_SIZE \
    (WS2812_LED_COUNT * WS2812_BITS + WS2812_RESET_SLOTS)

static uint16_t ws2812_buffer[WS2812_BUFFER_SIZE];

/* 1 = trwa wysyłanie przez DMA; volatile, bo zmienia go też przerwanie */
static volatile uint8_t ws2812_busy = 0;

/* --- Prostokąt jasności czerwonej: okres 3 s, wypełnienie 50% --- */
#define RED_PERIOD_MS      3000U  
#define RED_DUTY_PERCENT   50U    
#define RED_BRIGHTNESS     255U   

/* --- Piła jasności zielonej: okres 3 s, liniowy wzrost 0→255 --- */
#define GREEN_PERIOD_MS    3000U  /*  */
#define GREEN_BRIGHTNESS   255U   /* maks jasność */

/* --- Niebieska: 3 mrugnięcia- */
#define BTN_GPIO_PORT      GPIOC
#define BTN_GPIO_PIN       GPIO_PIN_13  /* Nucleo User Button, active-low */
#define BLUE_BLINK_COUNT   3U
#define BLUE_ON_MS         200U
#define BLUE_OFF_MS        200U
#define BLUE_BRIGHTNESS    255U

/* Aktualne natężenia RGB — volatile, żeby Live Expressions w debuggerze je widział */
volatile uint8_t led_r = 0;
volatile uint8_t led_g = 0;
volatile uint8_t led_b = 0;

static uint32_t wave_t0 = 0;          /* wspólny czas startu */
static uint8_t  red_prev = 0xFF;      /* poprzednie R */
static uint8_t  green_prev = 0xFF;   
static uint8_t  blue_prev = 0xFF;     

static uint8_t  btn_prev = 1U;        /* 1 = zwolniony (PC13 high) */

static uint8_t  blue_blinks_left = 0; /* ile mrugnięć zostało */
static uint8_t  blue_on = 0;          /* aktualny stan B podczas mrugania */
static uint32_t blue_phase_t0 = 0;    /* start bieżącej fazy on/off */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Buduje ramkę PWM  i uruchamia DMA na TIM3_CH1 (PA6) */
void WS2812_SetColor(uint8_t red,
                     uint8_t green,
                     uint8_t blue)
{

    while (ws2812_busy)
    {
    }

    /* WS2812B GRB (nie RGB) */
    uint8_t colors[3] = {green, red, blue};
    uint32_t index = 0;

    /* Każdy bajt → 8 impulsów PWM (najpierw najstarszy bit) */
    for (uint8_t c = 0; c < 3; c++)
    {
        for (int8_t bit = 7; bit >= 0; bit--)
        {
            if (colors[c] & (1U << bit))
            {
                ws2812_buffer[index++] = WS2812_1; /* bit = 1 */
            }
            else
            {
                ws2812_buffer[index++] = WS2812_0; /* bit = 0 */
            }
        }
    }

    /* Dopisz zera = stan niski na czas resetu, dioda zapamiętuje kolor */
    while (index < WS2812_BUFFER_SIZE)
    {
        ws2812_buffer[index++] = 0;
    }

    ws2812_busy = 1;

    /* DMA przepisuje bufor do rejestru CCR1 timera (kolejne wypełnienia PWM) */
    if (HAL_TIM_PWM_Start_DMA(
            &htim3,
            TIM_CHANNEL_1,
            (uint32_t *)ws2812_buffer,
            WS2812_BUFFER_SIZE) != HAL_OK)
    {
        ws2812_busy = 0;
        Error_Handler();
    }
}

/* Wywołanie z przerwania: DMA skończyło całą ramkę PWM */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1); /* wyłącz DMA i PWM */
        ws2812_busy = 0; /* wolno wysłać następną ramkę */
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  wave_t0 = HAL_GetTick();
  red_prev = 0xFF;   
  green_prev = 0xFF;
  blue_prev = 0xFF;
  btn_prev = (HAL_GPIO_ReadPin(BTN_GPIO_PORT, BTN_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE BEGIN 3 */
	  /* R: prostokąt; G: piła; B: 3 mrugnięcia po B1*/
	  {
	      uint32_t tick = HAL_GetTick();
	      uint32_t now = tick - wave_t0;

	      /* --- Przycisk: zbocze wciśnięcia (active-low) --- */
	      uint8_t btn_now =
	          (HAL_GPIO_ReadPin(BTN_GPIO_PORT, BTN_GPIO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
	      if (btn_now != btn_prev)
	      {
	          if (btn_prev == 1U && btn_now == 0U && blue_blinks_left == 0U)
	          {
	              /* Start sekwencji: 3× */
	              blue_blinks_left = BLUE_BLINK_COUNT;
	              blue_on = 1U;
	              blue_phase_t0 = tick;
	          }
	          btn_prev = btn_now;
	      }

	      /* --- Maszyna stanów mrugania B (nieblokująca) --- */
	      uint8_t b = 0U;
	      if (blue_blinks_left > 0U)
	      {
	          uint32_t phase_ms = blue_on ? BLUE_ON_MS : BLUE_OFF_MS;
	          if ((tick - blue_phase_t0) >= phase_ms)
	          {
	              blue_phase_t0 = tick;
	              if (blue_on)
	              {
	                  blue_on = 0U; 
	              }
	              else
	              {
	                  blue_blinks_left--;
	                  if (blue_blinks_left > 0U)
	                  {
	                      blue_on = 1U; /* kolejne mrugnięcie */
	                  }
	              }
	          }
	          b = blue_on ? (uint8_t)BLUE_BRIGHTNESS : 0U;
	      }

	      /* --- R: prostokąt 50%, --- */
	      uint32_t red_phase = now % RED_PERIOD_MS;
	      uint32_t high_ms = (RED_PERIOD_MS * RED_DUTY_PERCENT) / 100U;
	      uint8_t r = (red_phase < high_ms) ? (uint8_t)RED_BRIGHTNESS : 0U;

	      /* --- G: piła --- */
	      uint32_t green_phase = now % GREEN_PERIOD_MS;
	      uint8_t g = (uint8_t)((green_phase * GREEN_BRIGHTNESS) / GREEN_PERIOD_MS);

	      led_r = r;
	      led_g = g;
	      led_b = b;

	      if (r != red_prev || g != green_prev || b != blue_prev)
	      {
	          red_prev = r;
	          green_prev = g;
	          blue_prev = b;
	          WS2812_SetColor(r, g, b);
	      }
	  }
	  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 104;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 34;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* Nucleo User Button B1 → PC13, active-low, pull-up */
    GPIO_InitStruct.Pin = BTN_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN_GPIO_PORT, &GPIO_InitStruct);
  }
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
