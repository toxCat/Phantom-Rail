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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd1602.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * LM51772 Sim -- LCD bring-up demo (second black pill).
 *
 * Purpose: prove the PCF8574 + 1602 render path before any FC / I2C-link
 * traffic exists. IN and OUT values are STUBBED here; the real IN voltage
 * will arrive over I2C from the first black pill (translator), and OUT is
 * the simulated LM51772 setpoint.
 *
 * Screen flow:
 *   1. splash "LM51772 Sim"
 *   2. clear
 *   3. row0: "IN:nS XX.XXV"     (n = detected cell count)
 *      row1: "OUT:XX.XXV"
 *
 * The LCD itself is driven by the register-level (CMSIS, no HAL) bit-bang
 * driver in Src/lcd1602.c. It owns its own GPIO pins, so the generated HAL
 * I2C init is deliberately left uncalled (see MX_I2C*_Init note below).
 */

/* Format millivolts as exactly "XX.XX" (5 chars + NUL). */
static void fmt_volts(char *buf, uint32_t mv)
{
    uint32_t whole = mv / 1000U;
    uint32_t cv    = (mv % 1000U) / 10U;   /* centivolts, 2 digits */
    if (whole > 99U) whole = 99U;
    buf[0] = (char)('0' + whole / 10U);
    buf[1] = (char)('0' + whole % 10U);
    buf[2] = '.';
    buf[3] = (char)('0' + cv / 10U);
    buf[4] = (char)('0' + cv % 10U);
    buf[5] = '\0';
}

/* Naive cell count: smallest S in 1..8 with mv/S <= 4.20 V/cell.
 * Real detection belongs on the first black pill (it owns the ADC). */
static uint8_t cell_count(uint32_t mv)
{
    for (uint8_t s = 1; s <= 8; s++)
        if (mv <= (uint32_t)s * 4200U) return s;
    return 8;
}

/* Build a padded 16-char row (+NUL) and push it at the given line. */
static void draw_in(uint8_t s, uint32_t mv)
{
    char v[6], line[17];
    int n = 0;
    fmt_volts(v, mv);
    line[n++] = 'I'; line[n++] = 'N'; line[n++] = ':';
    line[n++] = (char)('0' + s); line[n++] = 'S'; line[n++] = ' ';
    for (int i = 0; i < 5; i++) line[n++] = v[i];
    line[n++] = 'V';
    while (n < 16) line[n++] = ' ';
    line[16] = '\0';
    lcd_set_cursor(0, 0);
    lcd_print(line);
}

static void draw_out(uint32_t mv)
{
    char v[6], line[17];
    int n = 0;
    fmt_volts(v, mv);
    line[n++] = 'O'; line[n++] = 'U'; line[n++] = 'T'; line[n++] = ':';
    for (int i = 0; i < 5; i++) line[n++] = v[i];
    line[n++] = 'V';
    while (n < 16) line[n++] = ' ';
    line[16] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(line);
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
  /* MX_I2C1_Init() / MX_I2C2_Init() intentionally NOT called: the LCD uses the
   * register-level bit-bang driver (Src/lcd1602.c), which drives its own GPIO
   * pins, and the I2C1 inter-board slave link is not implemented yet. The
   * generated HAL I2C init remains available in i2c.c for when it is. */
  /* USER CODE BEGIN 2 */
  lcd_init();

  /* 1-2: splash, then clear */
  lcd_set_cursor(0, 0);
  lcd_print("LM51772 Sim");
  lcd_delay_ms(1500);
  lcd_clear();

  uint32_t t = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- STUB DATA: replace with I2C value from first black pill ---- */
    uint32_t in_mv  = 7400U + (t % 1000U);   /* fake ramp 7.40 -> 8.40 V */
    uint32_t out_mv = 12000U;                /* fake setpoint 12.00 V   */
    /* ---------------------------------------------------------------- */

    uint8_t s = cell_count(in_mv);
    draw_in(s, in_mv);
    draw_out(out_mv);

    lcd_delay_ms(250);
    t += 100U;
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
