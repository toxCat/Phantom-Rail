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
#include "i2c1_slave.h"    /* inter-board I2C1 link (PB6/PB7) */
#include "phantom_link.h"  /* shared wire format (../Protocol) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Bench bring-up aid: when set, row1 shows the I2C1 link counters
 * "A<addr-hits> F<frames> x<flags>" instead of the stubbed OUT, and row0 shows
 * the received value as soon as ANY frame parses (ignoring the valid flag).
 * Left at 0 in normal operation (row1 = OUT substitute). */
#define LINK_DEBUG 0

/* Sag-warning text appended to the IN row, decided by the translator and sent
 * in the frame flags (PR_FLAG_LOW / PR_FLAG_CRIT). */
#define WARN_NONE 0   /* charged: no text            */
#define WARN_LOW  1   /* 3.2-3.6 V/cell: "LOW"       */
#define WARN_CRIT 2   /* < 3.2 V/cell: "(x_X)"       */

/* ACS709 current sensor on PA0 (VIOUT -> A0), Task 3. Bidirectional Hall
 * output = ACS_ZERO_MV at 0 A, rising ACS_SENS_MV_PER_A per amp. CALIBRATE to
 * your part + supply: measure VIOUT at 0 A -> ACS_ZERO_MV, take the mV/A slope
 * from the datasheet (or a known load). Power the sensor so VIOUT stays <=3V3. */
#define ACS_VREF_MV        3300U
#define ACS_ADC_MAX        4095U
#define ACS_ZERO_MV        2500U   /* VIOUT at 0 A (nominal -35BB @ 5V) */
#define ACS_SENS_MV_PER_A  28U     /* sensitivity, mV per amp          */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t g_cur_ma = 0;   /* last ACS709 current reading (mA), for SWD */
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
 * Purpose: render the source telemetry that the translator (first black pill)
 * pushes over the I2C1 link, and stand in for the LM51772 output. The IN row
 * now shows the REAL source: the translator reads the pack behind its /9
 * divider, detects the cell count, and writes a frame to us (I2C1 slave,
 * address PR_I2C_ADDR). OUT is still a stubbed LM51772 setpoint.
 *
 * Screen flow:
 *   1. splash "LM51772 Sim"
 *   2. clear
 *   3. row0: "IN:nS XX.XXV"     (n = cell count from the translator;
 *                                "--" until a valid frame arrives)
 *      row1: "OUT:XX.XXV"
 *
 * The LCD itself is driven by the register-level (CMSIS, no HAL) hardware-I2C2
 * driver in Src/lcd1602.c. It owns the I2C2 peripheral (PB10/PB3), so the
 * generated HAL I2C init is deliberately left uncalled (see MX_I2C*_Init note
 * below).
 */

/* -------- ACS709 current sense on PA0 (ADC1_IN0), Task 3 --------
 * This board simulates the LM51772's internal current sensor: it reads the
 * ACS709 VIOUT on A0 and reports amps to the translator over I2C1 (which cuts
 * the Power FET at the limit). Register-level, mirrors the translator's ADC. */
static void acs_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOA->MODER |=  (3u << (0 * 2));      /* PA0 -> analog */
    GPIOA->PUPDR &= ~(3u << (0 * 2));
    ADC1->SMPR2 |= ADC_SMPR2_SMP0;         /* channel 0: long sample time */
    ADC1->SQR1   = 0;
    ADC1->SQR3   = 0;                       /* 1st conversion = channel 0 */
    ADC1->CR2   |= ADC_CR2_ADON;
    lcd_delay_ms(1);                        /* tSTAB */
}

/* One conversion -> current in milliamps. *valid=0 on ADC timeout. */
static uint16_t acs_read_ma(uint8_t *valid)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = 2000U * (SystemCoreClock / 1000000U);   /* 2 ms budget */
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if ((DWT->CYCCNT - start) > ticks) { *valid = 0; return 0; }
    }
    uint16_t raw  = (uint16_t)ADC1->DR;
    uint32_t v_mv = (uint32_t)raw * ACS_VREF_MV / ACS_ADC_MAX;
    int32_t  dv   = (int32_t)v_mv - (int32_t)ACS_ZERO_MV;    /* signed offset */
    if (dv < 0) dv = 0;                                       /* unidirectional load */
    uint32_t ma   = (uint32_t)dv * 1000U / ACS_SENS_MV_PER_A;
    if (ma > 60000U) ma = 60000U;
    *valid = 1;
    return (uint16_t)ma;
}

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

/* Cell detection now lives on the translator (it owns the ADC); the sim just
 * displays the cell count it receives over I2C1. */

/* Build the 16-char IN row and push it. Layout: "IN:nS XX.XX" then a warning
 * field. Charged keeps the 'V' suffix; LOW appends " LOW"; CRIT drops the 'V'
 * to fit the 5-char "(x_X)" emoji exactly inside 16 columns.
 *   charged: "IN:6S 22.75V    "
 *   low:     "IN:6S 21.30V LOW"
 *   crit:    "IN:6S 18.00(x_X)"  */
static void draw_in(uint8_t s, uint32_t mv, uint8_t warn)
{
    char v[6], line[17];
    fmt_volts(v, mv);
    line[0] = 'I'; line[1] = 'N'; line[2] = ':';
    line[3] = (char)('0' + (s > 9U ? 9U : s)); line[4] = 'S'; line[5] = ' ';
    for (int i = 0; i < 5; i++) line[6 + i] = v[i];   /* "XX.XX" -> cols 6..10 */

    if (warn == WARN_CRIT) {
        line[11] = '('; line[12] = 'x'; line[13] = '_'; line[14] = 'X'; line[15] = ')';
    } else {
        line[11] = 'V';
        if (warn == WARN_LOW) { line[12] = ' '; line[13] = 'L'; line[14] = 'O'; line[15] = 'W'; }
        else                  { line[12] = ' '; line[13] = ' '; line[14] = ' '; line[15] = ' '; }
    }
    line[16] = '\0';
    lcd_set_cursor(0, 0);
    lcd_print(line);
}

/* Row0 when there's no live source yet (link idle / translator quiet):
 * keep the familiar layout but blank the numbers -> "IN:--S --.--V". */
static void draw_in_stale(void)
{
    lcd_set_cursor(0, 0);
    lcd_print("IN:--S --.--V   ");
}

#if LINK_DEBUG
/* Write v as decimal into d (max 5 digits); return the number of chars. */
static int put_u16(char *d, uint16_t v)
{
    char tmp[5];
    int  i = 0;
    if (v == 0) { d[0] = '0'; return 1; }
    while (v && i < 5) { tmp[i++] = (char)('0' + v % 10U); v /= 10U; }
    for (int j = 0; j < i; j++) d[j] = tmp[i - 1 - j];
    return i;
}
static char hex_nib(uint8_t nib) { return (char)(nib < 10 ? '0' + nib : 'A' + nib - 10); }

/* Row1 diagnostic: "A<hits> F<frames> x<flags>" (counters capped at 9999). */
static void draw_diag(uint32_t hits, uint32_t frames, uint8_t flags)
{
    char line[17];
    int  n = 0;
    line[n++] = 'A';
    n += put_u16(line + n, (uint16_t)(hits   > 9999U ? 9999U : hits));
    line[n++] = ' ';
    line[n++] = 'F';
    n += put_u16(line + n, (uint16_t)(frames > 9999U ? 9999U : frames));
    line[n++] = ' ';
    line[n++] = 'x';
    line[n++] = hex_nib((uint8_t)(flags >> 4));
    line[n++] = hex_nib((uint8_t)(flags & 0x0F));
    while (n < 16) line[n++] = ' ';
    line[16] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(line);
}
#endif /* LINK_DEBUG */

/* Row1 (OUT): substitute output voltage, then the measured current to the
 * right as "X.XXA" (Task 3). e.g. "OUT:12.00V 1.85A". */
__attribute__((unused)) static void draw_out(uint32_t mv, uint16_t cur_ma)
{
    char v[6], line[17];
    fmt_volts(v, mv);
    line[0] = 'O'; line[1] = 'U'; line[2] = 'T'; line[3] = ':';
    for (int i = 0; i < 5; i++) line[4 + i] = v[i];   /* "XX.XX" -> cols 4..8 */
    line[9] = 'V';

    uint32_t aw = cur_ma / 1000U;             /* whole amps */
    uint32_t ac = (cur_ma % 1000U) / 10U;     /* centi-amps (2 digits) */
    if (aw > 9U) { aw = 9U; ac = 99U; }       /* single-digit field caps at 9.99 */
    line[10] = ' ';
    line[11] = (char)('0' + aw);
    line[12] = '.';
    line[13] = (char)('0' + ac / 10U);
    line[14] = (char)('0' + ac % 10U);
    line[15] = 'A';
    line[16] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(line);
}

/* -------- On-board LED heartbeat (PC13, active low) --------
 * Headless bring-up aid: gives a boot/status signal that does NOT depend on
 * the I2C bus or the LCD, so we can tell "MCU not running" apart from "MCU
 * running but I2C silent". */
static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (13 * 2));
    GPIOC->MODER |=  (1u << (13 * 2));   /* general-purpose output */
    GPIOC->ODR   |=  (1u << 13);         /* LED off (cathode on PC13) */
}
static inline void led_toggle(void) { GPIOC->ODR ^= (1u << 13); }

/* DWT (the driver's us timer) isn't running until lcd_init(), so the boot
 * blink uses a crude cycle loop -- timing is approximate, only needs to be
 * visible to a human eye (~150 ms per half at 16 MHz HSI). */
static void crude_delay(uint32_t loops)
{
    for (volatile uint32_t i = 0; i < loops; i++) { __NOP(); }
}
static void led_flash(uint8_t n)
{
    while (n--) {
        GPIOC->ODR &= ~(1u << 13); crude_delay(600000U);   /* on  */
        GPIOC->ODR |=  (1u << 13); crude_delay(600000U);   /* off */
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
  /* MX_I2C1_Init() / MX_I2C2_Init() intentionally NOT called: the LCD driver
   * (Src/lcd1602.c) owns and configures I2C2 (PB10/PB3) at register level, and
   * the I2C1 inter-board slave link is not implemented yet. The generated HAL
   * I2C init remains available in i2c.c for when it is. */
  /* USER CODE BEGIN 2 */
  led_init();
  led_flash(3);            /* 3 flashes at boot => we ARE running from flash */

  /* Bring up I2C2 + the panel; auto-detects the PCF8574 address (0x27/0x3F).
   * Returns the address that ACKed, or 0 if the bus is silent. */
  uint8_t lcd_addr = lcd_init();

  /* Bring up the I2C1 slave link regardless of the LCD: the translator can
   * start pushing frames the moment we ACK our address. */
  i2c1_slave_init(PR_I2C_ADDR);
  acs_init();                    /* ACS709 current sense on PA0 (Task 3) */

  if (lcd_addr) {
    /* 1-2: splash, then clear */
    lcd_set_cursor(0, 0);
    lcd_print("LM51772 Sim");
    lcd_delay_ms(1500);
    lcd_clear();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Task 3: sample the ACS709 and publish it for the translator to READ back
     * (it makes the 2 A cutoff decision; we never gate our own FET). */
    uint8_t  cvalid = 0;
    uint16_t cur_ma = acs_read_ma(&cvalid);
    g_cur_ma = cur_ma;
    i2c1_slave_set_current(cur_ma, cvalid ? PR_CFLAG_VALID : 0U);

    /* Latest source telemetry from the translator over I2C1. */
    uint16_t in_mv;
    uint8_t  cells, flags;
    i2c1_slave_get(&in_mv, &cells, &flags);
    uint32_t age = i2c1_slave_age_ms();       /* 0xFFFFFFFF until a frame parses */

    /* Map the translator's sag flags to the warning appended on the IN row. */
    uint8_t warn = WARN_NONE;
    if (flags & PR_FLAG_CRIT)     warn = WARN_CRIT;
    else if (flags & PR_FLAG_LOW) warn = WARN_LOW;

#if LINK_DEBUG
    /* Bring-up view: row0 = received value the moment any frame parses (ignore
     * the valid flag), row1 = link counters. Read row1 to localize a failure:
     *   A0  F0   -> master never reached us (wiring / master / addressing)
     *   A>0 F0   -> addressed, but bytes/parse failed (framing)
     *   A>0 F>0  -> link works; if row0 reads 00.00V the ADC is the problem */
    if (lcd_addr) {
      if (age != 0xFFFFFFFFU) draw_in(cells, in_mv, warn);
      else                    draw_in_stale();
      uint32_t hits, frames;
      i2c1_slave_diag(&hits, &frames);
      draw_diag(hits, frames, flags);
    }
    int live = (age < 1500U);
#else
    /* Normal view: trust the reading only if flagged valid AND recent. Row0 =
     * IN + sag warning; row1 = OUT (substitute until FC control lands). */
    int live = (flags & PR_FLAG_VALID) && (age < 1500U);
    if (lcd_addr) {
      if (live) draw_in(cells, in_mv, warn);
      else      draw_in_stale();
      draw_out(12000U, cur_ma);   /* OUT: stub setpoint (12.00 V) + measured A */
    }
#endif

    /* Heartbeat encodes link state without needing the LCD:
     *   ~2 Hz  = live source frames arriving
     *   ~3 Hz  = running but no live source / link idle */
    led_toggle();
    lcd_delay_ms(live ? 250U : 150U);
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
