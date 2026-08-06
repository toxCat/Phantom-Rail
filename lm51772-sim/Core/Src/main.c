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
#include "i2c1_slave.h"    /* LM51772 register slave on I2C1 (PB6/PB7) */
#include "lm51772_regs.h"  /* shared register map (../Protocol) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Sag-warning text appended to the IN row, from the battery flags the host
 * writes into the extension registers (PR_BATT_LOW / PR_BATT_CRIT). */
#define WARN_NONE 0   /* charged: no text            */
#define WARN_LOW  1   /* 3.2-3.6 V/cell: "LOW"       */
#define WARN_CRIT 2   /* < 3.2 V/cell: "(x_X)"       */

/* ACS709 current sensor on PA0 (VIOUT -> A0), Task 3. Bidirectional Hall
 * output = ACS_ZERO_MV at 0 A, rising ACS_SENS_MV_PER_A per amp. CALIBRATE to
 * your part + supply: measure VIOUT at 0 A -> ACS_ZERO_MV, take the mV/A slope
 * from the datasheet (or a known load). Power the sensor so VIOUT stays <=3V3. */
#define ACS_VREF_MV        3300U
#define ACS_ADC_MAX        4095U
#define ACS_ZERO_MV        2494U   /* measured VIOUT at 0 A on this rig (~VCC/2) */
#define ACS_SENS_MV_PER_A  28U     /* -35BB @ ~5V; 1 ADC count ~= 29 mA of I */
#define ACS_OVERSAMPLE     8U      /* conversions averaged per sample (noise)  */
#define CUR_SAMPLE_MS      500U    /* re-sample the current at 2 Hz, hold between */
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
 * LM51772 Sim (second black pill).
 *
 * Presents the real LM51772 I2C register interface (I2C1 slave @ LM_ADDR, see
 * ../Protocol/lm51772_regs.h) to the translator, which acts as the host
 * controller. The host writes VOUT_TARGET / ILIM_THRESHOLD / CONV_EN2 and reads
 * status back, exactly as a flight controller would drive the real IC. This
 * board also reads the ACS709 (PA0) as the IC's output current and models the
 * status bits (CC_OPERATION / IOUT_OC vs the programmed ILIM).
 *
 * Screen (16x2):
 *   row0: "IN:nS XX.XXV [warn]"   battery view (from extension regs 0xE0-0xE3)
 *   row1: "XX.XXV X.XXA STS"      commanded VOUT, measured current, status
 *                                 (ON / CC / OC / OFF)
 *
 * The LCD is driven by the register-level hardware-I2C2 driver in Src/lcd1602.c
 * (PB10/PB3), separate from the I2C1 host link, so the generated HAL I2C init
 * is deliberately left uncalled.
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

/* One conversion with a CLEAN start: clear ADC_SR first so we always wait for
 * THIS conversion's EOC, never a stale flag left set (which would make the wait
 * fall straight through and read garbage/0 -- the source of the 0/value flicker).
 * *ok = 0 on timeout. */
static uint16_t acs_convert(uint8_t *ok)
{
    ADC1->SR = 0;                          /* clear EOC/OVR/STRT before starting */
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = 2000U * (SystemCoreClock / 1000000U);   /* 2 ms budget */
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if ((DWT->CYCCNT - start) > ticks) { *ok = 0; return 0; }
    }
    *ok = 1;
    return (uint16_t)ADC1->DR;             /* reading DR clears EOC */
}

/* Averaged current read -> milliamps. *valid = 0 only if every conversion in
 * the burst timed out (caller then holds its last good reading). */
static uint16_t acs_read_ma(uint8_t *valid)
{
    uint32_t sum = 0, good = 0;
    for (uint32_t i = 0; i < ACS_OVERSAMPLE; i++) {
        uint8_t ok;
        uint16_t r = acs_convert(&ok);
        if (ok) { sum += r; good++; }
    }
    if (good == 0) { *valid = 0; return 0; }

    uint32_t raw  = sum / good;
    uint32_t v_mv = raw * ACS_VREF_MV / ACS_ADC_MAX;
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

/* Row1 (OUTPUT side of the LM51772 model): commanded output voltage from
 * VOUT_TARGET, the measured current, and a status field:
 *   "12.00V 1.85A ON "   enabled & regulating
 *   "12.00V 2.05A CC "   in current-limit (ACS709 >= ILIM)
 *   "12.00V 2.05A OC "   over-current fault latched
 *   "12.00V 0.00A OFF"   converter disabled (CONV_EN2 = 0)                    */
static void draw_out(uint32_t mv, uint16_t cur_ma, const char *status)
{
    char v[6], line[17];
    fmt_volts(v, mv);
    for (int i = 0; i < 5; i++) line[i] = v[i];       /* "XX.XX" -> cols 0..4 */
    line[5] = 'V';
    line[6] = ' ';

    uint32_t aw = cur_ma / 1000U;             /* whole amps */
    uint32_t ac = (cur_ma % 1000U) / 10U;     /* centi-amps (2 digits) */
    if (aw > 9U) { aw = 9U; ac = 99U; }       /* single-digit field caps at 9.99 */
    line[7]  = (char)('0' + aw);
    line[8]  = '.';
    line[9]  = (char)('0' + ac / 10U);
    line[10] = (char)('0' + ac % 10U);
    line[11] = 'A';
    line[12] = ' ';
    line[13] = status[0]; line[14] = status[1]; line[15] = status[2];
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

  /* Bring up the LM51772 register slave (I2C1 @ LM_ADDR) regardless of the LCD:
   * the host controller can read/write our registers the moment we ACK. */
  i2c1_slave_init(LM_ADDR);
  acs_init();                    /* ACS709 current sense on PA0 (Iout) */

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
    /* 1. Measure output current (ACS709) at CUR_SAMPLE_MS (2 Hz) with last-good
     *    hold, so the reading is steady. This is the LM51772's internal Iout. */
    static uint32_t cur_stamp  = 0;
    static uint16_t cur_ma     = 0;
    static uint8_t  cur_primed = 0;
    if (!cur_primed ||
        (DWT->CYCCNT - cur_stamp) / (SystemCoreClock / 1000U) >= CUR_SAMPLE_MS) {
        uint8_t ok;
        uint16_t m = acs_read_ma(&ok);
        if (ok) { cur_ma = m; cur_primed = 1; }
        cur_stamp = DWT->CYCCNT;
    }
    g_cur_ma = cur_ma;

    /* 2. Decode the host-written control registers. */
    uint8_t  ctrl   = lm_reg_get(LM_REG_PD_CONTROL0);
    uint8_t  enable = (ctrl & LM_CONV_EN2) ? 1U : 0U;
    uint8_t  div20  = (lm_reg_get(LM_REG_MFR_D8) & LM_SEL_FB_DIV20) ? 1U : 0U;
    uint16_t vcode  = lm_vout_code(lm_reg_get(LM_REG_VOUT_LSB), lm_reg_get(LM_REG_VOUT_MSB));
    uint16_t vout_mv = lm_vout_to_mv(vcode, div20);
    uint16_t ilim_ma = lm_ilim_to_ma(lm_reg_get(LM_REG_ILIM));

    /* 3. Model status. CC when enabled and the measured current is at/above the
     *    programmed ILIM; IOUT_OC latches until CLEAR_FAULTS. VIN_UV mirrors the
     *    host's battery-critical flag (extension register). */
    static uint8_t oc_latched = 0;
    if (lm_take_clear_faults()) oc_latched = 0;
    uint8_t cc = (enable && cur_ma >= ilim_ma) ? 1U : 0U;
    if (cc) oc_latched = 1;

    uint8_t batt = lm_reg_get(PR_EXT_BATT);
    uint8_t status = 0;
    if (!enable)              status |= LM_ST_OFF;
    if (oc_latched)           status |= LM_ST_IOUT_OC;
    if (batt & PR_BATT_CRIT)  status |= LM_ST_VIN_UV;
    lm_reg_set(LM_REG_STATUS_BYTE, status);
    lm_reg_set(LM_REG_PD_STATUS0, cc ? LM_CC_OPERATION : 0U);

    /* 4. Display. Row0 = battery IN (from the host's extension registers) with
     *    the sag warning; row1 = LM51772 output view. */
    if (lcd_addr) {
      if (batt & PR_BATT_VALID) {
        uint16_t vin_mv = (uint16_t)(lm_reg_get(PR_EXT_VIN_LSB) |
                                     ((uint16_t)lm_reg_get(PR_EXT_VIN_MSB) << 8));
        uint8_t  cells  = lm_reg_get(PR_EXT_CELLS);
        uint8_t  warn   = (batt & PR_BATT_CRIT) ? WARN_CRIT
                        : (batt & PR_BATT_LOW)  ? WARN_LOW : WARN_NONE;
        draw_in(cells, vin_mv, warn);
      } else {
        draw_in_stale();
      }

      const char *st = !enable ? "OFF" : oc_latched ? "OC " : cc ? "CC " : "ON ";
      draw_out(vout_mv, cur_ma, st);
    }

    led_toggle();
    lcd_delay_ms(200U);
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
