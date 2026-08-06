#include "stm32f4xx.h"
#include "usart1.h"

/* PA9 = USART1_TX, PA10 = USART1_RX, both AF7. */
#define TX_PIN 9
#define RX_PIN 10

void usart1_init(uint32_t baud)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;      /* (also on for the ADC/FET) */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER &= ~((3u << (TX_PIN * 2)) | (3u << (RX_PIN * 2)));
    GPIOA->MODER |=  ((2u << (TX_PIN * 2)) | (2u << (RX_PIN * 2)));   /* AF mode */
    GPIOA->PUPDR &= ~(3u << (RX_PIN * 2));
    GPIOA->PUPDR |=  (1u << (RX_PIN * 2));     /* pull-up on RX (idle high) */

    /* AF7 = USART1 on PA9/PA10 (both in AFR[1], nibbles 1 and 2). */
    GPIOA->AFR[1] &= ~((0xFu << ((TX_PIN - 8) * 4)) | (0xFu << ((RX_PIN - 8) * 4)));
    GPIOA->AFR[1] |=  ((7u   << ((TX_PIN - 8) * 4)) | (7u   << ((RX_PIN - 8) * 4)));

    /* Baud from PCLK2 (APB2 prescaler PPRE2 = RCC->CFGR[15:13]), oversample 16. */
    uint32_t pclk2 = SystemCoreClock;
    uint32_t ppre2 = (RCC->CFGR >> 13) & 0x7u;
    if (ppre2 >= 4u) pclk2 >>= (ppre2 - 3u);
    USART1->BRR = (pclk2 + baud / 2u) / baud;

    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;   /* 8N1 */
}

void usart1_write(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        while (!(USART1->SR & USART_SR_TXE)) { }
        USART1->DR = data[i];
    }
    while (!(USART1->SR & USART_SR_TC)) { }    /* let the last byte flush out */
}

int usart1_read_byte(uint8_t *out, uint32_t timeout_us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = timeout_us * (SystemCoreClock / 1000000U);
    for (;;) {
        uint32_t sr = USART1->SR;
        if (sr & USART_SR_RXNE) { *out = (uint8_t)USART1->DR; return 1; }
        if (sr & (USART_SR_ORE | USART_SR_FE)) { (void)USART1->DR; }  /* clear+skip */
        if ((DWT->CYCCNT - start) > ticks) return 0;
    }
}
