#include "tp.h"
#include "tpl_os.h"

/*
 * Bring-up do ADC para o ADKeyboard (ICM-IN-01).
 * PA1 = ADC1_IN1. Le o valor cru e guarda em adc_raw (ler no GDB).
 * PC13 (BLUE) pisca como heartbeat. Apenas teste de HW.
 */

volatile uint16_t adc_raw = 0U;

static void adc_init(void)
{
  /* clock do GPIOA e do ADC1 */
  RCC->AHB1ENR |= (1U << 0);        /* GPIOAEN  */
  RCC->APB2ENR |= (1U << 8);        /* ADC1EN   */

  /* PA1 como analogico: MODER[3:2] = 11 */
  GPIOA->MODER |= (3U << (1U * 2U));

  /* prescaler do ADC: /4  -> 84 MHz / 4 = 21 MHz */
  ADC->CCR &= ~(3U << 16);
  ADC->CCR |=  (1U << 16);

  /* resolucao 12 bits (RES = 00) */
  ADC1->CR1 &= ~(3U << 24);

  /* conversao unica por disparo (CONT = 0) */
  ADC1->CR2 &= ~(1U << 1);

  /* sample time MAXIMO no canal 1 (SMP1 = 111 = 480 ciclos) */
  ADC1->SMPR2 &= ~(7U << 3);
  ADC1->SMPR2 |=  (7U << 3);

  /* 1 conversao na sequencia (L = 0000) */
  ADC1->SQR1 &= ~(0xFU << 20);

  /* 1o item da sequencia = canal 1 (IN1 / PA1) */
  ADC1->SQR3 &= ~(0x1FU << 0);
  ADC1->SQR3 |=  (1U << 0);

  /* liga o ADC e espera estabilizar */
  ADC1->CR2 |= (1U << 0);           /* ADON */
  { volatile uint32_t i; for (i = 0U; i < 10000U; i++) { } }
}

#define APP_Task_adcread_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
  initBoard();
  adc_init();
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(adcread)
{
  uint32_t guard = 100000U;

  ADC1->CR2 |= (1U << 30);                       /* SWSTART */
  while (((ADC1->SR & (1U << 1)) == 0U) && (guard != 0U)) { guard--; }
  adc_raw = (uint16_t)(ADC1->DR & 0x0FFFU);      /* ler DR limpa o EOC */

  TerminateTask();
}
#define APP_Task_adcread_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_heartbeat_START_SEC_CODE
#include "tpl_memmap.h"
TASK(heartbeat)
{
  ledToggle(BLUE);
  TerminateTask();
}
#define APP_Task_heartbeat_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(void, OS_CODE) assert_failed(uint8_t* file, uint32_t line)
{
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"