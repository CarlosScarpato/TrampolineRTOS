#include "tp.h"
#include "tpl_os.h"

/*
 * Bring-up dos LEDs do painel ICM (ativos em alto):
 *   PA10 = seta esquerda | PB0 = seta direita
 *   PB4  = warning (amar) | PB5 = falha CAN (verm)
 * PC13 (BLUE) pisca como heartbeat. Apenas teste de HW; nao e o driver final.
 */

#define APP_Task_ledseq_START_SEC_CODE
#include "tpl_memmap.h"

static void panel_leds_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

  /* PA10 como saida push-pull */
  GPIOA->MODER &= ~(3U << (10U * 2U));
  GPIOA->MODER |=  (1U << (10U * 2U));

  /* PB0, PB4, PB5 como saida push-pull */
  GPIOB->MODER &= ~((3U << (0U*2U)) | (3U << (4U*2U)) | (3U << (5U*2U)));
  GPIOB->MODER |=  ((1U << (0U*2U)) | (1U << (4U*2U)) | (1U << (5U*2U)));

/* todos apagados (BSRRH = reset) */
  GPIOA->BSRRH = (1U << 10U);
  GPIOB->BSRRH = (1U << 0U) | (1U << 4U) | (1U << 5U);
}

FUNC(int, OS_APPL_CODE) main(void)
{
  initBoard();
  panel_leds_init();
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(ledseq)
{
  static uint8_t idx = 0U;

  /* apaga os quatro (BSRRH) */
  GPIOA->BSRRH = (1U << 10U);
  GPIOB->BSRRH = (1U << 0U) | (1U << 4U) | (1U << 5U);

  /* acende so o atual (BSRRL) */
  switch (idx) {
    case 0U: GPIOA->BSRRL = (1U << 10U); break; /* seta esquerda */
    case 1U: GPIOB->BSRRL = (1U << 0U);  break; /* seta direita  */
    case 2U: GPIOB->BSRRL = (1U << 4U);  break; /* warning       */
    default: GPIOB->BSRRL = (1U << 5U);  break; /* falha CAN     */
  }

  idx = (uint8_t)((idx + 1U) % 4U);
  TerminateTask();
}
#define APP_Task_ledseq_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_heartbeat_START_SEC_CODE
#include "tpl_memmap.h"
TASK(heartbeat)
{
  ledToggle(BLUE);   /* PC13 — RTOS vivo */
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