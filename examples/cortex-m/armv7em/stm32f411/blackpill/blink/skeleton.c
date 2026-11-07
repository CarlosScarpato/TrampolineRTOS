#include "tp.h"
#include "tpl_os.h"

/*
 * Esqueleto de tempo real ICM - Camada 2 (5 tasks periodicas + heartbeat).
 * Cada task pulsa seu test point com WCET simulado.
 *   TP3 PB14 Acquisition  10ms
 *   TP4 PB15 Processing    20ms
 *   TP5 PA8  Display       50ms
 *   TP6 PA15 Diagnostics  100ms
 *   TP7 PB3  Logging      1000ms
 * (TP1/TP2 ISR_CAN/CAN_RX entram com o CAN real)
 */

static void delay_us(uint32_t us)
{
  volatile uint32_t n = us * 14U;
  while (n--) { __asm volatile("nop"); }
}

static void tp_init(void)
{
  volatile uint32_t dummy;
  RCC->AHB1ENR |= (1U << 0) | (1U << 1);   /* GPIOA, GPIOB */
  dummy = RCC->AHB1ENR; (void)dummy;

  /* saidas: PB14, PB15, PB3 | PA8, PA15 */
  GPIOB->MODER &= ~((3U<<(14U*2U)) | (3U<<(15U*2U)) | (3U<<(3U*2U)));
  GPIOB->MODER |=  ((1U<<(14U*2U)) | (1U<<(15U*2U)) | (1U<<(3U*2U)));
  GPIOA->MODER &= ~((3U<<(8U*2U)) | (3U<<(15U*2U)));
  GPIOA->MODER |=  ((1U<<(8U*2U)) | (1U<<(15U*2U)));

  /* todos baixos */
  GPIOB->BSRRH = (1U<<14) | (1U<<15) | (1U<<3);
  GPIOA->BSRRH = (1U<<8) | (1U<<15);
}

#define APP_Task_Acquisition_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
  initBoard();
  tp_init();
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(Acquisition)                 /* TP3 PB14, WCET ~0.1ms */
{
  GPIOB->BSRRL = (1U << 14);
  delay_us(100U);
  GPIOB->BSRRH = (1U << 14);
  TerminateTask();
}
#define APP_Task_Acquisition_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Processing_START_SEC_CODE
#include "tpl_memmap.h"
TASK(Processing)                  /* TP4 PB15, WCET ~0.2ms */
{
  GPIOB->BSRRL = (1U << 15);
  delay_us(200U);
  GPIOB->BSRRH = (1U << 15);
  TerminateTask();
}
#define APP_Task_Processing_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Display_START_SEC_CODE
#include "tpl_memmap.h"
TASK(Display)                     /* TP5 PA8, WCET ~2ms */
{
  GPIOA->BSRRL = (1U << 8);
  delay_us(2000U);
  GPIOA->BSRRH = (1U << 8);
  TerminateTask();
}
#define APP_Task_Display_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Diagnostics_START_SEC_CODE
#include "tpl_memmap.h"
TASK(Diagnostics)                 /* TP6 PA15, WCET ~0.1ms */
{
  GPIOA->BSRRL = (1U << 15);
  delay_us(100U);
  GPIOA->BSRRH = (1U << 15);
  TerminateTask();
}
#define APP_Task_Diagnostics_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Logging_START_SEC_CODE
#include "tpl_memmap.h"
TASK(Logging)                     /* TP7 PB3, WCET ~1ms */
{
  GPIOB->BSRRL = (1U << 3);
  delay_us(1000U);
  GPIOB->BSRRH = (1U << 3);
  TerminateTask();
}
#define APP_Task_Logging_STOP_SEC_CODE
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