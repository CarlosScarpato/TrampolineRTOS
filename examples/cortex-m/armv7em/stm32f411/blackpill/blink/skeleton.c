#include "tp.h"
#include "tpl_os.h"

static void delay_us(uint32_t us)
{
  volatile uint32_t n = us * 21U;   /* ~21 ciclos por us a 84MHz -O0 */
  while (n--) { __asm volatile("nop"); }
}

static void tp_init(void)
{
  volatile uint32_t dummy;

  /* liga clocks e ESPERA assentar (read-back) */
  RCC->AHB1ENR |= (1U << 0) | (1U << 1);   /* GPIOA, GPIOB */
  dummy = RCC->AHB1ENR; (void)dummy;

  /* PB14 (TP3) saida push-pull */
  GPIOB->MODER &= ~(3U << (14U*2U));
  GPIOB->MODER |=  (1U << (14U*2U));

  /* PA8 (TP5) saida push-pull */
  GPIOA->MODER &= ~(3U << (8U*2U));
  GPIOA->MODER |=  (1U << (8U*2U));

  GPIOB->BSRRH = (1U << 14);
  GPIOA->BSRRH = (1U << 8);
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

TASK(Acquisition)
{
  GPIOB->BSRRL = (1U << 14);
  delay_us(500U);
  GPIOB->BSRRH = (1U << 14);
  TerminateTask();
}
#define APP_Task_Acquisition_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Display_START_SEC_CODE
#include "tpl_memmap.h"
TASK(Display)
{
  GPIOA->BSRRL = (1U << 8);
  delay_us(2000U);
  GPIOA->BSRRH = (1U << 8);
  TerminateTask();
}
#define APP_Task_Display_STOP_SEC_CODE
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