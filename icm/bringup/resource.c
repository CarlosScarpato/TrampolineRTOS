#include "tp.h"
#include "tpl_os.h"

/*
 * Fase ICM_Resource - priority ceiling / protecao de dados compartilhados.
 * ICM_Data protegido por ICM_Resource (ceiling = prioridade da Acquisition).
 *   TP1 PB12 = marca "resource tomado" (alto enquanto alguem segura o cadeado)
 *   TP3 PB14 = Acquisition (escreve ICM_Data)   10ms
 *   TP5 PA8  = Display (le ICM_Data)             50ms
 * Demonstra: enquanto Display segura o resource, Acquisition (maior prio)
 * NAO preempta o trecho protegido -> sem inversao de prioridade.
 */

/* dados compartilhados do cluster */
typedef struct {
  uint16_t speed;
  uint16_t rpm;
  uint8_t  fuel;
  uint8_t  temp;
} ICM_Data_t;

volatile ICM_Data_t ICM_Data = {0,0,0,0};

static void delay_us(uint32_t us)
{
  volatile uint32_t n = us * 14U;
  while (n--) { __asm volatile("nop"); }
}

static void tp_init(void)
{
  volatile uint32_t dummy;
  RCC->AHB1ENR |= (1U << 0) | (1U << 1);
  dummy = RCC->AHB1ENR; (void)dummy;

  /* PB12 (TP1 resource-held), PB14 (TP3) */
  GPIOB->MODER &= ~((3U<<(12U*2U)) | (3U<<(14U*2U)));
  GPIOB->MODER |=  ((1U<<(12U*2U)) | (1U<<(14U*2U)));
  /* PA8 (TP5) */
  GPIOA->MODER &= ~(3U<<(8U*2U));
  GPIOA->MODER |=  (1U<<(8U*2U));

  GPIOB->BSRRH = (1U<<12) | (1U<<14);
  GPIOA->BSRRH = (1U<<8);
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

TASK(Acquisition)                 /* escreve ICM_Data sob resource */
{
  GPIOB->BSRRL = (1U << 14);              /* TP3 up */

  GetResource(ICM_Resource);
  GPIOB->BSRRL = (1U << 12);              /* TP1 up: resource tomado */
  ICM_Data.speed = 120U;                 /* escrita protegida */
  ICM_Data.rpm   = 3500U;
  delay_us(300U);                        /* secao critica curta */
  GPIOB->BSRRH = (1U << 12);              /* TP1 down */
  ReleaseResource(ICM_Resource);

  GPIOB->BSRRH = (1U << 14);              /* TP3 down */
  TerminateTask();
}
#define APP_Task_Acquisition_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Display_START_SEC_CODE
#include "tpl_memmap.h"
TASK(Display)                     /* le ICM_Data sob resource (secao longa) */
{
  volatile uint16_t s, r;
  GPIOA->BSRRL = (1U << 8);               /* TP5 up */

  GetResource(ICM_Resource);
  GPIOB->BSRRL = (1U << 12);              /* TP1 up: resource tomado */
  s = ICM_Data.speed;                    /* leitura protegida */
  r = ICM_Data.rpm;
  (void)s; (void)r;
  delay_us(3000U);                       /* secao critica LONGA (3ms) */
  GPIOB->BSRRH = (1U << 12);              /* TP1 down */
  ReleaseResource(ICM_Resource);

  GPIOA->BSRRH = (1U << 8);               /* TP5 down */
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