#include "tp.h"
#include "tpl_os.h"

/*
 * CAN RX por interrupcao (ISR cat 2 via OIL).
 * MCP2515 INT (PB1) -> EXTI1 -> ISR_CAN -> ActivateTask(Task_CAN_RX)
 * Task_CAN_RX le o frame via SPI. ISR pulsa TP1(PB12), task pulsa TP2(PB13).
 */

#define MCP_RESET      0xC0
#define MCP_READ       0x03
#define MCP_WRITE      0x02
#define MCP_BITMOD     0x05
#define REG_CNF3       0x28
#define REG_CNF2       0x29
#define REG_CNF1       0x2A
#define REG_CANINTE    0x2B
#define REG_CANINTF    0x2C
#define REG_CANCTRL    0x0F
#define REG_RXB0CTRL   0x60
#define REG_RXB0SIDH   0x61

volatile uint32_t isr_count = 0;
volatile uint32_t rx_count  = 0;
volatile uint16_t rx_id     = 0;
volatile uint8_t  rx_dlc    = 0;
volatile uint8_t  rx_data[8] = {0};

static void delay_us(uint32_t us)
{
  volatile uint32_t n = us * 14U;
  while (n--) { __asm volatile("nop"); }
}

/* ---- SPI1 ---- */
static void spi1_init(void)
{
  volatile uint32_t dummy;
  RCC->AHB1ENR |= (1U << 0);
  RCC->APB2ENR |= (1U << 12);
  dummy = RCC->AHB1ENR; (void)dummy;
  GPIOA->MODER &= ~((3U<<8)|(3U<<10)|(3U<<12)|(3U<<14));
  GPIOA->MODER |=  ((1U<<8)|(2U<<10)|(2U<<12)|(2U<<14));
  GPIOA->AFR[0] &= ~((0xFU<<20)|(0xFU<<24)|(0xFU<<28));
  GPIOA->AFR[0] |=  ((5U<<20)|(5U<<24)|(5U<<28));
  GPIOA->OSPEEDR |= (3U<<10)|(3U<<12)|(3U<<14);
  GPIOA->BSRRL = (1U << 4);
  SPI1->CR1 = 0;
  SPI1->CR1 |= (1U << 2) | (5U << 3) | (1U << 9) | (1U << 8);
  SPI1->CR1 |= (1U << 6);
}
static uint8_t spi_xfer(uint8_t d)
{
  while ((SPI1->SR & (1U<<1)) == 0U) {}
  *((volatile uint8_t*)&SPI1->DR) = d;
  while ((SPI1->SR & (1U<<0)) == 0U) {}
  return *((volatile uint8_t*)&SPI1->DR);
}
static void cs_low(void)  { GPIOA->BSRRH = (1U << 4); }
static void cs_high(void) { GPIOA->BSRRL = (1U << 4); }
static void mcp_reset(void){ cs_low(); (void)spi_xfer(MCP_RESET); cs_high(); delay_us(10000U); }
static uint8_t mcp_read(uint8_t a){ uint8_t v; cs_low(); (void)spi_xfer(MCP_READ);(void)spi_xfer(a); v=spi_xfer(0); cs_high(); return v; }
static void mcp_write(uint8_t a,uint8_t d){ cs_low(); (void)spi_xfer(MCP_WRITE);(void)spi_xfer(a);(void)spi_xfer(d); cs_high(); }
static void mcp_bitmod(uint8_t a,uint8_t m,uint8_t d){ cs_low(); (void)spi_xfer(MCP_BITMOD);(void)spi_xfer(a);(void)spi_xfer(m);(void)spi_xfer(d); cs_high(); }

static void mcp_config(void)
{
  mcp_reset();
  mcp_write(REG_CNF1, 0x00);
  mcp_write(REG_CNF2, 0x90);
  mcp_write(REG_CNF3, 0x02);
  mcp_write(REG_RXB0CTRL, 0x60);
  mcp_write(REG_CANINTE, 0x01);    /* RX0IE: gera INT ao receber */
  mcp_write(REG_CANINTF, 0x00);
  mcp_write(REG_CANCTRL, 0x00);    /* modo normal */
  delay_us(10000U);
}

/* TP1/TP2 como saida (PB12, PB13) */
static void tp_init(void)
{
  volatile uint32_t dummy;
  RCC->AHB1ENR |= (1U << 1);
  dummy = RCC->AHB1ENR; (void)dummy;
  GPIOB->MODER &= ~((3U<<(12U*2U))|(3U<<(13U*2U)));
  GPIOB->MODER |=  ((1U<<(12U*2U))|(1U<<(13U*2U)));
  GPIOB->BSRRH = (1U<<12)|(1U<<13);
}

static void read_rx0(void)
{
  uint8_t sidh, sidl, i, dlc;
  sidh = mcp_read(REG_RXB0SIDH);
  sidl = mcp_read(REG_RXB0SIDH + 1);
  rx_id = (uint16_t)((sidh << 3) | (sidl >> 5));
  dlc = mcp_read(REG_RXB0SIDH + 4) & 0x0F;
  rx_dlc = dlc;
  for (i=0U;i<8U;i++) rx_data[i] = mcp_read(REG_RXB0SIDH + 5 + i);
  mcp_bitmod(REG_CANINTF, 0x01, 0x00);  /* limpa RX0IF do MCP2515 */
  rx_count++;
}

#define APP_Task_Task_CAN_RX_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
  initBoard();
  spi1_init();
  tp_init();
  mcp_config();
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(Task_CAN_RX)
{
  GPIOB->BSRRL = (1U << 13);      /* TP2 up */
  read_rx0();
  GPIOB->BSRRH = (1U << 13);      /* TP2 down */
  TerminateTask();
}
#define APP_Task_Task_CAN_RX_STOP_SEC_CODE
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

#define APP_ISR_ISR_CAN_START_SEC_CODE
#include "tpl_memmap.h"
DeclareTask(Task_CAN_RX);

ISR(ISR_CAN)
{
  GPIOB->BSRRL = (1U << 12);      /* TP1 up */
  isr_count++;
  ActivateTask(Task_CAN_RX);      /* so ativa a task, nao toca SPI */
  GPIOB->BSRRH = (1U << 12);      /* TP1 down */
}
#define APP_ISR_ISR_CAN_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(void, OS_CODE) assert_failed(uint8_t* file, uint32_t line)
{
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"