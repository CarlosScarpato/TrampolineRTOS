#include "tp.h"
#include "tpl_os.h"

/*
 * Etapa 2 CAN: configura MCP2515 (8MHz, 500kbps), modo normal, e recebe frames.
 * Le o buffer RXB0 por polling. Resultados no GDB:
 *   mcp_mode    = CANSTAT apos ir p/ normal (deve virar 0x00)
 *   rx_count    = frames recebidos
 *   rx_id       = ID do ultimo frame
 *   rx_data[8]  = dados do ultimo frame
 * SPI1: SCK=PA5 MISO=PA6 MOSI=PA7 CS=PA4. PC13=heartbeat.
 */

#define MCP_RESET      0xC0
#define MCP_READ       0x03
#define MCP_WRITE      0x02
#define MCP_RD_STATUS  0xA0
#define MCP_BITMOD     0x05

#define REG_CANSTAT    0x0E
#define REG_CANCTRL    0x0F
#define REG_CNF3       0x28
#define REG_CNF2       0x29
#define REG_CNF1       0x2A
#define REG_CANINTE    0x2B
#define REG_CANINTF    0x2C
#define REG_RXB0CTRL   0x60
#define REG_RXB0SIDH   0x61

volatile uint8_t  mcp_mode   = 0xFF;
volatile uint32_t rx_count   = 0;
volatile uint16_t rx_id      = 0;
volatile uint8_t  rx_dlc     = 0;
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
  GPIOA->MODER |=  ((1U<<8)|(2U<<10)|(2U<<12)|(2U<<14));  /* PA4 out, PA5/6/7 AF */
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

static void mcp_reset(void)
{
  cs_low(); (void)spi_xfer(MCP_RESET); cs_high();
  delay_us(10000U);
}
static uint8_t mcp_read(uint8_t a)
{
  uint8_t v;
  cs_low(); (void)spi_xfer(MCP_READ); (void)spi_xfer(a); v = spi_xfer(0); cs_high();
  return v;
}
static void mcp_write(uint8_t a, uint8_t d)
{
  cs_low(); (void)spi_xfer(MCP_WRITE); (void)spi_xfer(a); (void)spi_xfer(d); cs_high();
}
static void mcp_bitmod(uint8_t a, uint8_t mask, uint8_t d)
{
  cs_low(); (void)spi_xfer(MCP_BITMOD); (void)spi_xfer(a);
  (void)spi_xfer(mask); (void)spi_xfer(d); cs_high();
}

static void mcp_config(void)
{
  mcp_reset();

  /* bit timing 8MHz, 500kbps, SP ~87.5% */
  mcp_write(REG_CNF1, 0x00);
  mcp_write(REG_CNF2, 0x90);
  mcp_write(REG_CNF3, 0x02);

  /* RXB0: aceita tudo (rollover off, no filter) */
  mcp_write(REG_RXB0CTRL, 0x60);   /* RXM=11 recebe qualquer msg */

  /* sem interrupcoes por enquanto (polling) */
  mcp_write(REG_CANINTE, 0x00);
  mcp_write(REG_CANINTF, 0x00);

  /* modo normal: CANCTRL REQOP=000 */
  mcp_write(REG_CANCTRL, 0x00);
  delay_us(10000U);
  mcp_mode = mcp_read(REG_CANSTAT);   /* deve virar 0x00 (OPMOD=000 normal) */
}

/* le RXB0 se houver frame; retorna 1 se leu */
static uint8_t mcp_poll_rx(void)
{
  uint8_t intf = mcp_read(REG_CANINTF);
  if (intf & 0x01) {                  /* RX0IF */
    uint8_t sidh, sidl, i, dlc;
    sidh = mcp_read(REG_RXB0SIDH);
    sidl = mcp_read(REG_RXB0SIDH + 1);
    rx_id = (uint16_t)((sidh << 3) | (sidl >> 5));
    dlc = mcp_read(REG_RXB0SIDH + 4) & 0x0F;
    rx_dlc = dlc;
    for (i = 0U; i < 8U; i++) {
      rx_data[i] = mcp_read(REG_RXB0SIDH + 5 + i);
    }
    mcp_bitmod(REG_CANINTF, 0x01, 0x00);  /* limpa RX0IF */
    rx_count++;
    return 1U;
  }
  return 0U;
}

#define APP_Task_cantest_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
  initBoard();
  spi1_init();
  mcp_config();
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(cantest)
{
  (void)mcp_poll_rx();
  TerminateTask();
}
#define APP_Task_cantest_STOP_SEC_CODE
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