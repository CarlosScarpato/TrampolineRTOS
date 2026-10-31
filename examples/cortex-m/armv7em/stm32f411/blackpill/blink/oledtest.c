#include "tp.h"
#include "tpl_os.h"

/* Etapa 3: SH1106 128x64, I2C1 0x3C. Escreve texto com fonte 5x7. */

#define OLED_ADDR   0x3C
#define OLED_CMD    0x00
#define OLED_DATA   0x40

volatile uint8_t oled_ok = 0xFF;

/* ---- I2C1 ---- */
static void i2c1_init(void)
{
  RCC->AHB1ENR |= (1U << 1);
  RCC->APB1ENR |= (1U << 21);
  GPIOB->MODER &= ~((3U << 12) | (3U << 14));
  GPIOB->MODER |=  ((2U << 12) | (2U << 14));
  GPIOB->OTYPER |= (1U << 6) | (1U << 7);
  GPIOB->OSPEEDR |= (3U << 12) | (3U << 14);
  GPIOB->PUPDR &= ~((3U << 12) | (3U << 14));
  GPIOB->PUPDR |=  ((1U << 12) | (1U << 14));
  GPIOB->AFR[0] &= ~((0xFU << 24) | (0xFU << 28));
  GPIOB->AFR[0] |=  ((4U  << 24) | (4U  << 28));
  I2C1->CR1 |= (1U << 15); I2C1->CR1 &= ~(1U << 15);
  I2C1->CR2 = 42U; I2C1->CCR = 210U; I2C1->TRISE = 43U;
  I2C1->CR1 |= (1U << 0);
}

static uint8_t i2c_write(uint8_t ctrl, const uint8_t *buf, uint16_t n)
{
  uint32_t g; uint16_t i;
  I2C1->CR1 |= (1U << 8);
  g = 100000U; while (((I2C1->SR1 & 1U)==0U) && g) g--; if (!g) return 0U;
  I2C1->DR = (uint8_t)(OLED_ADDR << 1);
  g = 100000U;
  while (((I2C1->SR1 & (1U<<1))==0U) && g) {
    if (I2C1->SR1 & (1U<<10)) { I2C1->SR1 &= ~(1U<<10); I2C1->CR1|=(1U<<9); return 0U; }
    g--;
  }
  if (!g) return 0U;
  (void)I2C1->SR2;
  g=100000U; while (((I2C1->SR1 & (1U<<7))==0U) && g) g--;
  I2C1->DR = ctrl;
  for (i=0U;i<n;i++){ g=100000U; while(((I2C1->SR1&(1U<<7))==0U)&&g)g--; if(!g)return 0U; I2C1->DR=buf[i]; }
  g=100000U; while (((I2C1->SR1 & (1U<<2))==0U) && g) g--;
  I2C1->CR1 |= (1U << 9);
  return 1U;
}

static void oled_cmd(uint8_t c){ uint8_t b=c; (void)i2c_write(OLED_CMD,&b,1U); }

static void sh1106_init(void)
{
  oled_cmd(0xAE); oled_cmd(0xD5); oled_cmd(0x80);
  oled_cmd(0xA8); oled_cmd(0x3F); oled_cmd(0xD3); oled_cmd(0x00);
  oled_cmd(0x40); oled_cmd(0xAD); oled_cmd(0x8B);
  oled_cmd(0xA1); oled_cmd(0xC8); oled_cmd(0xDA); oled_cmd(0x12);
  oled_cmd(0x81); oled_cmd(0xFF); oled_cmd(0xD9); oled_cmd(0x1F);
  oled_cmd(0xDB); oled_cmd(0x40); oled_cmd(0xA6); oled_cmd(0xAF);
}

static void sh1106_fill(uint8_t pat)
{
  uint8_t page,i; uint8_t line[128];
  for (i=0U;i<128U;i++) line[i]=pat;
  for (page=0U;page<8U;page++){ oled_cmd(0xB0+page); oled_cmd(0x02); oled_cmd(0x10); (void)i2c_write(OLED_DATA,line,128U); }
}

/* ---- fonte 5x7 (subset: digitos, letras usadas, espaco, : - . % ) ---- */
/* cada char = 5 colunas, cada coluna = 1 byte (bit0=topo) */

static const uint8_t FONT[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, /* ' ' idx 0 */
  {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
  {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
  {0x42,0x61,0x51,0x49,0x46}, /* 2 */
  {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
  {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
  {0x27,0x45,0x45,0x45,0x39}, /* 5 */
  {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
  {0x01,0x71,0x09,0x05,0x03}, /* 7 */
  {0x36,0x49,0x49,0x49,0x36}, /* 8 */
  {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
  {0x7E,0x11,0x11,0x11,0x7E}, /* A idx 11 */
  {0x7F,0x49,0x49,0x49,0x36}, /* B */
  {0x3E,0x41,0x41,0x41,0x22}, /* C */
  {0x7F,0x41,0x41,0x22,0x1C}, /* D */
  {0x7F,0x49,0x49,0x49,0x41}, /* E */
  {0x7F,0x09,0x09,0x09,0x01}, /* F */
  {0x3E,0x41,0x49,0x49,0x7A}, /* G */
  {0x7F,0x08,0x08,0x08,0x7F}, /* H */
  {0x00,0x41,0x7F,0x41,0x00}, /* I */
  {0x20,0x40,0x41,0x3F,0x01}, /* J */
  {0x7F,0x08,0x14,0x22,0x41}, /* K */
  {0x7F,0x40,0x40,0x40,0x40}, /* L */
  {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
  {0x7F,0x04,0x08,0x10,0x7F}, /* N */
  {0x3E,0x41,0x41,0x41,0x3E}, /* O */
  {0x7F,0x09,0x09,0x09,0x06}, /* P */
  {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
  {0x7F,0x09,0x19,0x29,0x46}, /* R */
  {0x46,0x49,0x49,0x49,0x31}, /* S */
  {0x01,0x01,0x7F,0x01,0x01}, /* T */
  {0x3F,0x40,0x40,0x40,0x3F}, /* U */
  {0x1F,0x20,0x40,0x20,0x1F}, /* V */
  {0x7F,0x20,0x18,0x20,0x7F}, /* W */
  {0x63,0x14,0x08,0x14,0x63}, /* X */
  {0x07,0x08,0x70,0x08,0x07}, /* Y */
  {0x61,0x51,0x49,0x45,0x43}, /* Z idx 36 */
  {0x00,0x36,0x36,0x00,0x00}, /* : idx 37 */
  {0x08,0x08,0x08,0x08,0x08}, /* - idx 38 */
  {0x00,0x60,0x60,0x00,0x00}, /* . idx 39 */
  {0x62,0x64,0x08,0x13,0x23}, /* % idx 40 */
};

/* mapeia um char ASCII para o indice na FONT */
static uint8_t font_idx(char ch)
{
  if (ch == ' ') return 0U;
  if (ch >= '0' && ch <= '9') return (uint8_t)(1U + (ch - '0'));
  if (ch >= 'A' && ch <= 'Z') return (uint8_t)(11U + (ch - 'A'));
  if (ch == ':') return 37U;
  if (ch == '-') return 38U;
  if (ch == '.') return 39U;
  if (ch == '%') return 40U;
  return 0U; /* desconhecido -> espaco */
}

/* escreve string na page (0-7) a partir da coluna col */
static void oled_text(uint8_t page, uint8_t col, const char *s)
{
  oled_cmd(0xB0 + page);
  oled_cmd(0x02 + (col & 0x0F));
  oled_cmd(0x10 + (col >> 4));
  while (*s) {
    uint8_t gi = font_idx(*s);
    uint8_t glyph[6];
    uint8_t k;
    for (k=0U;k<5U;k++) glyph[k] = FONT[gi][k];
    glyph[5] = 0x00;             /* espaco entre chars */
    (void)i2c_write(OLED_DATA, glyph, 6U);
    s++;
  }
}

/* ---- Trampoline ---- */
#define APP_Task_oledprobe_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
  initBoard();
  i2c1_init();
  sh1106_init();
  sh1106_fill(0x00);                 /* limpa */
  oled_text(0U, 0U,  "ICM TRAMPOLINE");
  oled_text(2U, 0U,  "VEL: 120 KM-H");
  oled_text(4U, 0U,  "RPM: 3500");
  oled_text(6U, 0U,  "TEMP: 90 FUEL 80%");
  oled_ok = 1U;
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(oledprobe){ TerminateTask(); }
#define APP_Task_oledprobe_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_heartbeat_START_SEC_CODE
#include "tpl_memmap.h"
TASK(heartbeat){ ledToggle(BLUE); TerminateTask(); }
#define APP_Task_heartbeat_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(void, OS_CODE) assert_failed(uint8_t* file, uint32_t line){}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"