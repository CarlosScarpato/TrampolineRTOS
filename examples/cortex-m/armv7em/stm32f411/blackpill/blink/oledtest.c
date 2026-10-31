#include "tp.h"
#include "tpl_os.h"

/*
 * Etapa 2 do bring-up da tela: inicializa o SH1106 (128x64, I2C1 0x3C)
 * e preenche a tela toda (todos os pixels acesos). PC13 = heartbeat.
 */

#define OLED_ADDR   0x3C
#define OLED_CMD    0x00    /* control byte: proximo e comando   */
#define OLED_DATA   0x40    /* control byte: proximos sao dados  */

volatile uint8_t oled_ok = 0xFF;   /* 1 = init enviado; ler no GDB */

/* ---- I2C1 low-level ---- */

static void i2c1_init(void)
{
  RCC->AHB1ENR |= (1U << 1);          /* GPIOBEN */
  RCC->APB1ENR |= (1U << 21);         /* I2C1EN  */

  GPIOB->MODER &= ~((3U << (6U*2U)) | (3U << (7U*2U)));
  GPIOB->MODER |=  ((2U << (6U*2U)) | (2U << (7U*2U)));   /* AF */
  GPIOB->OTYPER |= (1U << 6) | (1U << 7);                 /* open-drain */
  GPIOB->OSPEEDR |= (3U << (6U*2U)) | (3U << (7U*2U));
  GPIOB->PUPDR &= ~((3U << (6U*2U)) | (3U << (7U*2U)));
  GPIOB->PUPDR |=  ((1U << (6U*2U)) | (1U << (7U*2U)));   /* pull-up */
  GPIOB->AFR[0] &= ~((0xFU << (6U*4U)) | (0xFU << (7U*4U)));
  GPIOB->AFR[0] |=  ((4U  << (6U*4U)) | (4U  << (7U*4U))); /* AF4 = I2C1 */

  I2C1->CR1 |= (1U << 15);  I2C1->CR1 &= ~(1U << 15);     /* SWRST */
  I2C1->CR2   = 42U;
  I2C1->CCR   = 210U;
  I2C1->TRISE = 43U;
  I2C1->CR1  |= (1U << 0);                                /* PE */
}

/* envia 1 byte de controle + N bytes. retorna 0 em timeout */
static uint8_t i2c_write(uint8_t ctrl, const uint8_t *buf, uint16_t n)
{
  uint32_t guard;
  uint16_t i;

  I2C1->CR1 |= (1U << 8);                                 /* START */
  guard = 100000U; while (((I2C1->SR1 & (1U<<0))==0U) && guard) guard--;
  if (!guard) return 0U;

  I2C1->DR = (uint8_t)(OLED_ADDR << 1);                   /* addr+W */
  guard = 100000U;
  while (((I2C1->SR1 & (1U<<1))==0U) && guard) {
    if (I2C1->SR1 & (1U<<10)) { I2C1->SR1 &= ~(1U<<10); I2C1->CR1|=(1U<<9); return 0U; }
    guard--;
  }
  if (!guard) return 0U;
  (void)I2C1->SR2;                                        /* limpa ADDR */

  /* control byte */
  guard = 100000U; while (((I2C1->SR1 & (1U<<7))==0U) && guard) guard--;
  I2C1->DR = ctrl;

  /* payload */
  for (i = 0U; i < n; i++) {
    guard = 100000U; while (((I2C1->SR1 & (1U<<7))==0U) && guard) guard--;
    if (!guard) return 0U;
    I2C1->DR = buf[i];
  }

  guard = 100000U; while (((I2C1->SR1 & (1U<<2))==0U) && guard) guard--; /* BTF */
  I2C1->CR1 |= (1U << 9);                                 /* STOP */
  return 1U;
}

static void oled_cmd(uint8_t c)
{
  uint8_t b = c;
  (void)i2c_write(OLED_CMD, &b, 1U);
}

/* ---- SH1106 ---- */

static void sh1106_init(void)
{
  oled_cmd(0xAE);             /* display off */
  oled_cmd(0xD5); oled_cmd(0x80);
  oled_cmd(0xA8); oled_cmd(0x3F);   /* multiplex 64 */
  oled_cmd(0xD3); oled_cmd(0x00);
  oled_cmd(0x40);                   /* start line 0 */
  oled_cmd(0xAD); oled_cmd(0x8B);   /* charge pump (SH1106) */
  oled_cmd(0xA1);                   /* segment remap */
  oled_cmd(0xC8);                   /* COM scan dir */
  oled_cmd(0xDA); oled_cmd(0x12);
  oled_cmd(0x81); oled_cmd(0xFF);   /* contraste max */
  oled_cmd(0xD9); oled_cmd(0x1F);
  oled_cmd(0xDB); oled_cmd(0x40);
  oled_cmd(0xA6);                   /* normal (nao invertido) */
  oled_cmd(0xAF);                   /* display ON */
}

/* preenche a tela toda com 'pattern' (0xFF = tudo aceso, 0x00 = apagado) */
static void sh1106_fill(uint8_t pattern)
{
  uint8_t page;
  uint8_t line[128];
  uint8_t i;
  for (i = 0U; i < 128U; i++) line[i] = pattern;

  for (page = 0U; page < 8U; page++) {
    oled_cmd(0xB0 + page);          /* page address */
    oled_cmd(0x02);                 /* low col (SH1106 tem offset de 2!) */
    oled_cmd(0x10);                 /* high col */
    (void)i2c_write(OLED_DATA, line, 128U);
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
  sh1106_fill(0xFF);                 /* tela toda acesa */
  oled_ok = 1U;
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

TASK(oledprobe)
{
  TerminateTask();
}
#define APP_Task_oledprobe_STOP_SEC_CODE
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