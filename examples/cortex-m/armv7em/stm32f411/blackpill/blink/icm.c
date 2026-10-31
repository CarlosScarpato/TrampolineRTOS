#include "tp.h"
#include "tpl_os.h"

/*
 * ICM - Integracao Camada 1: CAN -> decodifica -> tela OLED.
 * ISR_CAN (PB1/EXTI1) -> ActivateTask(Task_CAN_RX) -> decodifica em ICM_Data.
 * Task_Display (50ms) -> mostra ICM_Data na tela SH1106 (I2C1 0x3C).
 * ICM_Data protegido por ICM_Resource.
 */

/* ======== dados compartilhados ======== */
typedef struct {
  uint16_t rpm;        /* M1 EngineSpeed */
  uint16_t speed_x100; /* M2 VehicleSpeed * 100 (0.01 km/h) */
  uint8_t  fuel_x2;    /* M3 FuelLevel * 2 (0.5%) */
  int16_t  temp;       /* M3 CoolantTemp (offset -40) */
  uint8_t  warn;       /* M4 WarningFlags */
} ICM_Data_t;

volatile ICM_Data_t ICM_Data = {0,0,0,0,0};
volatile uint32_t rx_count = 0;

static void delay_us(uint32_t us){ volatile uint32_t n=us*14U; while(n--){__asm volatile("nop");} }

/* ======== SPI1 / MCP2515 ======== */
#define MCP_READ 0x03
#define MCP_WRITE 0x02
#define MCP_RESET 0xC0
#define MCP_BITMOD 0x05
#define REG_CNF1 0x2A
#define REG_CNF2 0x29
#define REG_CNF3 0x28
#define REG_CANINTE 0x2B
#define REG_CANINTF 0x2C
#define REG_CANCTRL 0x0F
#define REG_RXB0CTRL 0x60
#define REG_RXB0SIDH 0x61

static void spi1_init(void)
{
  volatile uint32_t d; RCC->AHB1ENR|=(1U<<0); RCC->APB2ENR|=(1U<<12); d=RCC->AHB1ENR;(void)d;
  GPIOA->MODER &= ~((3U<<8)|(3U<<10)|(3U<<12)|(3U<<14));
  GPIOA->MODER |=  ((1U<<8)|(2U<<10)|(2U<<12)|(2U<<14));
  GPIOA->AFR[0] &= ~((0xFU<<20)|(0xFU<<24)|(0xFU<<28));
  GPIOA->AFR[0] |=  ((5U<<20)|(5U<<24)|(5U<<28));
  GPIOA->OSPEEDR |= (3U<<10)|(3U<<12)|(3U<<14);
  GPIOA->BSRRL=(1U<<4);
  SPI1->CR1=0; SPI1->CR1|=(1U<<2)|(5U<<3)|(1U<<9)|(1U<<8); SPI1->CR1|=(1U<<6);
}
static uint8_t spi_xfer(uint8_t x){ while((SPI1->SR&(1U<<1))==0U){} *((volatile uint8_t*)&SPI1->DR)=x; while((SPI1->SR&(1U<<0))==0U){} return *((volatile uint8_t*)&SPI1->DR); }
static void csl(void){GPIOA->BSRRH=(1U<<4);} static void csh(void){GPIOA->BSRRL=(1U<<4);}
static uint8_t mcp_read(uint8_t a){uint8_t v;csl();(void)spi_xfer(MCP_READ);(void)spi_xfer(a);v=spi_xfer(0);csh();return v;}
static void mcp_write(uint8_t a,uint8_t v){csl();(void)spi_xfer(MCP_WRITE);(void)spi_xfer(a);(void)spi_xfer(v);csh();}
static void mcp_bitmod(uint8_t a,uint8_t m,uint8_t v){csl();(void)spi_xfer(MCP_BITMOD);(void)spi_xfer(a);(void)spi_xfer(m);(void)spi_xfer(v);csh();}
static void mcp_init(void)
{
  csl();(void)spi_xfer(MCP_RESET);csh();delay_us(10000U);
  mcp_write(REG_CNF1,0x00); mcp_write(REG_CNF2,0x90); mcp_write(REG_CNF3,0x02);
  mcp_write(REG_RXB0CTRL,0x60); mcp_write(REG_CANINTE,0x01); mcp_write(REG_CANINTF,0x00);
  mcp_write(REG_CANCTRL,0x00); delay_us(10000U);
}

/* ======== I2C1 / SH1106 ======== */
#define OLED_ADDR 0x3C
static void i2c1_init(void)
{
  volatile uint32_t d; RCC->AHB1ENR|=(1U<<1); RCC->APB1ENR|=(1U<<21); d=RCC->AHB1ENR;(void)d;
  GPIOB->MODER &= ~((3U<<12)|(3U<<14)); GPIOB->MODER |= ((2U<<12)|(2U<<14));
  GPIOB->OTYPER |= (1U<<6)|(1U<<7); GPIOB->OSPEEDR |= (3U<<12)|(3U<<14);
  GPIOB->PUPDR &= ~((3U<<12)|(3U<<14)); GPIOB->PUPDR |= ((1U<<12)|(1U<<14));
  GPIOB->AFR[0] &= ~((0xFU<<24)|(0xFU<<28)); GPIOB->AFR[0] |= ((4U<<24)|(4U<<28));
  I2C1->CR1|=(1U<<15); I2C1->CR1&=~(1U<<15);
  I2C1->CR2=42U; I2C1->CCR=210U; I2C1->TRISE=43U; I2C1->CR1|=(1U<<0);
}
static uint8_t i2c_wr(uint8_t ctrl,const uint8_t*b,uint16_t n)
{
  uint32_t g; uint16_t i;
  I2C1->CR1|=(1U<<8); g=100000U; while(((I2C1->SR1&1U)==0U)&&g)g--; if(!g)return 0U;
  I2C1->DR=(uint8_t)(OLED_ADDR<<1); g=100000U;
  while(((I2C1->SR1&(1U<<1))==0U)&&g){ if(I2C1->SR1&(1U<<10)){I2C1->SR1&=~(1U<<10);I2C1->CR1|=(1U<<9);return 0U;} g--; }
  if(!g)return 0U; (void)I2C1->SR2;
  g=100000U;while(((I2C1->SR1&(1U<<7))==0U)&&g)g--; I2C1->DR=ctrl;
  for(i=0U;i<n;i++){ g=100000U;while(((I2C1->SR1&(1U<<7))==0U)&&g)g--; if(!g)return 0U; I2C1->DR=b[i]; }
  g=100000U;while(((I2C1->SR1&(1U<<2))==0U)&&g)g--; I2C1->CR1|=(1U<<9); return 1U;
}
static void ocmd(uint8_t c){uint8_t b=c;(void)i2c_wr(0x00,&b,1U);}
static void sh1106_init(void)
{
  ocmd(0xAE);ocmd(0xD5);ocmd(0x80);ocmd(0xA8);ocmd(0x3F);ocmd(0xD3);ocmd(0x00);
  ocmd(0x40);ocmd(0xAD);ocmd(0x8B);ocmd(0xA1);ocmd(0xC8);ocmd(0xDA);ocmd(0x12);
  ocmd(0x81);ocmd(0xFF);ocmd(0xD9);ocmd(0x1F);ocmd(0xDB);ocmd(0x40);ocmd(0xA6);ocmd(0xAF);
}
static void sh1106_clear(void)
{
  uint8_t p,i,line[128]; for(i=0;i<128;i++)line[i]=0;
  for(p=0;p<8;p++){ocmd(0xB0+p);ocmd(0x02);ocmd(0x10);(void)i2c_wr(0x40,line,128U);}
}

/* fonte 5x7 (digitos, letras usadas, : . - % espaco) */
static const uint8_t FONT[][5]={
{0,0,0,0,0},{0x3E,0x51,0x49,0x45,0x3E},{0,0x42,0x7F,0x40,0},{0x42,0x61,0x51,0x49,0x46},
{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},
{0x06,0x49,0x49,0x29,0x1E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
{0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},
{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
{0,0x41,0x7F,0x41,0},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
{0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
{0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},
{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},
{0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},
{0,0x36,0x36,0,0},{0x08,0x08,0x08,0x08,0x08},{0,0x60,0x60,0,0},{0x62,0x64,0x08,0x13,0x23}};
static uint8_t fidx(char c){
  if(c==' ')return 0; if(c>='0'&&c<='9')return 1+(c-'0'); if(c>='A'&&c<='Z')return 11+(c-'A');
  if(c==':')return 37; if(c=='-')return 38; if(c=='.')return 39; if(c=='%')return 40; return 0;}
static void otext(uint8_t page,uint8_t col,const char*s){
  ocmd(0xB0+page);ocmd(0x02+(col&0x0F));ocmd(0x10+(col>>4));
  while(*s){uint8_t g[6],k;for(k=0;k<5;k++)g[k]=FONT[fidx(*s)][k];g[5]=0;(void)i2c_wr(0x40,g,6U);s++;}
}
/* converte para string com largura fixa (preenche espacos a direita) */
static void u2s(uint16_t v,char*out){
  char tmp[6];int i=0,j=0; 
  if(v==0){tmp[i++]='0';}
  else { while(v){tmp[i++]='0'+(v%10);v/=10;} }
  while(i)out[j++]=tmp[--i];
  while(j<5)out[j++]=' ';   /* preenche com espacos ate 5 chars */
  out[j]=0;
}

/* ======== decodifica frame em ICM_Data ======== */
static void decode(uint16_t id,const uint8_t*d)
{
  switch(id){
    case 0x100: ICM_Data.rpm = (uint16_t)(d[0]|(d[1]<<8)); break;        /* EngineSpeed */
    case 0x200: ICM_Data.speed_x100 = (uint16_t)(d[0]|(d[1]<<8)); break; /* VehicleSpeed*100 */
    case 0x300: ICM_Data.fuel_x2 = d[0]; ICM_Data.temp = (int16_t)d[1]-40; break;
    case 0x400: ICM_Data.warn = d[0]; break;
    default: break;
  }
}
static void read_rx0(void)
{
  uint8_t sidh,sidl,i,dlc,data[8]; uint16_t id;
  sidh=mcp_read(REG_RXB0SIDH); sidl=mcp_read(REG_RXB0SIDH+1);
  id=(uint16_t)((sidh<<3)|(sidl>>5));
  dlc=mcp_read(REG_RXB0SIDH+4)&0x0F; (void)dlc;
  for(i=0;i<8;i++)data[i]=mcp_read(REG_RXB0SIDH+5+i);
  mcp_bitmod(REG_CANINTF,0x01,0x00);
  GetResource(ICM_Resource);
  decode(id,data);
  ReleaseResource(ICM_Resource);
  rx_count++;
}

/* ======== Trampoline ======== */
#define APP_Task_Task_CAN_RX_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int,OS_APPL_CODE) main(void)
{
  initBoard(); spi1_init(); i2c1_init(); mcp_init(); sh1106_init(); sh1106_clear();
  otext(0,0,"TRMPOLINE CLUSTER");
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}
TASK(Task_CAN_RX){ read_rx0(); TerminateTask(); }
#define APP_Task_Task_CAN_RX_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_Display_START_SEC_CODE
#include "tpl_memmap.h"
TASK(Display)
{
  ICM_Data_t snap; char buf[8];
  GetResource(ICM_Resource); snap=ICM_Data; ReleaseResource(ICM_Resource);

  otext(2,0,"RPM:");
  u2s(snap.rpm,buf); otext(2,48,buf);        /* era 42, empurra p/ 48 */

  otext(3,0,"VEL:");
  u2s((uint16_t)(snap.speed_x100/100U),buf); otext(3,48,buf);

  otext(4,0,"FUEL:");
  u2s((uint16_t)(snap.fuel_x2/2U),buf); otext(4,48,buf);

  otext(5,0,"TMP:");
  { int16_t t=snap.temp; uint16_t tu=(t<0)?(uint16_t)(-t):(uint16_t)t;
    u2s(tu,buf); otext(5,48,buf); }

  TerminateTask();
}
#define APP_Task_Display_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_heartbeat_START_SEC_CODE
#include "tpl_memmap.h"
TASK(heartbeat){ ledToggle(BLUE); TerminateTask(); }
#define APP_Task_heartbeat_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_ISR_ISR_CAN_START_SEC_CODE
#include "tpl_memmap.h"
DeclareTask(Task_CAN_RX);
ISR(ISR_CAN){ ActivateTask(Task_CAN_RX); }
#define APP_ISR_ISR_CAN_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(void,OS_CODE) assert_failed(uint8_t*file,uint32_t line){}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"