#include "stm32l4xx.h"

/* system clock:
 * %%%% STM32L432 Nucleo => 80MHz %%%%
 * - use internal HSI RC oscillator => 16MHz
 * - PLL source is HSI  => 16MHz
 * - configure PLLx10/2 => SYSCLK = 80MHz
 * - flash needs 4 wait states (F>64MHz)
 * - APB1 clocks sets to 80MHz
 * - APB2 clocks sets to 80MHz
 *
 * */
void updateClock_80MHz()
{
	// Must set flash latency (4 wait states): F>64MHz
    FLASH->ACR |= 4 << FLASH_ACR_LATENCY_Pos  // add 4 wait states
               |  FLASH_ACR_PRFTEN;           // enable prefetch buffer
	

	//no clock division for APB1
	RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk; 
	//no clock division for APB2
	RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk; 
	//no clock division for AHB
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk; 

    //make sure that MSI is the system clock (default)
	RCC->CFGR &= ~RCC_CFGR_SW_Msk;		// reset SW configuration
	while((RCC->CFGR & RCC_CFGR_SWS) != 0){}

    RCC->CR |= RCC_CR_HSION;		// turn on HSI (16MHz)
	while((RCC->CR & RCC_CR_HSIRDY) == 0){}


	RCC->CR &= ~(RCC_CR_PLLON);			// turn off PLL
    while(RCC->CR & RCC_CR_PLLRDY){}	// wait until pll is stopped

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC_Pos); // reset PLLSRC
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI ; // HSI => 16MHz

    RCC->PLLCFGR |= 7 << RCC_PLLCFGR_PLLPDIV_Pos ; // PLLSAI2CLK = VCO/7

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk ; // reset PLLN (numerator)
    RCC->PLLCFGR |= 10 << RCC_PLLCFGR_PLLN_Pos; //PLLN = 10

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk ; // reset PLLM (denominator) => PLLM=1
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR_Msk ; // reset PLLR (division factor) => PLLR=1

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // enable PLLR (the one for SYSCLK)

    RCC->CR |= RCC_CR_PLLON;			// turn on PLL
    while(!(RCC->CR & RCC_CR_PLLRDY)){}

	RCC->CFGR |= RCC_CFGR_SW_PLL;		// PLL is system clock

	SystemCoreClockUpdate();

}
