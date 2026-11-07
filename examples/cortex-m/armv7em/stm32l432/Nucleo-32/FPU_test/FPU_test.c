#include "tpl_os.h"
#include "stm32l4xx.h"
#include "pinAccess.h"
#include <math.h>

//see machines/cortex-m/armv7em/stm32l432/lib/serial.h
#include "serial.h"

#define APP_Task_backgroundTask_START_SEC_CODE
#include "tpl_memmap.h"


void updateClock_80MHz();
//update the systick configuration
void tpl_set_systick_timer();

//We use PA6 and PA7 to measure on logic analyser that the fpu computation is effectively preemted by the high priority task.
FUNC(int, OS_APPL_CODE) main(void)
{
  pinMode(GPIOB,3,OUTPUT); //led 1 is PB3
  updateClock_80MHz();
  //call systick config again => System Core Clock updated.
  tpl_set_systick_timer();

  tpl_serial_begin();
  pinMode(GPIOA, 6, OUTPUT); // refered as A5 on Nucleo
  pinMode(GPIOA, 7, OUTPUT); // refered as A6 on Nucleo
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

volatile float shared; //float type => atomic

TASK(highPrioTask)
{
  digitalWrite(GPIOA,6,1);
  volatile double tmp1; //volatile tmp value to limit compiler optimization

  double a = cos(shared);
  double b = sin(shared);
  tmp1 = sqrt(a*a+b*b);
  digitalWrite(GPIOA,6,0);
  TerminateTask();
}

// basic FPU reated calculation
// it computes the Sum of 1/(i^2), from 1 to ...
// This sum converge (slowly) to pi^2/6 => we get an approx of pi
// and print it every 1000 iterations 
TASK(backgroundTask)
{
  volatile double tmp1; //volatile tmp value to limit compiler optimization

  double n = 0.0;
  double un = 0.0;
  int idx = 0;
  while(1)
  {
    digitalWrite(GPIOA,7,1);
    n += 1.0;
    double term = 1/(n*n);
    tmp1 = term;
    un += term;
    double _pi = sqrt(un*6);
    shared = (float)_pi;
    int int_part = (int)_pi;
    int frac_part = (int)((_pi-int_part)*100000000UL);
    idx ++;
    digitalWrite(GPIOA,7,0);

    if(idx % 1000 == 0)
    {
      tpl_serial_wait_for_tx_complete();
      tpl_serial_print_string("_pi: ");
      tpl_serial_print_int(int_part,10,1); //val, base, field width
      tpl_serial_putchar('.');
      tpl_serial_print_int(frac_part,10,8); //val, base, field width
      tpl_serial_print_string("\r\n");
    }
  }


  // int i = 1;
  // un += 1/((double)i*i);
  // double pi = sqrt(un*6);
  // tpl_serial_print_string("count: ");
  // tpl_serial_print_int((int)(pi*1000),10,6); //val, base, field width
  // tpl_serial_print_string("\r\n");
  // i++;
  // digitalToggle(GPIOB,3); //led
  TerminateTask();
}

//tmp TODO: add ISR into library.
void itUsart();
ISR(usart)
{
	itUsart();
}

#define APP_Task_backgroundTask_STOP_SEC_CODE
#include "tpl_memmap.h"

