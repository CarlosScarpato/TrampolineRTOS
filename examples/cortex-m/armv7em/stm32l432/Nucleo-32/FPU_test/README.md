# Serial example

This application is a stress test for the CPU.

 * A background task performs a calculation requiring floating point operations (the sum of the terms 1/n²). The sequence converges to pi²/6, allowing us to calculate an approximation of pi. Every 1,000 iterations, the value is sent over the serial link.
 * The high-priority task preempts the background task to also perform floating-point calculations.

Reconstruction of the task scheduling with preemption, using a logic analyzer:

![logicScheduling.png](logicScheduling.png) 

The SystemCounter is at 1ms. The serial configuration is 115200, 8N1, with the serial line connected to the debugger (/dev/ttyACM0 on Linux).

`
goil --target=cortex-m/armv7em/stm32l432 --templates=../../../../../../goil/templates/ FPU_test.oil
`
