/*  MCP4921 */

/*
 * Copyright (c) 2018 Daniel Marks

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */

#include "Arduino.h"
#include "MCP4921.h"
#include "debugmsg.h"

#include <SPI.h>

SPIClass MCP4921_SPI(MCP4921_SPI_CHANNEL);

#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define DEMCR_TRCENA    0x01000000

#ifdef NEED_INITIALIZE_CYCLE_COUNTER
static void initialize_cortex_m3_cycle_counter(void)
{
  DEMCR |= DEMCR_TRCENA; 
  *DWT_CYCCNT = 0; 
  DWT_CTRL |= CYCCNTENA;
}
#endif

void MCP4921::tone(unsigned int frequency, unsigned int duration, unsigned int amplitude)
{
  unsigned int waitcycles = (MCP4921_CPU_CLOCK_RATE / 2) / frequency;
  unsigned int cycles = (duration * (MCP4921_CPU_CLOCK_RATE / 2000))/waitcycles;
  unsigned int temp;

  while (cycles>0)
  {
    cycles--;
    temp = CPU_CYCLES;
    while (((unsigned int)(CPU_CYCLES - temp)) < waitcycles);
    sendSample(amplitude);
    temp = CPU_CYCLES;
    while (((unsigned int)(CPU_CYCLES - temp)) < waitcycles);
    sendSample(0);
  }
}

void MCP4921::sendSample(uint16_t sample)
{
  byte data;
  digitalWrite(MCP4921_CHIP_SELECT, LOW); 
  data = MCP4921_SPI.transfer((byte)(0x30 | ((sample & 0xF00) >> 8))); 
  data = MCP4921_SPI.transfer((byte)(sample & 0xFF)); 
  digitalWrite(MCP4921_CHIP_SELECT, HIGH);
}

void MCP4921::setup(void)
{
#ifdef NEED_INITIALIZE_CYCLE_COUNTER
  initialize_cortex_m3_cycle_counter();
#endif

  afio_remap(AFIO_REMAP_SPI1); // remap SPI1

  gpio_set_mode(GPIOB, 3, GPIO_AF_OUTPUT_PP);
  gpio_set_mode(GPIOB, 4, GPIO_INPUT_FLOATING);
  gpio_set_mode(GPIOB, 5, GPIO_AF_OUTPUT_PP);
  
  MCP4921_SPI.begin(); 
  MCP4921_SPI.setBitOrder(MSBFIRST); 
  MCP4921_SPI.setDataMode(SPI_MODE0); 
  MCP4921_SPI.setClockDivider(SPI_CLOCK_DIV32);  

  pinMode(MCP4921_DAC_LATCH,OUTPUT);
  digitalWrite(MCP4921_DAC_LATCH,LOW);

  pinMode(MCP4921_CHIP_SELECT,OUTPUT);
  digitalWrite(MCP4921_CHIP_SELECT,HIGH);
}



