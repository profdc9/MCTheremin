/*  Frequency Counter Class */

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
#include "frequencyCounter.h"
#include "debugmsg.h"

HardwareTimer time1(1);
HardwareTimer time2(2);

void FrequencyCounter::readUpdate(unsigned int &counts1, unsigned int &counts2)
{
  counts1 = time1.getCount();
  counts2 = time2.getCount();
}

void FrequencyCounter::setup(void)
{
  time2.pause();
  time2.setCount(0);
  time2.setPrescaleFactor(1);
  time2.setOverflow(65535);  
  pinMode(PA1,INPUT);
  TIMER2->regs.gen->CCMR1 |= 0x0101; // Ch. 2 as TI2 CC2S
  TIMER2->regs.gen->SMCR |= 0x0007; // Ext. clk mode 1
  TIMER2->regs.gen->SMCR |= 0x0060; // TI2FP2 is the trigger
  TIMER2->regs.gen->CR1 |= 0x0001; // enable counting 
  time2.refresh();
  time2.resume();

  time1.pause();
  time1.setCount(0);
  time1.setPrescaleFactor(1);
  time1.setOverflow(65535);  
  pinMode(PA8,INPUT);
  TIMER1->regs.gen->CCMR1 |= 0x0001; // Ch. 1 as TI1
  TIMER1->regs.gen->SMCR |= 0x0007; // Ext. clk mode 1
  TIMER1->regs.gen->SMCR |= 0x0050; // TI1FP1 is the trigger
  TIMER1->regs.gen->CR1 |= 0x0001; // enable counting 
  time1.refresh();
  time1.resume();
}
