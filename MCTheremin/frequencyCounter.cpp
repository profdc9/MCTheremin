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
HardwareTimer timeb(3);

enum freq_counter_state { FREQ_COUNTER_IDLE, FREQ_COUNTER_ARMED, FREQ_COUNTER_PULSES };
enum freq_counter_read_state { FREQ_COUNTER_READ_IDLE, FREQ_COUNTER_READ_WAITING, FREQ_COUNTER_READ_READY };

typedef struct _active_counter
{
	freq_counter_state state;
	freq_counter_read_state state_read;
	unsigned short count_last1, count_last2;
	unsigned int total_counts1, total_counts2;
	unsigned int number_of_ticks;
	unsigned int read_total_counts1, read_total_counts2;
	unsigned int read_number_of_ticks;
  unsigned int stop_acquire_ticks;
  AuxCounterFunction aux_fcn;
} freq_active_counter;

static volatile freq_active_counter active_ctr;

static void freqCounterInterrupt(void)
{
	unsigned short current_cnt2 = time2.getCount();  // must be first in interrupt!
  unsigned short current_cnt1 = time1.getCount();  // must be first in interrupt!
  if (active_ctr.aux_fcn != NULL)
      (*active_ctr.aux_fcn)();
	if (active_ctr.state == FREQ_COUNTER_PULSES)
	{
		active_ctr.number_of_ticks++;
	  active_ctr.total_counts2 += (current_cnt2 - active_ctr.count_last2) & 0xFFFF;
    active_ctr.total_counts1 += (current_cnt1 - active_ctr.count_last1) & 0xFFFF;
		active_ctr.count_last2 = current_cnt2;
    active_ctr.count_last1 = current_cnt1;
		if ((active_ctr.state_read == FREQ_COUNTER_READ_WAITING) && (active_ctr.number_of_ticks >= active_ctr.stop_acquire_ticks))
		{
			active_ctr.read_number_of_ticks = active_ctr.number_of_ticks;
			active_ctr.read_total_counts2 = active_ctr.total_counts2;
      active_ctr.read_total_counts1 = active_ctr.total_counts1;
      active_ctr.total_counts2 = 0;
      active_ctr.total_counts1 = 0;
      active_ctr.number_of_ticks = 0;
			active_ctr.state_read = FREQ_COUNTER_READ_READY;
		}
		return;
	} else if (active_ctr.state == FREQ_COUNTER_ARMED)
	{
    active_ctr.count_last2 = current_cnt2;
		active_ctr.count_last1 = current_cnt1;
		active_ctr.total_counts2 = 0;
    active_ctr.total_counts1 = 0;
		active_ctr.number_of_ticks = 0;
		active_ctr.state = FREQ_COUNTER_PULSES;
		return;
	}
}

void FrequencyCounter::armCounter(void)
{
  active_ctr.state = FREQ_COUNTER_IDLE;
  active_ctr.state_read = FREQ_COUNTER_READ_IDLE; 
	active_ctr.state = FREQ_COUNTER_ARMED;
}

void FrequencyCounter::requestUpdate(unsigned int stopticks)
{
	active_ctr.read_total_counts1 = 0;
  active_ctr.read_total_counts2 = 0;
	active_ctr.read_number_of_ticks = 0;
	active_ctr.state_read = FREQ_COUNTER_READ_WAITING;
  active_ctr.stop_acquire_ticks = stopticks;
}

void FrequencyCounter::readUpdate(unsigned int &counts1, unsigned int &counts2, unsigned int &ticks)
{
 	if (active_ctr.state_read != FREQ_COUNTER_READ_READY)
  {
    counts1 = counts2 = ticks = 0;
    return;
  }
  counts1 = active_ctr.read_total_counts1;
  counts2 = active_ctr.read_total_counts2;
  ticks = active_ctr.read_number_of_ticks;
  return;
}

void FrequencyCounter::stopCounter(void)
{ 
  active_ctr.state = FREQ_COUNTER_IDLE;
  return;
}

void FrequencyCounter::setAuxFunction(AuxCounterFunction pAuxFcn)
{
   active_ctr.aux_fcn = pAuxFcn;
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

  active_ctr.state = FREQ_COUNTER_IDLE;
  active_ctr.state_read = FREQ_COUNTER_READ_IDLE;
  active_ctr.aux_fcn = NULL;
  timeb.pause();
  timeb.setCount(0);
  timeb.setPrescaleFactor(prescaleFactor);
  timeb.setOverflow(checkInterval);
  timeb.setCompare(TIMER_CH1, 1);
  timeb.attachCompare1Interrupt(freqCounterInterrupt);
  timeb.refresh();
  timeb.resume();
}
