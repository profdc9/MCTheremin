/*
 *  Theremin STM32duino project
 */

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

#include "FrequencyCounter.h"
#include "ButtonPanel.h"
#include "MCP4921.h"
#include "debugmsg.h"
#include "EEPROMobjectstore.h"
#include "waves.h"

#define MCTHEREMIN_LED1 PB12
#define MCTHEREMIN_LED2 PB14
#define MCTHEREMIN_INTLED PB7
#define MCTHEREMIN_ALTSIG PB6

#define MCTHEREMIN_PITCHADJ 4
#define MCTHEREMIN_SENSITIVITY 5
#define MCTHEREMIN_TIMBRE 6
#define MCTHEREMIN_VOLUMEADJ 7

#define MCTHEREMIN_CPU_CLOCK_RATE (F_CPU)
#define MCTHEREMIN_SAMPLE_RATE 32000
#define MCTHEREMIN_INITIAL_CLOCKS 16000
#define MCTHEREMIN_WAIT_TICKS 1

#define MCTHEREMIN_WAIT_CYCLES (MCTHEREMIN_CPU_CLOCK_RATE/MCTHEREMIN_SAMPLE_RATE)

#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define DEMCR_TRCENA    0x01000000

int skipsamples;
int volumelevel;
int cursample;
unsigned int volume;
const unsigned short *curtone = sinewave;
bool extrasounds = false;

unsigned int avgvolumecount;
unsigned int avgpitchcount;
int pitchscalefactor = 2048;
int volscalefactor = 10;

static void initialize_cortex_m3_cycle_counter(void)
{
  DEMCR |= DEMCR_TRCENA; 
  *DWT_CYCCNT = 0; 
  DWT_CTRL |= CYCCNTENA;
}

void readInterrupt(void);
 
HardwareTimer timeb(3);

void setup_timer(unsigned short checkInterval, unsigned short prescaleFactor)
{
  timeb.pause();
  timeb.setCount(0);
  timeb.setPrescaleFactor(prescaleFactor);
  timeb.setOverflow(checkInterval);
  timeb.setCompare(TIMER_CH1, 1);
  timeb.attachCompare1Interrupt(readInterrupt);
  timeb.refresh();
  timeb.resume();  
}

FrequencyCounter freqCounter(2000);
const byte button_list[4] = {PB0,PB1,PB10,PB11};
ButtonPanel buttonPanel(4,button_list,10,10);
MCP4921 mcp4921;

void setup_controls(void)
{
  pinMode(MCTHEREMIN_ALTSIG,INPUT);
  pinMode(MCTHEREMIN_LED1,OUTPUT);
  digitalWrite(MCTHEREMIN_LED1,HIGH);
  pinMode(MCTHEREMIN_LED2,OUTPUT);
  digitalWrite(MCTHEREMIN_LED2,HIGH);
  pinMode(MCTHEREMIN_INTLED,OUTPUT);
  digitalWrite(MCTHEREMIN_INTLED,HIGH);
  pinMode(MCTHEREMIN_PITCHADJ,INPUT_ANALOG);
  pinMode(MCTHEREMIN_TIMBRE,INPUT_ANALOG);
  pinMode(MCTHEREMIN_SENSITIVITY,INPUT_ANALOG);
  pinMode(MCTHEREMIN_VOLUMEADJ,INPUT_ANALOG);
}

void setup() {
   afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); // release PB3 and PB5 
//   disableDebugPorts();
   initialize_cortex_m3_cycle_counter();
  // default debug messages off if on UART1 (might need it for rig)
#ifdef SERIAL_USB
  setDebugMsgMode(1);
#else
  setDebugMsgMode(0);
#endif
   mcp4921.setup();
   freqCounter.setup();
   Serial.begin(115200); // Ignored by Maple. But needed by boards using Hardware serial via a USB to Serial Adaptor
   buttonPanel.setup();
   EEPROMstore.setup(2);
   //EEPROMstore.formatBank(0);
   cursample = 0;
   setup_controls();
   setup_timer(MCTHEREMIN_WAIT_CYCLES,1);
   calibrate();
}

unsigned int initialclocks = 0;
unsigned int lastvolumeval = 0, lastpitchval = 0;
unsigned int volumecountfilter = 0, pitchcountfilter = 0;
volatile unsigned int ticks = 0;
unsigned int lastticks = 0;
int circbufpl = 0;
int analogcircbufpl = 0;
volatile bool soundOn = false;

#define CIRCBUFFERLEN 1024

unsigned short pitchsamples[CIRCBUFFERLEN];
unsigned short volumesamples[CIRCBUFFERLEN];
unsigned short pitchknobsamples[CIRCBUFFERLEN];
unsigned short volumeknobsamples[CIRCBUFFERLEN];

void reseteverything(void)
{
  initialclocks = 0;
  circbufpl = 0;
  analogcircbufpl = 0;
  cursample = 0;
  ticks = 0;
  for (int i=0;i<CIRCBUFFERLEN;i++) 
  {
    pitchsamples[i] = 0;
    pitchknobsamples[i] = 0;
    volumesamples[i] = 0;
    volumeknobsamples[i] = 0;
  }
}

 void calculatecounts(void)
 {
  pitchcountfilter = 0;
  volumecountfilter = 0;
  pitchscalefactor = 0;
  volscalefactor = 0;
  for (int i=0;i<CIRCBUFFERLEN;i++)
  {
     pitchcountfilter += pitchsamples[i];
     volumecountfilter += volumesamples[i];
     pitchscalefactor += pitchknobsamples[i];
     volscalefactor += volumeknobsamples[i];
  }
 }

 void readInterrupt(void) 
 {
  unsigned int volumeval, pitchval;
  ticks++;
  freqCounter.readUpdate(volumeval,pitchval);
  if (initialclocks < MCTHEREMIN_INITIAL_CLOCKS)
      initialclocks++;
  else
  {
      if (soundOn)
      {
          cursample += skipsamples;
          cursample &= 0xFFFFF;
          mcp4921.sendSample((curtone[cursample >> 8] * volumelevel) >> 8);
      }
  }
  circbufpl++;
  if (circbufpl >= CIRCBUFFERLEN)
      circbufpl = 0;
  pitchsamples[circbufpl] = (pitchval - lastpitchval) & 0xFFFF;
  lastpitchval = pitchval;
  volumesamples[circbufpl] = (volumeval - lastvolumeval) & 0xFFFF;
  lastvolumeval = volumeval;
  buttonPanel.pollButtons(); 
}

void getanalogcntrls(void)
{
  analogcircbufpl++;
  if (analogcircbufpl >= CIRCBUFFERLEN)
      analogcircbufpl = 0;
  pitchknobsamples[analogcircbufpl] = analogRead(MCTHEREMIN_PITCHADJ);
  volumeknobsamples[analogcircbufpl] = analogRead(MCTHEREMIN_VOLUMEADJ); 
}

void calibrate(void)
{
  soundOn = false;
  for (int i=0;i<2;i++)
  {
    digitalWrite(MCTHEREMIN_LED1,LOW);
    if (extrasounds) mcp4921.tone(523,500);
      else delay(500);
    digitalWrite(MCTHEREMIN_LED1,HIGH);
    if (extrasounds) mcp4921.tone(392,1000);
      else delay(500);
  }
  reseteverything();
  while (ticks < 50000)
    digitalWrite(MCTHEREMIN_LED1,(ticks & 0x400) ? HIGH : LOW);
  calculatecounts();
  avgvolumecount = volumecountfilter;
  avgpitchcount = pitchcountfilter;
  digitalWrite(MCTHEREMIN_LED1,LOW);
  if (extrasounds) mcp4921.tone(392,500);
    else delay(500);
  digitalWrite(MCTHEREMIN_LED1,HIGH);  
  //reseteverything();
  soundOn = true;
}

void loop() {
  static int ledticks = 0;
  digitalWrite(MCTHEREMIN_LED2,((++ledticks) & 0x400) != 0? LOW : HIGH);

  while (lastticks == ticks) {};

  getanalogcntrls();

  calculatecounts();
  volumelevel = avgvolumecount - volumecountfilter;
  volscalefactor = volscalefactor*35/(4096*CIRCBUFFERLEN);
  if (volumelevel < 0)
      volumelevel = 0xFF;
  else
  {
      volumelevel = 0xFF - volumelevel * volscalefactor / 16;
      if (volumelevel < 0) volumelevel = 0;
  }
  pitchscalefactor /= CIRCBUFFERLEN;
  skipsamples = (avgpitchcount - pitchcountfilter) * pitchscalefactor;
  if (skipsamples < 8192)
    skipsamples = 0;
  else
    skipsamples = skipsamples / 4;
  if (buttonPanel.getButtonState(0))
     calibrate(); 
  curtone = wavelist[(analogRead(MCTHEREMIN_TIMBRE)*WAVE_NUM)/4096];
   /*{
     static int updat =0;
     updat++;
     if (updat > 2000)
     {
      updat = 0;
      DEBUGMSG("pitchscalefactor = %u volscalefactor=%u",pitchscalefactor,volscalefactor);
      DEBUGMSG("skipsamples=%u pitchcountfilter=%u volumecountfilter=%u",skipsamples,pitchcountfilter,volumecountfilter);
      DEBUGMSG("avgpitchcount=%u avgvolumecount=%u volumelevel=%u",avgpitchcount,avgvolumecount,volumelevel);
     }
  } */
}
