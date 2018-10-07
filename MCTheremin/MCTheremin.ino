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
#define MCTHEREMIN_SAMPLE_RATE 16000
#define MCTHEREMIN_INITIAL_CLOCKS 16000
#define MCTHEREMIN_SAMPLE_CLOCKS 80
#define MCTHEREMIN_WAIT_TICKS 10
#define MCTHEREMIN_CALIB_MULT 64

#define MCTHEREMIN_WAIT_CYCLES (MCTHEREMIN_CPU_CLOCK_RATE/MCTHEREMIN_SAMPLE_RATE)
#define MCTHEREMIN_CLOCKS_SAMPLE (MCTHEREMIN_SAMPLE_RATE/MCTHEREMIN_SAMPLE_CLOCKS)

#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define DEMCR_TRCENA    0x01000000

unsigned int lastcycle;
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

FrequencyCounter freqCounter(10000);
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
   interruptButtons = &buttonPanel;
   freqCounter.setAuxFunction(pollButtons_interrupt);
   EEPROMstore.setup(2);
   //EEPROMstore.formatBank(0);
   lastcycle = CPU_CYCLES;
   cursample = 0;
   setup_controls();
   calibrate();
}

void testFrequencyCounter(void)
{
    unsigned int counts1, counts2, ticks;
    static int cnt=0;
    Serial.println(String("starting acquisition ")+cnt);
    cnt++;
    freqCounter.armCounter();
    freqCounter.requestUpdate(100);
    delay(120);
    freqCounter.readUpdate(counts1,counts2,ticks);
    Serial.print("counts = ");
    Serial.print(counts1);
    Serial.print(" ");
    Serial.print(counts2);
    Serial.print(" ");
    Serial.print(ticks);
    Serial.print(" ");
    Serial.print(buttonPanel.getButtonState(0) ? '+' : '-');
    Serial.print(buttonPanel.getButtonState(1) ? '+' : '-');
    Serial.print(buttonPanel.getButtonState(2) ? '+' : '-');
    Serial.println(buttonPanel.getButtonState(3) ? '+' : '-');
    freqCounter.stopCounter();
}

#define MCTHEREMIN_CLOCKS_SAMPLE (MCTHEREMIN_SAMPLE_RATE/MCTHEREMIN_SAMPLE_CLOCKS)

void calibrate(void)
{
  unsigned int volumecount, pitchcount, ticks;

  for (int i=0;i<2;i++)
  {
    digitalWrite(MCTHEREMIN_LED1,LOW);
    if (extrasounds) mcp4921.tone(523,500);
      else delay(500);
    digitalWrite(MCTHEREMIN_LED1,HIGH);
    if (extrasounds) mcp4921.tone(392,1000);
      else delay(500);
  }
  freqCounter.armCounter();
  freqCounter.requestUpdate(MCTHEREMIN_WAIT_TICKS*MCTHEREMIN_CALIB_MULT);
  do
  {
    digitalWrite(MCTHEREMIN_LED1,LOW);
    if (extrasounds) mcp4921.tone(440,30);
        else delay(30);
    digitalWrite(MCTHEREMIN_LED1,HIGH);
    if (extrasounds) mcp4921.tone(494,30);
        else delay(30);
    freqCounter.readUpdate(volumecount,pitchcount,ticks);
  } while (ticks == 0);
  digitalWrite(MCTHEREMIN_LED1,LOW);
  if (extrasounds) mcp4921.tone(392,500);
    else delay(500);
  digitalWrite(MCTHEREMIN_LED1,HIGH);  
  avgvolumecount = volumecount;
  avgpitchcount = pitchcount;
  DEBUGMSG("avgvolumecount=%u avgpitchcount=%u",avgvolumecount,avgpitchcount);
  reseteverything();
}

#define COUNTFILTERFRAC 32
#define COUNTFILTERTOTAL 256

unsigned int initialclocks = 0;
unsigned int volumecount = 0, pitchcount = 0, ticks = 0;
unsigned int volumecountfilter = 0, pitchcountfilter = 0;

void reseteverything(void)
{
  initialclocks = 0;
  volumecount = 0;
  pitchcount = 0;
  ticks = 0;
  volumecountfilter = 10;
  pitchcountfilter = 2048;
}

void loop() {
  static int ledticks = 0;
  digitalWrite(MCTHEREMIN_LED2,((++ledticks) & 0x2000) != 0? LOW : HIGH);
  while (((unsigned int)(CPU_CYCLES-lastcycle)) < MCTHEREMIN_WAIT_CYCLES);
  lastcycle += MCTHEREMIN_WAIT_CYCLES;
  cursample += skipsamples;
  cursample &= 0xFFFFF;
  if (initialclocks < MCTHEREMIN_INITIAL_CLOCKS)
      initialclocks++;
  else
      mcp4921.sendSample((curtone[cursample >> 8] * volumelevel) >> 8);
  freqCounter.readUpdate(volumecount,pitchcount,ticks);
  if (ticks > 0)
  {
     //DEBUGMSG("volumecount=%u pitchcount=%u",volumecount,pitchcount);
     if (pitchcountfilter == 0)
         pitchcountfilter = pitchcount*COUNTFILTERTOTAL;
    else
         pitchcountfilter = (pitchcountfilter*(COUNTFILTERTOTAL-COUNTFILTERFRAC))/COUNTFILTERTOTAL + pitchcount*COUNTFILTERFRAC;
    if (volumecountfilter == 0)
         volumecountfilter = volumecount*COUNTFILTERTOTAL;
    else
         volumecountfilter = (volumecountfilter*(COUNTFILTERTOTAL-COUNTFILTERFRAC))/COUNTFILTERTOTAL + volumecount*COUNTFILTERFRAC;
      volumelevel = avgvolumecount - (volumecountfilter*MCTHEREMIN_CALIB_MULT)/COUNTFILTERTOTAL;
    if (volumelevel > 0)
    {
        volumelevel = 0xFF - volumelevel * volscalefactor / 256;
        if (volumelevel < 0) volumelevel = 0;
    }  else
       volumelevel = 0xFF;
    skipsamples = avgpitchcount - (pitchcountfilter*MCTHEREMIN_CALIB_MULT)/COUNTFILTERTOTAL;
    //DEBUGMSG("skipsamples=%d",skipsamples);
    if (skipsamples > 0)
       skipsamples = (skipsamples * pitchscalefactor)/MCTHEREMIN_CALIB_MULT;
    else
       skipsamples = 0;     
    //DEBUGMSG("skipsamples=%u pitchcountfilter=%u volumecountfilter=%u",skipsamples,(pitchcountfilter*MCTHEREMIN_CALIB_MULT)/COUNTFILTERTOTAL,volumecountfilter);
    //DEBUGMSG("avgpitchcount=%u volumelevel=%u",avgpitchcount,volumelevel);
    if (buttonPanel.getButtonState(0))
    {
        calibrate();
        lastcycle = CPU_CYCLES;
    }  
    pitchscalefactor = analogRead(MCTHEREMIN_PITCHADJ)*2;
    volscalefactor = (((unsigned int)(analogRead(MCTHEREMIN_VOLUMEADJ)))*25)/4096;
    curtone = wavelist[(analogRead(MCTHEREMIN_TIMBRE)*WAVE_NUM)/4096];
    //DEBUGMSG("pitchscalefactor = %u volscalefactor=%u",pitchscalefactor,volscalefactor);
    freqCounter.requestUpdate(MCTHEREMIN_WAIT_TICKS);
  }
}
