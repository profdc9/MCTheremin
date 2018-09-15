#ifndef _MCP4921_H
#define _MCP4921_H

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

#define MCP4921_CHIP_SELECT PB9
#define MCP4921_DAC_LATCH PB8
#define MCP4921_SPI_CHANNEL 1
#define MCP4921_CPU_CLOCK_RATE (F_CPU)

class MCP4921
{
  public:
  
  MCP4921(void)
  {
  };
  
  void setup(void);
  void sendSample(uint16_t sample);
  void tone(unsigned int frequency, unsigned int duration, unsigned int amplitude=4095);
};

#endif /* _BUTTONPANEL_H */

