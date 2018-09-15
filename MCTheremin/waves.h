#ifndef _WAVES_H
#define _WAVES_H

#define WAVE_NUM 4
#define WAVE_TABLESIZE 4096

extern const unsigned short sinewave[WAVE_TABLESIZE];
extern const unsigned short triangle[WAVE_TABLESIZE];
extern const unsigned short distorted[WAVE_TABLESIZE];
extern const unsigned short distorted2[WAVE_TABLESIZE];

extern const unsigned short *wavelist[WAVE_NUM];

#endif /* _WAVES_H */
