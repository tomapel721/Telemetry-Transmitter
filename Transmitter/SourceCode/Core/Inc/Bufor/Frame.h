#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>

/*  Prototyp struktury danych przechowuj¹cej wszystkie informacje,
	które maj¹ zostaæ póŸniej wys³ane								*/

typedef struct Frame
{
	uint8_t frameData[8];
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint32_t frameStdID; // DLC,RTR,STRid,IDE
	uint32_t frameIDE;
	uint32_t frameRTR;
	uint32_t frameDLC;
}Frame;



#endif