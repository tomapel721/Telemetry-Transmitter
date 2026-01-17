#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>
#define CAN_DATA_LENGTH 8
/* Struct which contains data to be sent in telemetry */

typedef struct Frame
{
	uint8_t frameData[CAN_DATA_LENGTH];
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint32_t frameStdID;
	uint32_t frameIDE;
	uint32_t frameRTR;
	uint32_t frameDLC;
}Frame;

#endif
