/*
 * SD_helpers.c
 *
 *  Created on: Jan 17, 2026
 *      Author: Tomasz Pelan
 */

#include "SD/SD_helpers.h"
#include "Buffer/Frame.h"
#include "stdint.h"
#include "ff.h"
#include "math.h"

#define FILENAME_BUFFER_LENGTH 35
#define ASCII_ZERO_SIGN_CODE 48
#define SD_CARD_BUFFER_SIZE 100
#define SD_CARD_SAVE_THRESHOLD 10

extern uint8_t counterSD;
extern UINT bw;
extern uint32_t miliseconds;
extern uint32_t seconds;
extern FATFS fs;
extern FIL fil;
extern FRESULT fresult;
extern char filename[FILENAME_BUFFER_LENGTH];

/* Function converts data from given decimal to hex */
static inline char hex_nibble(uint8_t dec)
{
	return "0123456789ABCDEF"[dec];
}

/* Helper function to get number of digits in given number */
static uint8_t numOfDigits(uint32_t number)
{
	uint8_t numberOfDigits = 0;
	while (number > 0)
	{
		numberOfDigits++;
		number /= 10;
	}
	return numberOfDigits;
}

/* Function returns size of given buffer */
static int bufsize(char* buf)
{
	int i = 0;
	while( *buf++ != '\0') i++;
	return i;
}

/* Function save given frame data to SD card in KVASER-defined format */
void writeFrameToSDCard(Frame* frame)
{
   char bufferSD[SD_CARD_BUFFER_SIZE];
   if (counterSD % SD_CARD_SAVE_THRESHOLD == 0)
	{
	   fresult = f_open(&fil, filename, FA_OPEN_ALWAYS | FA_WRITE);
	}

	// Get to the end of file
	fresult = f_lseek(&fil,  f_size(&fil));
	uint8_t length = 0;
	bufferSD[length++] = ' ';
	bufferSD[length++] = '1';
	bufferSD[length++] = ' ';
	bufferSD[length++] = ' ';
	bufferSD[length++] = ' ';
	bufferSD[length++] = ' ';
	for (int i = 28; i >= 0; i -= 4)
	{
		bufferSD[length++] = hex_nibble(frame->frameStdID >> i & 0xF);
	}

	for (int i = 0; i < 9; i++)
	{
		bufferSD[length++] = ' ';
	}
	bufferSD[length++] = hex_nibble(frame->frameDLC & 0xF);
	bufferSD[length++] = ' ';
	bufferSD[length++] = ' ';

	for (uint8_t i = 0; i < frame->frameDLC; i++)
	{
		bufferSD[length++] = hex_nibble(frame->frameData[i] >> 4 & 0xF);
		bufferSD[length++] = hex_nibble(frame->frameData[i] & 0xF);
		bufferSD[length++] = ' ';
		bufferSD[length++] = ' ';
	}
	for (uint8_t i = 0; i < CAN_DATA_LENGTH - frame->frameDLC + 1; i++)
	{
		bufferSD[length++] = ' ';
		bufferSD[length++] = ' ';
		bufferSD[length++] = ' ';
		bufferSD[length++] = ' ';
	}

	const uint8_t numOfSecondsDigits = numOfDigits(seconds);
	for (int i = 0; i < numOfSecondsDigits; i++)
	{
		const uint32_t divider = pow(10, numOfSecondsDigits - 1 - i);
		const uint8_t numberToWriteInBuffer = seconds / divider;
		seconds -= numberToWriteInBuffer * divider;
		bufferSD[length++] = numberToWriteInBuffer + ASCII_ZERO_SIGN_CODE;
	}
	bufferSD[length++] = '.';
	const uint8_t NUM_OF_MILISECONDS_DIGITS = 6;
	for (int i = 0; i < NUM_OF_MILISECONDS_DIGITS; i++)
	{
		const uint32_t divider = pow(10, NUM_OF_MILISECONDS_DIGITS - 1 - i);
		const uint8_t numberToWriteInBuffer = miliseconds / divider;
		miliseconds -= numberToWriteInBuffer * divider;
		bufferSD[length++] = numberToWriteInBuffer + ASCII_ZERO_SIGN_CODE;
	}
	bufferSD[length++] = ' ';
	bufferSD[length++] = 'R';
	bufferSD[length++] = '\n';
	bufferSD[length++] = '\0';
	fresult = f_write(&fil, bufferSD, bufsize(bufferSD), &bw);

	counterSD++;
	if (counterSD % 10 == 0)
	{
		f_close(&fil);
		counterSD = 0;
	}
}


