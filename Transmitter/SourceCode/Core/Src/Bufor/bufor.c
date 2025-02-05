/*
 *  bufor.c
 *
 *  Created on: July 6, 2021
 *  Company: Polsl Racing
 *  Department: Electronics Team
 *  Author: Tomasz Pelan
 */
#include "Bufor/bufor.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


void InitBuffer(Buffer* buf)
{
	if (buf == NULL)
	{
		return;
	}
		buf->possiblePlaces[0] = false;
		buf->possiblePlaces[1] = false;
		buf->possiblePlaces[2] = false;
}

void CopyFrame(Frame* source, Frame* destination)
{
	if (source == NULL || destination == NULL)
	{
		return;
	}
	memcpy(destination, source, sizeof(Frame));
}

void AddToBuffer(Buffer* buf, Frame* addedFrame)
{

	if (buf == NULL || addedFrame == NULL)
	{
		return;
	}
	static uint32_t licznik1 = 0, licznik2 = 0, licznik3 = 0;
	if (!IsBufferFull(buf))
	{
		if (!buf->possiblePlaces[0])
		{
			CopyFrame(addedFrame, &(buf->first));
			buf->possiblePlaces[0] = true;
			licznik1++;
		}
		else if (!buf->possiblePlaces[1])
		{
			CopyFrame(addedFrame, &(buf->second));
			buf->possiblePlaces[1] = true;
			licznik2++;
		}
		else if (!buf->possiblePlaces[2])
		{
			CopyFrame(addedFrame, &(buf->third));
			buf->possiblePlaces[2] = true;
			licznik3++;
		}
	}
	
}

void RemoveFromBuffer(Buffer* buf, Frame* removedFrame)
{
	if (buf == NULL || removedFrame == NULL)
	{
		return;
	}

	if (!IsBufferEmpty(buf))
	{
		CopyFrame(&(buf->first), removedFrame);
		if (buf->possiblePlaces[1])
		{
			CopyFrame(&(buf->second), &(buf->first));
			if (buf->possiblePlaces[2])
			{
				CopyFrame(&(buf->third), &(buf->second));
			}
			else
			{
				buf->possiblePlaces[1] = false;
			}
		}
		else
		{
			buf->possiblePlaces[0] = false;
		}
		buf->possiblePlaces[2] = false;
	}
	
}

bool IsBufferEmpty(Buffer* buf)
{
	if (buf == NULL)
	{
		return;
	}

	if (buf->possiblePlaces[0] == false && 
		buf->possiblePlaces[1] == false && 
		buf->possiblePlaces[2] == false)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

bool IsBufferFull(Buffer* buf)
{
	if (buf == NULL)
	{
		return;
	}

	if (buf->possiblePlaces[0] == true && 
		buf->possiblePlaces[1] == true && 
		buf->possiblePlaces[2] == true)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}
