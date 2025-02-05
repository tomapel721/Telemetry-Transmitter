/*
 *  bufor.h
 *
 *  Created on: July 6, 2021
 *  Company: Polsl Racing
 *  Department: Electronics Team
 *  Author: Tomasz Pelan
 */
#ifndef BUFOR_H
#define BUFOR_H
#include "Frame.h"
#include <stdint.h>
#include <stdbool.h>
typedef struct Buffer
{
	Frame first;
	Frame second;
	Frame third;
	bool possiblePlaces[3];
	
} Buffer;
	
	/* Funkcja inicjalizujace zmienne w buforze */
	void InitBuffer(Buffer* buf);

	/* Funkcja dodajaca nowy element do bufora */
	void AddToBuffer(Buffer* buf,Frame* addedFrame);

	/* Funkcja usuwajaca z bufora element pierwszy(i jednoczesnie przesuwa pozostale) */
	void RemoveFromBuffer(Buffer* buf,Frame* removedFrame);

	/* Funkcja sprawdzajaca czy bufor jest pusty */
	bool IsBufferEmpty(Buffer* buf);

	/* Funkcja sprawdzajaca czy bufor jest pelny */
	bool IsBufferFull(Buffer* buf);

	/* Funkcja kopiujaca dane z jednej ramki do drugiej */
	void CopyFrame(Frame* source, Frame* destination); // przetestowane - dzia³a

#endif