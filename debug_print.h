// debug_print.h

#ifndef _DEBUG_PRINT_h
#define _DEBUG_PRINT_h

#define DEBUG

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#ifdef DEBUG
	#define DebugPrint(n)	Serial.print(n);
	#define DebugPrintln(n) Serial.println(n);
	#define DebugPrintf(...)  Serial.printf(__VA_ARGS__);
#else
	#define DebugPrint(n)
	#define DebugPrintln(n)
	#define DebugPrintf(...)
#endif

#endif

