#ifndef MC33810_H
#define MC33810_H

#include <SPI.h>
#include "globals.h"
#include BOARD_H //Note that this is not a real file, it is defined in globals.h.
#include "pin.h"

extern IOPortMaskOutputPin MC33810_1_CS;
extern IOPortMaskOutputPin MC33810_2_CS;

//#define MC33810_ONOFF_CMD   3
static const uint8_t MC33810_ONOFF_CMD = 0x30; //48 in decimal
static volatile uint8_t mc33810_1_requestedState; //Current binary state of the 1st ICs IGN and INJ values
static volatile uint8_t mc33810_2_requestedState; //Current binary state of the 2nd ICs IGN and INJ values
static volatile uint8_t mc33810_1_returnState; //Current binary state of the 1st ICs IGN and INJ values
static volatile uint8_t mc33810_2_returnState; //Current binary state of the 2nd ICs IGN and INJ values

void initMC33810(void);

//These are default values for which injector is attached to which output on the IC.
//They may (Probably will) be changed during init by the board specific config in init.ino
#if defined(CORE_TEENSY)
extern byte MC33810_BIT_INJ1;
extern byte MC33810_BIT_INJ2;
extern byte MC33810_BIT_INJ3;
extern byte MC33810_BIT_INJ4;
extern byte MC33810_BIT_INJ5;
extern byte MC33810_BIT_INJ6;
extern byte MC33810_BIT_INJ7;
extern byte MC33810_BIT_INJ8;

extern byte MC33810_BIT_IGN1;
extern byte MC33810_BIT_IGN2;
extern byte MC33810_BIT_IGN3;
extern byte MC33810_BIT_IGN4;
extern byte MC33810_BIT_IGN5;
extern byte MC33810_BIT_IGN6;
extern byte MC33810_BIT_IGN7;
extern byte MC33810_BIT_IGN8;
#else
extern byte MC33810_BIT_INJ1;
extern byte MC33810_BIT_INJ2;
extern byte MC33810_BIT_INJ3;
extern byte MC33810_BIT_INJ4;
extern byte MC33810_BIT_INJ5;
extern byte MC33810_BIT_INJ6;
extern byte MC33810_BIT_INJ7;
extern byte MC33810_BIT_INJ8;

extern byte MC33810_BIT_IGN1;
extern byte MC33810_BIT_IGN2;
extern byte MC33810_BIT_IGN3;
extern byte MC33810_BIT_IGN4;
extern byte MC33810_BIT_IGN5;
extern byte MC33810_BIT_IGN6;
extern byte MC33810_BIT_IGN7;
extern byte MC33810_BIT_IGN8;
#endif

#endif