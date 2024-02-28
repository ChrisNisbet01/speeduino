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
typedef enum mc33810_injector_pin_assignments_t
{
#if defined(CORE_TEENSY)
  MC33810_BIT_INJ1 = 3,
  MC33810_BIT_INJ2 = 1,
  MC33810_BIT_INJ3 = 0,
  MC33810_BIT_INJ4 = 2,
  MC33810_BIT_INJ5 = 3,
  MC33810_BIT_INJ6 = 1,
  MC33810_BIT_INJ7 = 0,
  MC33810_BIT_INJ8 = 2,
#else
  MC33810_BIT_INJ1 = 1,
  MC33810_BIT_INJ2 = 2,
  MC33810_BIT_INJ3 = 3,
  MC33810_BIT_INJ4 = 4,
  MC33810_BIT_INJ5 = 5,
  MC33810_BIT_INJ6 = 6,
  MC33810_BIT_INJ7 = 7,
  MC33810_BIT_INJ8 = 8,
#endif
} mc33810_injector_pin_assignments_t;

typedef enum mc33810_ignition_pin_assignments_t
{
#if defined(CORE_TEENSY)
  MC33810_BIT_IGN1 = 4,
  MC33810_BIT_IGN2 = 5,
  MC33810_BIT_IGN3 = 6,
  MC33810_BIT_IGN4 = 7,
  MC33810_BIT_IGN5 = 4,
  MC33810_BIT_IGN6 = 5,
  MC33810_BIT_IGN7 = 6,
  MC33810_BIT_IGN8 = 7,
#else
  MC33810_BIT_IGN1 = 1,
  MC33810_BIT_IGN2 = 2,
  MC33810_BIT_IGN3 = 3,
  MC33810_BIT_IGN4 = 4,
  MC33810_BIT_IGN5 = 5,
  MC33810_BIT_IGN6 = 6,
  MC33810_BIT_IGN7 = 7,
  MC33810_BIT_IGN8 = 8,
#endif
} mc33810_ignition_pin_assignments_t;


#define openInjector1_MC33810() MC33810_1_CS.off(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define openInjector2_MC33810() MC33810_1_CS.off(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define openInjector3_MC33810() MC33810_1_CS.off(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define openInjector4_MC33810() MC33810_1_CS.off(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define openInjector5_MC33810() MC33810_2_CS.off(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define openInjector6_MC33810() MC33810_2_CS.off(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define openInjector7_MC33810() MC33810_2_CS.off(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define openInjector8_MC33810() MC33810_2_CS.off(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()

#define closeInjector1_MC33810() MC33810_1_CS.off(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define closeInjector2_MC33810() MC33810_1_CS.off(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define closeInjector3_MC33810() MC33810_1_CS.off(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define closeInjector4_MC33810() MC33810_1_CS.off(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define closeInjector5_MC33810() MC33810_2_CS.off(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define closeInjector6_MC33810() MC33810_2_CS.off(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define closeInjector7_MC33810() MC33810_2_CS.off(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define closeInjector8_MC33810() MC33810_2_CS.off(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()

#define injector1Toggle_MC33810() MC33810_1_CS.off(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define injector2Toggle_MC33810() MC33810_1_CS.off(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define injector3Toggle_MC33810() MC33810_1_CS.off(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define injector4Toggle_MC33810() MC33810_1_CS.off(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define injector5Toggle_MC33810() MC33810_2_CS.off(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_INJ5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define injector6Toggle_MC33810() MC33810_2_CS.off(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_INJ6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define injector7Toggle_MC33810() MC33810_2_CS.off(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_INJ7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define injector8Toggle_MC33810() MC33810_2_CS.off(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_INJ8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()

#define coil1High_MC33810() MC33810_1_CS.off(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil2High_MC33810() MC33810_1_CS.off(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil3High_MC33810() MC33810_1_CS.off(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil4High_MC33810() MC33810_1_CS.off(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil5High_MC33810() MC33810_2_CS.off(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_IGN5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define coil6High_MC33810() MC33810_2_CS.off(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_IGN6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define coil7High_MC33810() MC33810_2_CS.off(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_IGN7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define coil8High_MC33810() MC33810_2_CS.off(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_IGN8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()

#define coil1Low_MC33810() MC33810_1_CS.off(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_IGN1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil2Low_MC33810() MC33810_1_CS.off(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_IGN2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil3Low_MC33810() MC33810_1_CS.off(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_IGN3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil4Low_MC33810() MC33810_1_CS.off(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_IGN4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil5Low_MC33810() MC33810_2_CS.off(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_IGN5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define coil6Low_MC33810() MC33810_2_CS.off(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_IGN6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define coil7Low_MC33810() MC33810_2_CS.off(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_IGN7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define coil8Low_MC33810() MC33810_2_CS.off(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_IGN8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()

#define coil1Toggle_MC33810() MC33810_1_CS.off(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_IGN1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil2Toggle_MC33810() MC33810_1_CS.off(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_IGN2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil3Toggle_MC33810() MC33810_1_CS.off(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_IGN3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil4Toggle_MC33810() MC33810_1_CS.off(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_IGN4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_CS.on()
#define coil5Toggle_MC33810() MC33810_2_CS.off(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_IGN5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define coil6Toggle_MC33810() MC33810_2_CS.off(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_IGN6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define coil7Toggle_MC33810() MC33810_2_CS.off(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_IGN7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()
#define coil8Toggle_MC33810() MC33810_2_CS.off(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_IGN8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_CS.on()

#endif