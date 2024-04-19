#include "acc_mc33810.h"
#include "globals.h"
#include <SPI.h>

IOPortMaskOutputPin MC33810_1_CS;
IOPortMaskOutputPin MC33810_2_CS;

#if defined(CORE_TEENSY)
byte MC33810_BIT_INJ1 = 3;
byte MC33810_BIT_INJ2 = 1;
byte MC33810_BIT_INJ3 = 0;
byte MC33810_BIT_INJ4 = 2;
byte MC33810_BIT_INJ5 = 3;
byte MC33810_BIT_INJ6 = 1;
byte MC33810_BIT_INJ7 = 0;
byte MC33810_BIT_INJ8 = 2;

byte MC33810_BIT_IGN1 = 4;
byte MC33810_BIT_IGN2 = 5;
byte MC33810_BIT_IGN3 = 6;
byte MC33810_BIT_IGN4 = 7;
byte MC33810_BIT_IGN5 = 4;
byte MC33810_BIT_IGN6 = 5;
byte MC33810_BIT_IGN7 = 6;
byte MC33810_BIT_IGN8 = 7;
#else
byte MC33810_BIT_INJ1 = 1;
byte MC33810_BIT_INJ2 = 2;
byte MC33810_BIT_INJ3 = 3;
byte MC33810_BIT_INJ4 = 4;
byte MC33810_BIT_INJ5 = 5;
byte MC33810_BIT_INJ6 = 6;
byte MC33810_BIT_INJ7 = 7;
byte MC33810_BIT_INJ8 = 8;

byte MC33810_BIT_IGN1 = 1;
byte MC33810_BIT_IGN2 = 2;
byte MC33810_BIT_IGN3 = 3;
byte MC33810_BIT_IGN4 = 4;
byte MC33810_BIT_IGN5 = 5;
byte MC33810_BIT_IGN6 = 6;
byte MC33810_BIT_IGN7 = 7;
byte MC33810_BIT_IGN8 = 8;
#endif

static bool done_init = false;

void initMC33810(void)
{
  if (done_init)
  {
    return;
  }
  done_init = true;

  //Set pin port/masks
  MC33810_1_CS.configure();
  MC33810_1_CS.configure();

  //Set the output states of both ICs to be off to fuel and ignition
  mc33810_1_requestedState = 0;
  mc33810_2_requestedState = 0;
  mc33810_1_returnState = 0;
  mc33810_2_returnState = 0;

  SPI.begin();
  //These are the SPI settings per the datasheet
  SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));

  //Set the ignition outputs to GPGD mode
  /*
  0001 = Mode select command
  1111 = Set all 1 GD[0...3] outputs to use GPGD mode
  00000000 = All remaining values are unused (For us)
  */
  uint16_t cmd = 0b0001111100000000;
  //IC1
  MC33810_1_CS.off();
  SPI.transfer16(cmd);
  MC33810_1_CS.on();
  //IC2
  MC33810_2_CS.off();
  SPI.transfer16(cmd);
  MC33810_2_CS.on();
}

