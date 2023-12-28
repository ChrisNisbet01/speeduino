#include "injector_schedule_direct.h"
#include "globals.h"

//Macros are used to define how each injector control system functions. These are then called by the master openInjectx() function.
//The DIRECT macros (ie individual pins) are defined below. Others should be defined in their relevant acc_x.h file
#define openInjector1_DIRECT()  { *inj1_pin_port |= (inj1_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ1); }
#define closeInjector1_DIRECT() { *inj1_pin_port &= ~(inj1_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ1); }
#define injector1Toggle_DIRECT() (*inj1_pin_port ^= inj1_pin_mask )

#define openInjector2_DIRECT()  { *inj2_pin_port |= (inj2_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ2); }
#define closeInjector2_DIRECT() { *inj2_pin_port &= ~(inj2_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ2); }
#define injector2Toggle_DIRECT() (*inj2_pin_port ^= inj2_pin_mask )

#define openInjector3_DIRECT()  { *inj3_pin_port |= (inj3_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ3); }
#define closeInjector3_DIRECT() { *inj3_pin_port &= ~(inj3_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ3); }
#define injector3Toggle_DIRECT() (*inj3_pin_port ^= inj3_pin_mask )

#define openInjector4_DIRECT()  { *inj4_pin_port |= (inj4_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ4); }
#define closeInjector4_DIRECT() { *inj4_pin_port &= ~(inj4_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ4); }
#define injector4Toggle_DIRECT() (*inj4_pin_port ^= inj4_pin_mask )

#define openInjector5_DIRECT()  { *inj5_pin_port |= (inj5_pin_mask); }
#define closeInjector5_DIRECT() { *inj5_pin_port &= ~(inj5_pin_mask); }
#define injector5Toggle_DIRECT() (*inj5_pin_port ^= inj5_pin_mask )

#define openInjector6_DIRECT()  { *inj6_pin_port |= (inj6_pin_mask); }
#define closeInjector6_DIRECT() { *inj6_pin_port &= ~(inj6_pin_mask); }
#define injector6Toggle_DIRECT() (*inj6_pin_port ^= inj6_pin_mask )

#define openInjector7_DIRECT()  { *inj7_pin_port |= (inj7_pin_mask); }
#define closeInjector7_DIRECT() { *inj7_pin_port &= ~(inj7_pin_mask); }
#define injector7Toggle_DIRECT() (*inj7_pin_port ^= inj7_pin_mask )

#define openInjector8_DIRECT()  { *inj8_pin_port |= (inj8_pin_mask); }
#define closeInjector8_DIRECT() { *inj8_pin_port &= ~(inj8_pin_mask); }
#define injector8Toggle_DIRECT() (*inj8_pin_port ^= inj8_pin_mask )

static void open_injector1_direct(void)
{
  openInjector1_DIRECT();
}

static void open_injector2_direct(void)
{
  openInjector2_DIRECT();
}

static void open_injector3_direct(void)
{
  openInjector3_DIRECT();
}

static void open_injector4_direct(void)
{
  openInjector4_DIRECT();
}

static void open_injector5_direct(void)
{
  openInjector5_DIRECT();
}

static void open_injector6_direct(void)
{
  openInjector6_DIRECT();
}

static void open_injector7_direct(void)
{
  openInjector7_DIRECT();
}

static void open_injector8_direct(void)
{
  openInjector8_DIRECT();
}

static void close_injector1_direct(void)
{
  closeInjector1_DIRECT();
}

static void close_injector2_direct(void)
{
  closeInjector2_DIRECT();
}

static void close_injector3_direct(void)
{
  closeInjector3_DIRECT();
}

static void close_injector4_direct(void)
{
  closeInjector4_DIRECT();
}

static void close_injector5_direct(void)
{
  closeInjector5_DIRECT();
}

static void close_injector6_direct(void)
{
  closeInjector6_DIRECT();
}

static void close_injector7_direct(void)
{
  closeInjector7_DIRECT();
}

static void close_injector8_direct(void)
{
  closeInjector8_DIRECT();
}

static void toggle_injector1_direct(void)
{
  injector1Toggle_DIRECT();
}

static void toggle_injector2_direct(void)
{
  injector2Toggle_DIRECT();
}

static void toggle_injector3_direct(void)
{
  injector3Toggle_DIRECT();
}

static void toggle_injector4_direct(void)
{
  injector4Toggle_DIRECT();
}

static void toggle_injector5_direct(void)
{
  injector5Toggle_DIRECT();
}

static void toggle_injector6_direct(void)
{
  injector6Toggle_DIRECT();
}

static void toggle_injector7_direct(void)
{
  injector7Toggle_DIRECT();
}

static void toggle_injector8_direct(void)
{
  injector8Toggle_DIRECT();
}

static injector_control_st const injector_control_direct[injector_id_COUNT] = {
  [injector_id_1] = {
    .open = open_injector1_direct,
    .close = close_injector1_direct,
    .toggle = toggle_injector1_direct,
  },
  [injector_id_2] = {
    .open = open_injector2_direct,
    .close = close_injector2_direct,
    .toggle = toggle_injector2_direct,
  },
  [injector_id_3] = {
    .open = open_injector3_direct,
    .close = close_injector3_direct,
    .toggle = toggle_injector3_direct,
  },
  [injector_id_4] = {
    .open = open_injector4_direct,
    .close = close_injector4_direct,
    .toggle = toggle_injector4_direct,
  },
  [injector_id_5] = {
    .open = open_injector5_direct,
    .close = close_injector5_direct,
    .toggle = toggle_injector5_direct,
  },
  [injector_id_6] = {
    .open = open_injector6_direct,
    .close = close_injector6_direct,
    .toggle = toggle_injector6_direct,
  },
  [injector_id_7] = {
    .open = open_injector7_direct,
    .close = close_injector7_direct,
    .toggle = toggle_injector7_direct,
  },
  [injector_id_8] = {
    .open = open_injector8_direct,
    .close = close_injector8_direct,
    .toggle = toggle_injector8_direct,
  }
};

static void init_direct_injectors(void)
{
  pinMode(pinInjector1, OUTPUT);
  pinMode(pinInjector2, OUTPUT);
  pinMode(pinInjector3, OUTPUT);
  pinMode(pinInjector4, OUTPUT);
#   if (INJ_CHANNELS >= 5)
  pinMode(pinInjector5, OUTPUT);
#   endif
#   if (INJ_CHANNELS >= 6)
  pinMode(pinInjector6, OUTPUT);
#   endif
#   if (INJ_CHANNELS >= 7)
  pinMode(pinInjector7, OUTPUT);
#   endif
#   if (INJ_CHANNELS >= 8)
  pinMode(pinInjector8, OUTPUT);
#   endif

  inj1_pin_port = portOutputRegister(digitalPinToPort(pinInjector1));
  inj1_pin_mask = digitalPinToBitMask(pinInjector1);
  inj2_pin_port = portOutputRegister(digitalPinToPort(pinInjector2));
  inj2_pin_mask = digitalPinToBitMask(pinInjector2);
  inj3_pin_port = portOutputRegister(digitalPinToPort(pinInjector3));
  inj3_pin_mask = digitalPinToBitMask(pinInjector3);
  inj4_pin_port = portOutputRegister(digitalPinToPort(pinInjector4));
  inj4_pin_mask = digitalPinToBitMask(pinInjector4);
  inj5_pin_port = portOutputRegister(digitalPinToPort(pinInjector5));
  inj5_pin_mask = digitalPinToBitMask(pinInjector5);
  inj6_pin_port = portOutputRegister(digitalPinToPort(pinInjector6));
  inj6_pin_mask = digitalPinToBitMask(pinInjector6);
  inj7_pin_port = portOutputRegister(digitalPinToPort(pinInjector7));
  inj7_pin_mask = digitalPinToBitMask(pinInjector7);
  inj8_pin_port = portOutputRegister(digitalPinToPort(pinInjector8));
  inj8_pin_mask = digitalPinToBitMask(pinInjector8);
}

injectors_st injectors_direct =
{
  .init = init_direct_injectors,
  .control = injector_control_direct,
};

