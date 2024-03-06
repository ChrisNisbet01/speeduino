#include "injector_schedule_mc33810.h"
#include "acc_mc33810.h"
#include "bit_macros.h"

typedef void (&open_injector_fn)(void);
typedef void (&close_injector_fn)(void);
typedef void (&toggle_injector_fn)(void);

struct mc33810_injector_control_st
{
  mc33810_injector_control_st(open_injector_fn &_open, close_injector_fn &_close, toggle_injector_fn &_toggle)
    : open(_open),
      close(_close),
      toggle(_toggle)
  {
  }
  open_injector_fn open;
  close_injector_fn close;
  toggle_injector_fn toggle;
};

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

static void open_injector1_mc33810(void)
{
  openInjector1_MC33810();
}

static void open_injector2_mc33810(void)
{
  openInjector2_MC33810();
}

static void open_injector3_mc33810(void)
{
  openInjector3_MC33810();
}

static void open_injector4_mc33810(void)
{
  openInjector4_MC33810();
}

#if INJ_CHANNELS >= 5
static void open_injector5_mc33810(void)
{
  openInjector5_MC33810();
}
#endif

#if INJ_CHANNELS >= 6
static void open_injector6_mc33810(void)
{
  openInjector6_MC33810();
}
#endif

#if INJ_CHANNELS >= 7
static void open_injector7_mc33810(void)
{
  openInjector7_MC33810();
}
#endif

#if INJ_CHANNELS >= 8
static void open_injector8_mc33810(void)
{
  openInjector8_MC33810();
}
#endif

static void close_injector1_mc33810(void)
{
  closeInjector1_MC33810();
}

static void close_injector2_mc33810(void)
{
  closeInjector2_MC33810();
}

static void close_injector3_mc33810(void)
{
  closeInjector3_MC33810();
}

static void close_injector4_mc33810(void)
{
  closeInjector4_MC33810();
}

#if INJ_CHANNELS >= 5
static void close_injector5_mc33810(void)
{
  closeInjector5_MC33810();
}
#endif

#if INJ_CHANNELS >= 6
static void close_injector6_mc33810(void)
{
  closeInjector6_MC33810();
}
#endif

#if INJ_CHANNELS >= 7
static void close_injector7_mc33810(void)
{
  closeInjector7_MC33810();
}
#endif

#if INJ_CHANNELS >= 8
static void close_injector8_mc33810(void)
{
  closeInjector8_MC33810();
}
#endif

static void toggle_injector1_mc33810(void)
{
  injector1Toggle_MC33810();
}

static void toggle_injector2_mc33810(void)
{
  injector2Toggle_MC33810();
}

static void toggle_injector3_mc33810(void)
{
  injector3Toggle_MC33810();
}

static void toggle_injector4_mc33810(void)
{
  injector4Toggle_MC33810();
}

#if INJ_CHANNELS >= 5
static void toggle_injector5_mc33810(void)
{
  injector5Toggle_MC33810();
}
#endif

#if INJ_CHANNELS >= 6
static void toggle_injector6_mc33810(void)
{
  injector6Toggle_MC33810();
}
#endif

#if INJ_CHANNELS >= 7
static void toggle_injector7_mc33810(void)
{
  injector7Toggle_MC33810();
}
#endif

#if INJ_CHANNELS >= 8
static void toggle_injector8_mc33810(void)
{
  injector8Toggle_MC33810();
}
#endif

static mc33810_injector_control_st const
injector_control_mc33810[injector_id_COUNT] = {
  [injector_id_1] = mc33810_injector_control_st(open_injector1_mc33810, close_injector1_mc33810, toggle_injector1_mc33810),
  [injector_id_2] = mc33810_injector_control_st(open_injector2_mc33810, close_injector2_mc33810, toggle_injector2_mc33810),
  [injector_id_3] = mc33810_injector_control_st(open_injector3_mc33810, close_injector3_mc33810, toggle_injector3_mc33810),
  [injector_id_4] = mc33810_injector_control_st(open_injector4_mc33810, close_injector4_mc33810, toggle_injector4_mc33810),
#if INJ_CHANNELS >= 5
  [injector_id_5] = mc33810_injector_control_st(open_injector5_mc33810, close_injector5_mc33810, toggle_injector5_mc33810),
#endif
#if INJ_CHANNELS >= 6
  [injector_id_6] = mc33810_injector_control_st(open_injector6_mc33810, close_injector6_mc33810, toggle_injector6_mc33810),
#endif
#if INJ_CHANNELS >= 7
  [injector_id_7] = mc33810_injector_control_st(open_injector7_mc33810, close_injector7_mc33810, toggle_injector7_mc33810),
#endif
#if INJ_CHANNELS >= 8
  [injector_id_8] = mc33810_injector_control_st(open_injector8_mc33810, close_injector8_mc33810, toggle_injector8_mc33810),
#endif
};

static void init_mc38810_injectors(void)
{
  initMC33810();
}

static void openInjector(injector_id_t injector)
{
  injector_control_mc33810[injector].open();
}

static void closeInjector(injector_id_t injector)
{
  injector_control_mc33810[injector].close();
}

static void toggleInjector(injector_id_t injector)
{
  injector_control_mc33810[injector].toggle();
}

injectors_st injectors_mc33810 =
{
  .init = init_mc38810_injectors,
  .open = openInjector,
  .close = closeInjector,
  .toggle = toggleInjector,
};
