#include "injector_schedule_direct.h"
#include "globals.h"
#include "injector_pins.h"

typedef void (*open_injector_fn)(void);
typedef void (*close_injector_fn)(void);
typedef void (*toggle_injector_fn)(void);

typedef struct injector_control_st
{
  open_injector_fn open;
  close_injector_fn close;
  toggle_injector_fn toggle;
} injector_control_st;


//Macros are used to define how each injector control system functions. These are then called by the master openInjectx() function.
//The DIRECT macros (ie individual pins) are defined below. Others should be defined in their relevant acc_x.h file
#define openInjector1_DIRECT() { inj1.on(); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ1); }
#define closeInjector1_DIRECT() { inj1.off();  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ1); }
#define injector1Toggle_DIRECT() { inj1.toggle(); }

#define openInjector2_DIRECT() { inj2.on(); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ2); }
#define closeInjector2_DIRECT() { inj2.off();  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ2); }
#define injector2Toggle_DIRECT() { inj2.toggle(); }

#define openInjector3_DIRECT() { inj3.on(); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ3); }
#define closeInjector3_DIRECT() { inj3.off();  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ3); }
#define injector3Toggle_DIRECT() { inj3.toggle(); }

#define openInjector4_DIRECT() { inj4.on(); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ4); }
#define closeInjector4_DIRECT() { inj4.off();  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ4); }
#define injector4Toggle_DIRECT() { inj4.toggle(); }

#if (INJ_CHANNELS >= 5)
#define openInjector5_DIRECT() \
  {                            \
    inj5.on();                 \
  }
#define closeInjector5_DIRECT() { inj5.off(); }
#define injector5Toggle_DIRECT() { inj5.toggle(); }

#endif
#if (INJ_CHANNELS >= 6)
#define openInjector6_DIRECT() \
  {                            \
    inj6.on();                 \
  }
#define closeInjector6_DIRECT() { inj6.off(); }
#define injector6Toggle_DIRECT() { inj6.toggle(); }

#endif
#if (INJ_CHANNELS >= 7)
#define openInjector7_DIRECT() \
  {                            \
    inj7.on();                 \
  }
#define closeInjector7_DIRECT() { inj7.off(); }
#define injector7Toggle_DIRECT(){ inj7.toggle(); }

#endif
#if (INJ_CHANNELS >= 8)
#define openInjector8_DIRECT() \
  {                            \
    inj8.on();                 \
  }
#define closeInjector8_DIRECT() { inj8.off(); }
#define injector8Toggle_DIRECT() { inj8.toggle(); }
#endif

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

#if (INJ_CHANNELS >= 5)
static void open_injector5_direct(void)
{
  openInjector5_DIRECT();
}

#endif
#if (INJ_CHANNELS >= 6)
static void open_injector6_direct(void)
{
  openInjector6_DIRECT();
}

#endif
#if (INJ_CHANNELS >= 7)
static void open_injector7_direct(void)
{
  openInjector7_DIRECT();
}

#endif
#if (INJ_CHANNELS >= 8)
static void open_injector8_direct(void)
{
  openInjector8_DIRECT();
}
#endif

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

#if (INJ_CHANNELS >= 5)
static void close_injector5_direct(void)
{
  closeInjector5_DIRECT();
}

#endif
#if (INJ_CHANNELS >= 6)
static void close_injector6_direct(void)
{
  closeInjector6_DIRECT();
}

#endif
#if (INJ_CHANNELS >= 7)
static void close_injector7_direct(void)
{
  closeInjector7_DIRECT();
}

#endif
#if (INJ_CHANNELS >= 8)
static void close_injector8_direct(void)
{
  closeInjector8_DIRECT();
}
#endif

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

#if INJ_CHANNELS >= 5
static void toggle_injector5_direct(void)
{
  injector5Toggle_DIRECT();
}

#endif
#if INJ_CHANNELS >= 6
static void toggle_injector6_direct(void)
{
  injector6Toggle_DIRECT();
}

#endif
#if INJ_CHANNELS >= 7
static void toggle_injector7_direct(void)
{
  injector7Toggle_DIRECT();
}

#endif
#if INJ_CHANNELS >= 8
static void toggle_injector8_direct(void)
{
  injector8Toggle_DIRECT();
}
#endif

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
#if (INJ_CHANNELS >= 5)
    [injector_id_5] = {
        .open = open_injector5_direct,
        .close = close_injector5_direct,
        .toggle = toggle_injector5_direct,
    },
#endif
#if (INJ_CHANNELS >= 6)
    [injector_id_6] = {
        .open = open_injector6_direct,
        .close = close_injector6_direct,
        .toggle = toggle_injector6_direct,
    },
#endif
#if (INJ_CHANNELS >= 7)
    [injector_id_7] = {
        .open = open_injector7_direct,
        .close = close_injector7_direct,
        .toggle = toggle_injector7_direct,
    },
#endif
#if (INJ_CHANNELS >= 8)
    [injector_id_8] = {
        .open = open_injector8_direct,
        .close = close_injector8_direct,
        .toggle = toggle_injector8_direct,
    }
#endif
};

static void init_direct_injectors(void)
{
  inj1.configure(inj1.pin);
  inj2.configure(inj2.pin);
  inj3.configure(inj3.pin);
  inj4.configure(inj4.pin);
#if (INJ_CHANNELS >= 5)
  inj5.configure(inj5.pin);
#endif
#if (INJ_CHANNELS >= 6)
  inj6.configure(inj6.pin);
#endif
#if (INJ_CHANNELS >= 7)
  inj7.configure(inj7.pin);
#endif
#if (INJ_CHANNELS >= 8)
  inj8.configure(inj8.pin);
#endif
}

static void openInjector(injector_id_t injector)
{
  injector_control_direct[injector].open();
}

static void closeInjector(injector_id_t injector)
{
  injector_control_direct[injector].close();
}

static void toggleInjector(injector_id_t injector)
{
  injector_control_direct[injector].toggle();
}

injectors_st injectors_direct =
{
  .init = init_direct_injectors,
  .open = openInjector,
  .close = closeInjector,
  .toggle = toggleInjector,
};
