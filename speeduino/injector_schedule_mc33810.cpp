#include "injector_schedule_mc33810.h"
#include "acc_mc33810.h"

typedef void (*open_injector_fn)(void);
typedef void (*close_injector_fn)(void);
typedef void (*toggle_injector_fn)(void);

typedef struct injector_control_st
{
  open_injector_fn open;
  close_injector_fn close;
  toggle_injector_fn toggle;
} injector_control_st;

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

static injector_control_st const injector_control_mc33810[injector_id_COUNT] = {
    [injector_id_1] = {
        .open = open_injector1_mc33810,
        .close = close_injector1_mc33810,
        .toggle = toggle_injector1_mc33810,
    },
    [injector_id_2] = {
        .open = open_injector2_mc33810,
        .close = close_injector2_mc33810,
        .toggle = toggle_injector2_mc33810,
    },
    [injector_id_3] = {
        .open = open_injector3_mc33810,
        .close = close_injector3_mc33810,
        .toggle = toggle_injector3_mc33810,
    },
    [injector_id_4] = {
        .open = open_injector4_mc33810,
        .close = close_injector4_mc33810,
        .toggle = toggle_injector4_mc33810,
    },
#if INJ_CHANNELS >= 5
    [injector_id_5] = {
        .open = open_injector5_mc33810,
        .close = close_injector5_mc33810,
        .toggle = toggle_injector5_mc33810,
    },
#endif
#if INJ_CHANNELS >= 6
    [injector_id_6] = {
        .open = open_injector6_mc33810,
        .close = close_injector6_mc33810,
        .toggle = toggle_injector6_mc33810,
    },
#endif
#if INJ_CHANNELS >= 7
    [injector_id_7] = {
        .open = open_injector7_mc33810,
        .close = close_injector7_mc33810,
        .toggle = toggle_injector7_mc33810,
    },
#endif
#if INJ_CHANNELS >= 8
    [injector_id_8] = {
        .open = open_injector8_mc33810,
        .close = close_injector8_mc33810,
        .toggle = toggle_injector8_mc33810,
    }
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
