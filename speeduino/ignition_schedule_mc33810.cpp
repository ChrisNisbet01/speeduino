#include "ignition_schedule_mc33810.h"
#include "globals.h"
#include "acc_mc33810.h"
#include "bit_macros.h"

typedef void (*ignition_begin_charge_fn)(void);
typedef void (*ignition_end_charge_fn)(void);
typedef void (*ignition_toggle_fn)(void);
typedef struct ignition_control_st
{
  ignition_begin_charge_fn begin_charge;
  ignition_end_charge_fn end_charge;
  ignition_toggle_fn toggle;
} ignition_control_st;


//Set the value of the coil pins to the coilHIGH or coilLOW state
#define coil1Charging_MC33810()      if(configPage4.IgInv == GOING_HIGH) { coil1Low_MC33810();  } else { coil1High_MC33810(); }
#define coil1StopCharging_MC33810()  if(configPage4.IgInv == GOING_HIGH) { coil1High_MC33810(); } else { coil1Low_MC33810();  }
#define coil2Charging_MC33810()      if(configPage4.IgInv == GOING_HIGH) { coil2Low_MC33810();  } else { coil2High_MC33810(); }
#define coil2StopCharging_MC33810()  if(configPage4.IgInv == GOING_HIGH) { coil2High_MC33810(); } else { coil2Low_MC33810();  }
#define coil3Charging_MC33810()      if(configPage4.IgInv == GOING_HIGH) { coil3Low_MC33810();  } else { coil3High_MC33810(); }
#define coil3StopCharging_MC33810()  if(configPage4.IgInv == GOING_HIGH) { coil3High_MC33810(); } else { coil3Low_MC33810();  }
#define coil4Charging_MC33810()      if(configPage4.IgInv == GOING_HIGH) { coil4Low_MC33810();  } else { coil4High_MC33810(); }
#define coil4StopCharging_MC33810()  if(configPage4.IgInv == GOING_HIGH) { coil4High_MC33810(); } else { coil4Low_MC33810();  }
#if IGN_CHANNELS >= 5
#define coil5Charging_MC33810()      if(configPage4.IgInv == GOING_HIGH) { coil5Low_MC33810();  } else { coil5High_MC33810(); }
#define coil5StopCharging_MC33810()  if(configPage4.IgInv == GOING_HIGH) { coil5High_MC33810(); } else { coil5Low_MC33810();  }
#endif
#if IGN_CHANNELS >= 6
#define coil6Charging_MC33810()      if(configPage4.IgInv == GOING_HIGH) { coil6Low_MC33810();  } else { coil6High_MC33810(); }
#define coil6StopCharging_MC33810()  if(configPage4.IgInv == GOING_HIGH) { coil6High_MC33810(); } else { coil6Low_MC33810();  }
#endif
#if IGN_CHANNELS >= 7
#define coil7Charging_MC33810()      if(configPage4.IgInv == GOING_HIGH) { coil7Low_MC33810();  } else { coil7High_MC33810(); }
#define coil7StopCharging_MC33810()  if(configPage4.IgInv == GOING_HIGH) { coil7High_MC33810(); } else { coil7Low_MC33810();  }
#endif
#if IGN_CHANNELS >= 6
#define coil8Charging_MC33810()      if(configPage4.IgInv == GOING_HIGH) { coil8Low_MC33810();  } else { coil8High_MC33810(); }
#define coil8StopCharging_MC33810()  if(configPage4.IgInv == GOING_HIGH) { coil8High_MC33810(); } else { coil8Low_MC33810();  }
#endif

void coil1Toggle_mc33810(void)
{
  coil1Toggle_MC33810();
}

void coil2Toggle_mc33810(void)
{
  coil2Toggle_MC33810();
}

void coil3Toggle_mc33810(void)
{
  coil3Toggle_MC33810();
}

void coil4Toggle_mc33810(void)
{
  coil4Toggle_MC33810();
}

#if IGN_CHANNELS >= 5
void coil5Toggle_mc33810(void)
{
  coil5Toggle_MC33810();
}
#endif

#if IGN_CHANNELS >= 6
void coil6Toggle_mc33810(void)
{
  coil6Toggle_MC33810();
}
#endif

#if IGN_CHANNELS >= 7
void coil7Toggle_mc33810(void)
{
  coil7Toggle_MC33810();
}
#endif

#if IGN_CHANNELS >= 8
void coil8Toggle_mc33810(void)
{
  coil8Toggle_MC33810();
}
#endif

void beginCoil1Charge_mc33810(void)
{
  coil1Charging_MC33810();
}

void endCoil1Charge_mc33810(void)
{
  coil1StopCharging_MC33810();
}

void beginCoil2Charge_mc33810(void)
{
  coil2Charging_MC33810();
}

void endCoil2Charge_mc33810(void)
{
  coil2StopCharging_MC33810();
}

void beginCoil3Charge_mc33810(void)
{
  coil3Charging_MC33810();
}

void endCoil3Charge_mc33810(void)
{
  coil3StopCharging_MC33810();
}

void beginCoil4Charge_mc33810(void)
{
  coil4Charging_MC33810();
}

void endCoil4Charge_mc33810(void)
{
  coil4StopCharging_MC33810();
}

#if IGN_CHANNELS >= 5
void beginCoil5Charge_mc33810(void)
{
  coil5Charging_MC33810();
}

void endCoil5Charge_mc33810(void)
{
  coil5StopCharging_MC33810();
}
#endif

#if IGN_CHANNELS >= 6
void beginCoil6Charge_mc33810(void)
{
  coil6Charging_MC33810();
}

void endCoil6Charge_mc33810(void)
{
  coil6StopCharging_MC33810();
}
#endif

#if IGN_CHANNELS >= 7
void beginCoil7Charge_mc33810(void)
{
  coil7Charging_MC33810();
}

void endCoil7Charge_mc33810(void)
{
  coil7StopCharging_MC33810();
}
#endif

#if IGN_CHANNELS >= 8
void beginCoil8Charge_mc33810(void)
{
  coil8Charging_MC33810();
}

void endCoil8Charge_mc33810(void)
{
  coil8StopCharging_MC33810();
}
#endif

static ignition_control_st const ignition_control_mc33810[ignition_id_COUNT] = {
  [ignition_id_1] = {
    .begin_charge = beginCoil1Charge_mc33810,
    .end_charge = endCoil1Charge_mc33810,
    .toggle = coil1Toggle_mc33810,
  },
  [ignition_id_2] = {
    .begin_charge = beginCoil2Charge_mc33810,
    .end_charge = endCoil2Charge_mc33810,
    .toggle = coil2Toggle_mc33810,
  },
  [ignition_id_3] = {
    .begin_charge = beginCoil3Charge_mc33810,
    .end_charge = endCoil3Charge_mc33810,
    .toggle = coil3Toggle_mc33810,
  },
  [ignition_id_4] = {
    .begin_charge = beginCoil4Charge_mc33810,
    .end_charge = endCoil4Charge_mc33810,
    .toggle = coil4Toggle_mc33810,
  },
#if IGN_CHANNELS >= 5
  [ignition_id_5] = {
    .begin_charge = beginCoil5Charge_mc33810,
    .end_charge = endCoil5Charge_mc33810,
    .toggle = coil5Toggle_mc33810,
  },
#endif
#if IGN_CHANNELS >= 6
  [ignition_id_6] = {
    .begin_charge = beginCoil6Charge_mc33810,
    .end_charge = endCoil6Charge_mc33810,
    .toggle = coil6Toggle_mc33810,
  },
#endif
#if IGN_CHANNELS >= 7
  [ignition_id_7] = {
    .begin_charge = beginCoil7Charge_mc33810,
    .end_charge = endCoil7Charge_mc33810,
    .toggle = coil7Toggle_mc33810,
  },
#endif
#if IGN_CHANNELS >= 8
  [ignition_id_8] = {
    .begin_charge = beginCoil8Charge_mc33810,
    .end_charge = endCoil8Charge_mc33810,
    .toggle = coil8Toggle_mc33810,
  }
#endif
};

static void init_mc33810_ignition(void)
{
  initMC33810();
}

static void coil_begin_charge(ignition_id_t coil)
{
  ignition_control_mc33810[coil].begin_charge();
}

static void coil_end_charge(ignition_id_t coil)
{
  ignition_control_mc33810[coil].end_charge();
}

static void coil_toggle(ignition_id_t coil)
{
  ignition_control_mc33810[coil].toggle();
}

ignition_st const ignition_mc33810 =
{
  .init = init_mc33810_ignition,
  .begin_charge = coil_begin_charge,
  .end_charge = coil_end_charge,
  .toggle = coil_toggle,
};
