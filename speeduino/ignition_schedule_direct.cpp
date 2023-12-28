#include "ignition_schedule_direct.h"
#include "globals.h"

typedef void (*ignition_begin_charge_fn)(void);
typedef void (*ignition_end_charge_fn)(void);
typedef void (*ignition_toggle_fn)(void);
typedef struct ignition_control_st
{
  ignition_begin_charge_fn begin_charge;
  ignition_end_charge_fn end_charge;
  ignition_toggle_fn toggle;
} ignition_control_st;


#define coil1Low_DIRECT()       (*ign1_pin_port &= ~(ign1_pin_mask))
#define coil1High_DIRECT()      (*ign1_pin_port |= (ign1_pin_mask))
#define coil1Toggle_DIRECT() (*ign1_pin_port ^= ign1_pin_mask )

#define coil2Low_DIRECT()       (*ign2_pin_port &= ~(ign2_pin_mask))
#define coil2High_DIRECT()      (*ign2_pin_port |= (ign2_pin_mask))
#define coil2Toggle_DIRECT() (*ign2_pin_port ^= ign2_pin_mask )

#define coil3Low_DIRECT()       (*ign3_pin_port &= ~(ign3_pin_mask))
#define coil3High_DIRECT()      (*ign3_pin_port |= (ign3_pin_mask))
#define coil3Toggle_DIRECT() (*ign3_pin_port ^= ign3_pin_mask )

#define coil4Low_DIRECT()       (*ign4_pin_port &= ~(ign4_pin_mask))
#define coil4High_DIRECT()      (*ign4_pin_port |= (ign4_pin_mask))
#define coil4Toggle_DIRECT() (*ign4_pin_port ^= ign4_pin_mask )

#define coil5Low_DIRECT()       (*ign5_pin_port &= ~(ign5_pin_mask))
#define coil5High_DIRECT()      (*ign5_pin_port |= (ign5_pin_mask))
#define coil5Toggle_DIRECT() (*ign5_pin_port ^= ign5_pin_mask )

#define coil6Low_DIRECT()       (*ign6_pin_port &= ~(ign6_pin_mask))
#define coil6High_DIRECT()      (*ign6_pin_port |= (ign6_pin_mask))
#define coil6Toggle_DIRECT() (*ign6_pin_port ^= ign6_pin_mask )

#define coil7Low_DIRECT()       (*ign7_pin_port &= ~(ign7_pin_mask))
#define coil7High_DIRECT()      (*ign7_pin_port |= (ign7_pin_mask))
#define coil7Toggle_DIRECT() (*ign7_pin_port ^= ign7_pin_mask )

#define coil8Low_DIRECT()       (*ign8_pin_port &= ~(ign8_pin_mask))
#define coil8High_DIRECT()      (*ign8_pin_port |= (ign8_pin_mask))
#define coil8Toggle_DIRECT() (*ign8_pin_port ^= ign8_pin_mask )

//Set the value of the coil pins to the coilHIGH or coilLOW state
#define coil1Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil1Low_DIRECT() : coil1High_DIRECT())
#define coil1StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil1High_DIRECT() : coil1Low_DIRECT())
#define coil2Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil2Low_DIRECT() : coil2High_DIRECT())
#define coil2StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil2High_DIRECT() : coil2Low_DIRECT())
#define coil3Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil3Low_DIRECT() : coil3High_DIRECT())
#define coil3StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil3High_DIRECT() : coil3Low_DIRECT())
#define coil4Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil4Low_DIRECT() : coil4High_DIRECT())
#define coil4StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil4High_DIRECT() : coil4Low_DIRECT())
#define coil5Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil5Low_DIRECT() : coil5High_DIRECT())
#define coil5StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil5High_DIRECT() : coil5Low_DIRECT())
#define coil6Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil6Low_DIRECT() : coil6High_DIRECT())
#define coil6StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil6High_DIRECT() : coil6Low_DIRECT())
#define coil7Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil7Low_DIRECT() : coil7High_DIRECT())
#define coil7StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil7High_DIRECT() : coil7Low_DIRECT())
#define coil8Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil8Low_DIRECT() : coil8High_DIRECT())
#define coil8StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil8High_DIRECT() : coil8Low_DIRECT())

void coil1Toggle_direct(void)
{
    coil1Toggle_DIRECT();
}
void coil2Toggle_direct(void)
{
    coil2Toggle_DIRECT();
}
void coil3Toggle_direct(void)
{
    coil3Toggle_DIRECT();
}
void coil4Toggle_direct(void)
{
    coil4Toggle_DIRECT();
}
void coil5Toggle_direct(void)
{
    coil5Toggle_DIRECT();
}
void coil6Toggle_direct(void)
{
    coil6Toggle_DIRECT();
}
void coil7Toggle_direct(void)
{
    coil7Toggle_DIRECT();
}
void coil8Toggle_direct(void)
{
    coil8Toggle_DIRECT();
}

void beginCoil1Charge_direct(void)
{
    coil1Charging_DIRECT();
}
void endCoil1Charge_direct(void)
{
    coil1StopCharging_DIRECT();
}

void beginCoil2Charge_direct(void)
{
    coil2Charging_DIRECT();
}
void endCoil2Charge_direct(void)
{
    coil2StopCharging_DIRECT();
}

void beginCoil3Charge_direct(void)
{
    coil3Charging_DIRECT();
}
void endCoil3Charge_direct(void)
{
    coil3StopCharging_DIRECT();
}

void beginCoil4Charge_direct(void)
{
    coil4Charging_DIRECT();
}
void endCoil4Charge_direct(void)
{
    coil4StopCharging_DIRECT();
}

void beginCoil5Charge_direct(void)
{
    coil5Charging_DIRECT();
}
void endCoil5Charge_direct(void)
{
    coil5StopCharging_DIRECT();
}

void beginCoil6Charge_direct(void)
{
    coil6Charging_DIRECT();
}
void endCoil6Charge_direct(void)
{
    coil6StopCharging_DIRECT();
}

void beginCoil7Charge_direct(void)
{
    coil7Charging_DIRECT();
}
void endCoil7Charge_direct(void)
{
    coil7StopCharging_DIRECT();
}

void beginCoil8Charge_direct(void)
{
    coil8Charging_DIRECT();
}
void endCoil8Charge_direct(void)
{
  coil8StopCharging_DIRECT();
}

static ignition_control_st const ignition_control_direct[ignition_id_COUNT] = {
  [ignition_id_1] = {
    .begin_charge = beginCoil1Charge_direct,
    .end_charge = endCoil1Charge_direct,
    .toggle = coil1Toggle_direct,
  },
  [ignition_id_2] = {
    .begin_charge = beginCoil2Charge_direct,
    .end_charge = endCoil2Charge_direct,
    .toggle = coil2Toggle_direct,
  },
  [ignition_id_3] = {
    .begin_charge = beginCoil3Charge_direct,
    .end_charge = endCoil3Charge_direct,
    .toggle = coil3Toggle_direct,
  },
  [ignition_id_4] = {
    .begin_charge = beginCoil4Charge_direct,
    .end_charge = endCoil4Charge_direct,
    .toggle = coil4Toggle_direct,
  },
  [ignition_id_5] = {
    .begin_charge = beginCoil5Charge_direct,
    .end_charge = endCoil5Charge_direct,
    .toggle = coil5Toggle_direct,
  },
  [ignition_id_6] = {
    .begin_charge = beginCoil6Charge_direct,
    .end_charge = endCoil6Charge_direct,
    .toggle = coil6Toggle_direct,
  },
  [ignition_id_7] = {
    .begin_charge = beginCoil7Charge_direct,
    .end_charge = endCoil7Charge_direct,
    .toggle = coil7Toggle_direct,
  },
  [ignition_id_8] = {
    .begin_charge = beginCoil8Charge_direct,
    .end_charge = endCoil8Charge_direct,
    .toggle = coil8Toggle_direct,
  }
};

static void init_direct_ignition(void)
{
  pinMode(pinCoil1, OUTPUT);
  pinMode(pinCoil2, OUTPUT);
  pinMode(pinCoil3, OUTPUT);
  pinMode(pinCoil4, OUTPUT);
  #if (IGN_CHANNELS >= 5)
  pinMode(pinCoil5, OUTPUT);
  #endif
  #if (IGN_CHANNELS >= 6)
  pinMode(pinCoil6, OUTPUT);
  #endif
  #if (IGN_CHANNELS >= 7)
  pinMode(pinCoil7, OUTPUT);
  #endif
  #if (IGN_CHANNELS >= 8)
  pinMode(pinCoil8, OUTPUT);
  #endif

  ign1_pin_port = portOutputRegister(digitalPinToPort(pinCoil1));
  ign1_pin_mask = digitalPinToBitMask(pinCoil1);
  ign2_pin_port = portOutputRegister(digitalPinToPort(pinCoil2));
  ign2_pin_mask = digitalPinToBitMask(pinCoil2);
  ign3_pin_port = portOutputRegister(digitalPinToPort(pinCoil3));
  ign3_pin_mask = digitalPinToBitMask(pinCoil3);
  ign4_pin_port = portOutputRegister(digitalPinToPort(pinCoil4));
  ign4_pin_mask = digitalPinToBitMask(pinCoil4);
  ign5_pin_port = portOutputRegister(digitalPinToPort(pinCoil5));
  ign5_pin_mask = digitalPinToBitMask(pinCoil5);
  ign6_pin_port = portOutputRegister(digitalPinToPort(pinCoil6));
  ign6_pin_mask = digitalPinToBitMask(pinCoil6);
  ign7_pin_port = portOutputRegister(digitalPinToPort(pinCoil7));
  ign7_pin_mask = digitalPinToBitMask(pinCoil7);
  ign8_pin_port = portOutputRegister(digitalPinToPort(pinCoil8));
  ign8_pin_mask = digitalPinToBitMask(pinCoil8);
}

static void coil_begin_charge(ignition_id_t coil)
{
  ignition_control_direct[coil].begin_charge();
}

static void coil_end_charge(ignition_id_t coil)
{
  ignition_control_direct[coil].end_charge();
}

static void coil_toggle(ignition_id_t coil)
{
  ignition_control_direct[coil].toggle();
}

ignition_st const ignition_direct =
{
  .init = init_direct_ignition,
  .begin_charge = coil_begin_charge,
  .end_charge = coil_end_charge,
  .toggle = coil_toggle,
};

