#include <unity.h>
#include "globals.h"
#include "init.h"
#include "../test_utils.h"
#include "storage.h"
#include "injector_pins.h"
#include "ignition_pins.h"

void prepareForInitialiseAll(uint8_t boardId);

#define UNKNOWN_PIN 0xFF

#if !defined(NOT_A_PIN)
#define NOT_A_PIN 0
#endif

uint8_t getPinMode(uint8_t pin)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);

  // I don't see an option for mega to return this, but whatever...
  if (NOT_A_PIN == port) return UNKNOWN_PIN;

  // Is there a bit we can check?
  if (0 == bit) return UNKNOWN_PIN;

  // Is there only a single bit set?
  if (bit & (bit - 1)) return UNKNOWN_PIN;

  volatile uint8_t *reg, *out;
  reg = portModeRegister(port);
  out = portOutputRegister(port);

  if (*reg & bit)
    return OUTPUT;
  else if (*out & bit)
    return INPUT_PULLUP;
  else
    return INPUT;
}

void test_initialisation_complete(void)
{
  prepareForInitialiseAll(3);
  initialiseAll(); //Run the main initialise function
  TEST_ASSERT_EQUAL(true, currentStatus.initialisationComplete);
}

void test_initialisation_ports(void)
{
  //Test that all the port values have been set
  prepareForInitialiseAll(3);
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_TRUE(inj1.is_configured());
  TEST_ASSERT_TRUE(inj2.is_configured());
  TEST_ASSERT_TRUE(inj3.is_configured());
  TEST_ASSERT_TRUE(inj4.is_configured());

  TEST_ASSERT_TRUE(ign1.is_configured());
  TEST_ASSERT_TRUE(ign2.is_configured());
  TEST_ASSERT_TRUE(ign3.is_configured());
  TEST_ASSERT_TRUE(ign4.is_configured());
}

//Test that all mandatory output pins have their mode correctly set to output
void test_initialisation_outputs_V03(void)
{
  prepareForInitialiseAll(2);
  initialiseAll(); //Run the main initialise function

  char msg[32];
  strcpy_P(msg, PSTR("Coil1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign1.pin), msg);
  strcpy_P(msg, PSTR("Coil2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign2.pin), msg);
  strcpy_P(msg, PSTR("Coil3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign3.pin), msg);
  strcpy_P(msg, PSTR("Coil4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign4.pin), msg);
  strcpy_P(msg, PSTR("Injector 1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj1.pin), msg);
  strcpy_P(msg, PSTR("Injector 2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj2.pin), msg);
  strcpy_P(msg, PSTR("Injector 3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj3.pin), msg);
  strcpy_P(msg, PSTR("Injector 4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj4.pin), msg);
  strcpy_P(msg, PSTR("Tacho Out"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(TachOut.pin), msg);
  strcpy_P(msg, PSTR("Fuel Pump"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(FuelPump.pin), msg);
  strcpy_P(msg, PSTR("Fan"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Fan.pin), msg);
}

//Test that all mandatory output pins have their mode correctly set to output
void test_initialisation_outputs_V04(void)
{
  prepareForInitialiseAll(3);
  initialiseAll(); //Run the main initialise function

  char msg[32];
  strcpy_P(msg, PSTR("Coil1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign1.pin), msg);
  strcpy_P(msg, PSTR("Coil2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign2.pin), msg);
  strcpy_P(msg, PSTR("Coil3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign3.pin), msg);
  strcpy_P(msg, PSTR("Coil4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign4.pin), msg);
  strcpy_P(msg, PSTR("Injector 1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj1.pin), msg);
  strcpy_P(msg, PSTR("Injector 2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj2.pin), msg);
  strcpy_P(msg, PSTR("Injector 3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj3.pin), msg);
  strcpy_P(msg, PSTR("Injector 4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj4.pin), msg);
  strcpy_P(msg, PSTR("Tacho Out"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(TachOut.pin), msg);
  strcpy_P(msg, PSTR("Fuel Pump"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(FuelPump.pin), msg);
  strcpy_P(msg, PSTR("Fan"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Fan.pin), msg);
  /*
  if(isIdlePWM)
  {
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Idle1.pin), "Idle 1");
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Idle2.pin), "Idle 2");
  }
  else if (isIdleStepper)
  {
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(StepperDir.pin), "Stepper Dir");
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(StepperStep.pin), "Stepper Step");
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(StepperEnable.pin), "Stepper Enable");
  }

  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Fan.pin), "Fan");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Boost.pin), "Boost");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(VVT_1.pin), "VVT1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(VVT_2.pin), "VVT2");
  */

}

//Test that all mandatory output pins have their mode correctly set to output
void test_initialisation_outputs_MX5_8995(void)
{
  prepareForInitialiseAll(9);
  initialiseAll(); //Run the main initialise function

  char msg[32];
  strcpy_P(msg, PSTR("Coil1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign1.pin), msg);
  strcpy_P(msg, PSTR("Coil2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign2.pin), msg);
  strcpy_P(msg, PSTR("Coil3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign3.pin), msg);
  strcpy_P(msg, PSTR("Coil4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(ign4.pin), msg);
  strcpy_P(msg, PSTR("Injector 1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj1.pin), msg);
  strcpy_P(msg, PSTR("Injector 2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj2.pin), msg);
  strcpy_P(msg, PSTR("Injector 3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj3.pin), msg);
  strcpy_P(msg, PSTR("Injector 4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(inj4.pin), msg);
  strcpy_P(msg, PSTR("Tacho Out"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(TachOut.pin), msg);
  strcpy_P(msg, PSTR("Fuel Pump"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(FuelPump.pin), msg);
  strcpy_P(msg, PSTR("Fan"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Fan.pin), msg);
}

void test_initialisation_outputs_PWM_idle(void)
{
  prepareForInitialiseAll(3);

  //Force 2 channel PWM idle
  configPage6.iacChannels = 1;
  configPage6.iacAlgorithm = 2;

  initialiseAll(); //Run the main initialise function

  bool isIdlePWM = (configPage6.iacAlgorithm > 0) && ((configPage6.iacAlgorithm <= 3) || (configPage6.iacAlgorithm == 6));

  char msg[32];
  strcpy_P(msg, PSTR("Is PWM Idle"));
  TEST_ASSERT_TRUE_MESSAGE(isIdlePWM, msg);
  strcpy_P(msg, PSTR("Idle 1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Idle1.pin), msg);
  strcpy_P(msg, PSTR("Idle 2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Idle2.pin), msg);
}

void test_initialisation_outputs_stepper_idle(void)
{
  prepareForInitialiseAll(9);
  bool isIdleStepper = (configPage6.iacAlgorithm > 3) && (configPage6.iacAlgorithm != 6);
  initialiseAll(); //Run the main initialise function

  char msg[32];
  strcpy_P(msg, PSTR("Is Stepper Idle"));
  TEST_ASSERT_TRUE_MESSAGE(isIdleStepper, msg);
  strcpy_P(msg, PSTR("Stepper Dir"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(StepperDir.pin), msg);
  strcpy_P(msg, PSTR("Stepper Step"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(StepperStep.pin), msg);
  strcpy_P(msg, PSTR("Stepper Enable"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(StepperEnable.pin), msg);
}

void test_initialisation_outputs_boost(void)
{
  prepareForInitialiseAll(9);
  initialiseAll(); //Run the main initialise function

  char msg[32];
  strcpy_P(msg, PSTR("Boost"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(Boost.pin), msg);
}

void test_initialisation_outputs_VVT(void)
{
  prepareForInitialiseAll(9);
  initialiseAll(); //Run the main initialise function

  char msg[32];
  strcpy_P(msg, PSTR("VVT1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(VVT_1.pin), msg);
  strcpy_P(msg, PSTR("VVT2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(VVT_2.pin), msg);
}

void test_initialisation_outputs_reset_control_use_board_default(void)
{
  prepareForInitialiseAll(9);
  configPage4.resetControlConfig = RESET_CONTROL_PREVENT_WHEN_RUNNING;
  configPage4.resetControlPin = 0; // Flags to use board default
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_NOT_EQUAL(0, ResetControl.pin);
  TEST_ASSERT_EQUAL(resetControl, RESET_CONTROL_PREVENT_WHEN_RUNNING);
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(ResetControl.pin));
}

void test_initialisation_outputs_reset_control_override_board_default(void)
{
  prepareForInitialiseAll(9);
  configPage4.resetControlConfig = RESET_CONTROL_PREVENT_WHEN_RUNNING;
  configPage4.resetControlPin = 45; // Use a different pin
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL(45, ResetControl.pin);
  TEST_ASSERT_EQUAL(resetControl, RESET_CONTROL_PREVENT_WHEN_RUNNING);
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(ResetControl.pin));
}

void test_initialisation_user_pin_override_board_default(void)
{
  prepareForInitialiseAll(3);
  // We do not test all pins, too many & too fragile. So fingers crossed the
  // same pattern is used for all.
  configPage2.tachoPin = 15;
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL(15, TachOut.pin);
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(TachOut.pin));
}

// All config user pin fields are <= 6 *bits*. So too small to
// assign BOARD_MAX_IO_PINS to. So while there is defensive code
// in place, it cannot be unit tested.
#if false
void test_initialisation_user_pin_not_valid_no_override(void)
{
  prepareForInitialiseAll(3);
  configPage2.tachoPin = (uint8_t)BOARD_MAX_IO_PINS;// + (uint8_t)1U;
  ++configPage2.tachoPin;
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL(49, TachOut.pin);
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(TachOut.pin));
}
#endif

void test_initialisation_input_user_pin_does_not_override_outputpin(void)
{
  // A user definable input pin should not overwrite any output pins.
  prepareForInitialiseAll(3);
  configPage6.launchPin = 49; // 49 is the default tacho output
  initialiseAll(); //Run the main initialise function

  TEST_ASSERT_EQUAL(49, TachOut.pin);
  TEST_ASSERT_EQUAL(OUTPUT, getPinMode(TachOut.pin));
  TEST_ASSERT_EQUAL(49, Launch.pin);
}

void testInitialisation()
{
  RUN_TEST_P(test_initialisation_complete);
  RUN_TEST_P(test_initialisation_ports);
  RUN_TEST_P(test_initialisation_outputs_V03);
  RUN_TEST_P(test_initialisation_outputs_V04);
  RUN_TEST_P(test_initialisation_outputs_MX5_8995);
  RUN_TEST_P(test_initialisation_outputs_PWM_idle);
  RUN_TEST_P(test_initialisation_outputs_boost);
  RUN_TEST_P(test_initialisation_outputs_VVT);
  RUN_TEST_P(test_initialisation_outputs_reset_control_use_board_default);
  RUN_TEST_P(test_initialisation_outputs_reset_control_override_board_default);
  RUN_TEST_P(test_initialisation_user_pin_override_board_default);
  RUN_TEST_P(test_initialisation_input_user_pin_does_not_override_outputpin);
}