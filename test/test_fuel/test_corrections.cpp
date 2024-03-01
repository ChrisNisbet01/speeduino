#include <globals.h>
#include <corrections.h>
#include <unity.h>
#include "test_corrections.h"
#include "tps_dot.h"
#include "bit_macros.h"

void testCorrections()
{
  test_corrections_WUE();
  test_corrections_dfco();
  test_corrections_TAE(); //TPS based accel enrichment corrections
  /*
  RUN_TEST(test_corrections_cranking); //Not written yet
  RUN_TEST(test_corrections_ASE); //Not written yet
  RUN_TEST(test_corrections_floodclear); //Not written yet
  RUN_TEST(test_corrections_closedloop); //Not written yet
  RUN_TEST(test_corrections_flex); //Not written yet
  RUN_TEST(test_corrections_bat); //Not written yet
  RUN_TEST(test_corrections_iatdensity); //Not written yet
  RUN_TEST(test_corrections_baro); //Not written yet
  RUN_TEST(test_corrections_launch); //Not written yet
  RUN_TEST(test_corrections_dfco); //Not written yet
  */
}

void test_corrections_WUE_active(void)
{
  //Check for WUE being active
  currentStatus.coolant = 0;
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP);

  ((uint8_t*)WUETable.axisX)[9] = 120 + CALIBRATION_TEMPERATURE_OFFSET; //Set a WUE end value of 120
  correctionWUE();
  TEST_ASSERT_BIT_HIGH(BIT_ENGINE_WARMUP, currentStatus.engine);
}

void test_corrections_WUE_inactive(void)
{
  //Check for WUE being inactive due to the temp being too high
  currentStatus.coolant = 200;
  ((uint8_t*)WUETable.axisX)[9] = 120 + CALIBRATION_TEMPERATURE_OFFSET; //Set a WUE end value of 120
  correctionWUE();
  TEST_ASSERT_BIT_LOW(BIT_ENGINE_WARMUP, currentStatus.engine);
}

void test_corrections_WUE_inactive_value(void)
{
  //Check for WUE being set to the final row of the WUE curve if the coolant is above the max WUE temp
  currentStatus.coolant = 200;
  ((uint8_t*)WUETable.axisX)[9] = 100;
  ((uint8_t*)WUETable.values)[9] = 123; //Use a value other than 100 here to ensure we are using the non-default value

  //Force invalidate the cache
  WUETable.cacheTime = currentStatus.secl - 1;

  TEST_ASSERT_EQUAL(123, correctionWUE() );
}

void test_corrections_WUE_active_value(void)
{
  //Check for WUE being made active and returning a correct interpolated value
  currentStatus.coolant = 80;
  //Set some fake values in the table axis. Target value will fall between points 6 and 7
  ((uint8_t*)WUETable.axisX)[0] = 0;
  ((uint8_t*)WUETable.axisX)[1] = 0;
  ((uint8_t*)WUETable.axisX)[2] = 0;
  ((uint8_t*)WUETable.axisX)[3] = 0;
  ((uint8_t*)WUETable.axisX)[4] = 0;
  ((uint8_t*)WUETable.axisX)[5] = 0;
  ((uint8_t*)WUETable.axisX)[6] = 70 + CALIBRATION_TEMPERATURE_OFFSET;
  ((uint8_t*)WUETable.axisX)[7] = 90 + CALIBRATION_TEMPERATURE_OFFSET;
  ((uint8_t*)WUETable.axisX)[8] = 100 + CALIBRATION_TEMPERATURE_OFFSET;
  ((uint8_t*)WUETable.axisX)[9] = 120 + CALIBRATION_TEMPERATURE_OFFSET;

  ((uint8_t*)WUETable.values)[6] = 120;
  ((uint8_t*)WUETable.values)[7] = 130;

  //Force invalidate the cache
  WUETable.cacheTime = currentStatus.secl - 1;

  //Value should be midway between 120 and 130 = 125
  TEST_ASSERT_EQUAL(125, correctionWUE() );
}

void test_corrections_WUE(void)
{
  RUN_TEST(test_corrections_WUE_active);
  RUN_TEST(test_corrections_WUE_inactive);
  RUN_TEST(test_corrections_WUE_active_value);
  RUN_TEST(test_corrections_WUE_inactive_value);
}
void test_corrections_cranking(void)
{

}
void test_corrections_ASE(void)
{

}
void test_corrections_floodclear(void)
{

}
void test_corrections_closedloop(void)
{

}
void test_corrections_flex(void)
{

}
void test_corrections_bat(void)
{

}
void test_corrections_iatdensity(void)
{

}
void test_corrections_baro(void)
{

}
void test_corrections_launch(void)
{

}

void setup_DFCO_on()
{
  //Sets all the required conditions to have the DFCO be active
  configPage2.dfcoEnabled = 1; //Ensure DFCO option is turned on
  currentStatus.RPM = 4000; //Set the current simulated RPM to a level above the DFCO rpm threshold
  currentStatus.TPS = 0; //Set the simulated TPS to 0
  currentStatus.coolant = 80;
  configPage4.dfcoRPM = 150; //DFCO enable RPM = 1500
  configPage4.dfcoTPSThresh = 1;
  configPage4.dfcoHyster = 25;
  configPage2.dfcoMinCLT = 40; //Actually 0 with offset
  configPage2.dfcoDelay = 10;

  dfcoDelay = 1;
  correctionDFCO();
  dfcoDelay = 20;
}
//**********************************************************************************************************************
void test_corrections_dfco_on(void)
{
  //Test under ideal conditions that DFCO goes active
  setup_DFCO_on();

  TEST_ASSERT_TRUE(correctionDFCO());
}
void test_corrections_dfco_off_RPM()
{
  //Test that DFCO comes on and then goes off when the RPM drops below threshold
  setup_DFCO_on();

  TEST_ASSERT_TRUE(correctionDFCO()); //Make sure DFCO is on initially
  currentStatus.RPM = 1000; //Set the current simulated RPM below the threshold + hyster
  TEST_ASSERT_FALSE(correctionDFCO()); //Test DFCO is now off
}
void test_corrections_dfco_off_TPS()
{
  //Test that DFCO comes on and then goes off when the TPS goes above the required threshold (ie not off throttle)
  setup_DFCO_on();

  TEST_ASSERT_TRUE(correctionDFCO()); //Make sure DFCO is on initially
  currentStatus.TPS = 10; //Set the current simulated TPS to be above the threshold
  TEST_ASSERT_FALSE(correctionDFCO()); //Test DFCO is now off
}
void test_corrections_dfco_off_delay()
{
  //Test that DFCO comes will not activate if there has not been a long enough delay
  //The steup function below simulates a 2 second delay
  setup_DFCO_on();

  //Set the threshold to be 2.5 seconds, above the simulated delay of 2s
  configPage2.dfcoDelay = 250;

  TEST_ASSERT_FALSE(correctionDFCO()); //Make sure DFCO does not come on
}
void setup_DFCO_taper_on()
{
  //Test that DFCO comes will not activate if there has not been a long enough delay
  //The steup function below simulates a 2 second delay
  setup_DFCO_on();

  configPage9.dfcoTaperEnable = 1; //Enable
  configPage9.dfcoTaperTime = 20; //2.0 second
  configPage9.dfcoTaperFuel = 0; //Scale fuel to 0%
  configPage9.dfcoTaperAdvance = 20; //Reduce 20deg until full fuel cut

  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_DFCO);
  //Set the threshold to be 2.5 seconds, above the simulated delay of 2s
  configPage2.dfcoDelay = 250;
}
void test_corrections_dfco_taper()
{
  setup_DFCO_taper_on();

  TEST_ASSERT_FALSE(correctionDFCO()); //Make sure DFCO does not come on
  correctionDFCOfuel();
  TEST_ASSERT_EQUAL(20, dfcoTaper); //Check if value was reset to setting
}
void test_corrections_dfco_taper_fuel()
{
  setup_DFCO_taper_on();

  correctionDFCOfuel();
  TEST_ASSERT_EQUAL(20, dfcoTaper); //Check if value was reset to setting

  BIT_SET(currentStatus.status1, BIT_STATUS1_DFCO);
  dfcoTaper = 10;
  TEST_ASSERT_EQUAL(50, correctionDFCOfuel());
  dfcoTaper = 5;
  TEST_ASSERT_EQUAL(25, correctionDFCOfuel());

  configPage9.dfcoTaperTime = 10; //1.0 second
  dfcoTaper = 15; //Check for overflow
  TEST_ASSERT_EQUAL(100, correctionDFCOfuel());
  configPage9.dfcoTaperEnable = 0; //Disable
  TEST_ASSERT_EQUAL(0, correctionDFCOfuel());
}
void test_corrections_dfco_taper_ign()
{
  setup_DFCO_taper_on();

  dfcoTaper = 20;
  BIT_SET(currentStatus.status1, BIT_STATUS1_DFCO);

  TEST_ASSERT_EQUAL(20, correctionDFCOignition(20));
  dfcoTaper = 15;
  TEST_ASSERT_EQUAL(15, correctionDFCOignition(20));
  dfcoTaper = 10;
  TEST_ASSERT_EQUAL(10, correctionDFCOignition(20));
  dfcoTaper = 5;
  TEST_ASSERT_EQUAL(5, correctionDFCOignition(20));
  configPage9.dfcoTaperEnable = 0; //Disable
  TEST_ASSERT_EQUAL(20, correctionDFCOignition(20));
}

void test_corrections_dfco()
{
  RUN_TEST(test_corrections_dfco_on);
  RUN_TEST(test_corrections_dfco_off_RPM);
  RUN_TEST(test_corrections_dfco_off_TPS);
  RUN_TEST(test_corrections_dfco_off_delay);
  RUN_TEST(test_corrections_dfco_taper);
  RUN_TEST(test_corrections_dfco_taper_fuel);
  RUN_TEST(test_corrections_dfco_taper_ign);
}

//**********************************************************************************************************************
//Setup a basic TAE enrichment curve, threshold etc that are common to all tests. Specifica values maybe updated in each individual test
void test_corrections_TAE_setup()
{
  configPage2.aeMode = AE_MODE_TPS; //Set AE to TPS

  configPage4.taeValues[0] = 70;
  configPage4.taeValues[1] = 103;
  configPage4.taeValues[2] = 124;
  configPage4.taeValues[3] = 136;

  //Note: These values are divided by 10
  configPage4.taeBins[0] = 0;
  configPage4.taeBins[1] = 8;
  configPage4.taeBins[2] = 22;
  configPage4.taeBins[3] = 97;

  configPage2.taeThresh = 0;
  configPage2.taeMinChange = 0;

  //Divided by 100
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

	//Set the coolant to be above the warmup AE taper
	configPage2.aeColdTaperMax = 60;
	configPage2.aeColdTaperMin = 0;
	currentStatus.coolant = (int)configPage2.aeColdTaperMax - CALIBRATION_TEMPERATURE_OFFSET + 1;

  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Make sure AE is turned off
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC); //Make sure AE is turned off
}

void test_corrections_TAE_no_rpm_taper()
{
  //Disable the taper
  currentStatus.RPM = 2000;
  configPage2.aeTaperMin = 50; //5000
  configPage2.aeTaperMax = 60; //6000

  currentStatus.TPS = 50; //25% actual value (range is 0 - 200)
  tpsDOT.reset();
  tpsDOT.update(0, 0, 0);
  /* Pretend 1/30 of a second has elapsed. */
  currentStatus.tpsDOT = tpsDOT.update(currentStatus.TPS, MICROS_PER_SEC / 30, configPage2.taeMinChange);

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(750, currentStatus.tpsDOT); //DOT is 750%/s (25 * 30)
  TEST_ASSERT_EQUAL(100 + 132, accelValue);
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
}

void test_corrections_TAE_50pc_rpm_taper()
{
  //RPM is 50% of the way through the taper range
  currentStatus.RPM = 3000;
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

  currentStatus.TPS = 50; //25% actual value (0 - 200 range)
  tpsDOT.reset();
  tpsDOT.update(0, 0, 0);
  /* Pretend 1/30 of a second has elapsed. */
  currentStatus.tpsDOT = tpsDOT.update(currentStatus.TPS, MICROS_PER_SEC / 30, configPage2.taeMinChange);

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(750, currentStatus.tpsDOT); //DOT is 750%/s (25 * 30)
  TEST_ASSERT_EQUAL(100 + 66, accelValue);
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
}

void test_corrections_TAE_110pc_rpm_taper()
{
  //RPM is 110% of the way through the taper range, which should result in no additional AE
  currentStatus.RPM = 5400;
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

  currentStatus.TPS = 50; //25% actual value (0 - 200 range)
  tpsDOT.reset();
  tpsDOT.update(0, 0, 0);
  /* Pretend 1/30 of a second has elapsed. */
  currentStatus.tpsDOT = tpsDOT.update(currentStatus.TPS, MICROS_PER_SEC / 30, configPage2.taeMinChange);

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(750, currentStatus.tpsDOT); //DOT is 750%/s (25 * 30)
  TEST_ASSERT_EQUAL(100, accelValue); //Should be no AE as we're above the RPM taper end point
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
}

void test_corrections_TAE_under_threshold()
{
  //RPM is 50% of the way through the taper range, but TPS value will be below threshold
  currentStatus.RPM = 3000;
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

  currentStatus.TPS = 6; //3% actual value (0 - 200 range). TPSDot should be 90%/s (3% change in 1/30 of a second)
  tpsDOT.reset();
  tpsDOT.update(0, 0, 0);
  /* Pretend 1/30 of a second has elapsed. */
  currentStatus.tpsDOT = tpsDOT.update(currentStatus.TPS, MICROS_PER_SEC / 30, configPage2.taeMinChange);

  configPage2.taeThresh = 100; //Above the reading of 90%/s

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(90, currentStatus.tpsDOT); //DOT is 90%/s (3% * 30)
  TEST_ASSERT_EQUAL(100, accelValue); //Should be no AE as we're above the RPM taper end point
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged off
}

void test_corrections_TAE_50pc_warmup_taper()
{
  //Disable the RPM taper
  currentStatus.RPM = 2000;
  configPage2.aeTaperMin = 50; //5000
  configPage2.aeTaperMax = 60; //6000

  currentStatus.TPS = 50; //25% actual value (0 - 200 range)
  tpsDOT.reset();
  tpsDOT.update(0, 0, 0);
  /* Pretend 1/30 of a second has elapsed. */
  currentStatus.tpsDOT = tpsDOT.update(currentStatus.TPS, MICROS_PER_SEC / 30, configPage2.taeMinChange);

	//Set a cold % of 50% increase
	configPage2.aeColdPct = 150;
	configPage2.aeColdTaperMax = 60 + CALIBRATION_TEMPERATURE_OFFSET;
	configPage2.aeColdTaperMin = 0 + CALIBRATION_TEMPERATURE_OFFSET;
	//Set the coolant to be 50% of the way through the warmup range
	currentStatus.coolant = 30;

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(750, currentStatus.tpsDOT); //DOT is 750%/s (25 * 30)
  TEST_ASSERT_EQUAL(100 + 165, accelValue); //Total AE should be 132 + (50% * 50%) = 132 * 1.25 = 165
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
}

void test_corrections_TAE()
{
  test_corrections_TAE_setup();


  RUN_TEST(test_corrections_TAE_no_rpm_taper);
	BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Flag must be cleared between tests
  RUN_TEST(test_corrections_TAE_50pc_rpm_taper);
	BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Flag must be cleared between tests
  RUN_TEST(test_corrections_TAE_110pc_rpm_taper);
	BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Flag must be cleared between tests
  RUN_TEST(test_corrections_TAE_under_threshold);
	BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Flag must be cleared between tests
  RUN_TEST(test_corrections_TAE_50pc_warmup_taper);


}