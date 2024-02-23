#include <unity.h>
#include "decoders/decoders.h"
#include "globals.h"
#include "Nissan360.h"
#include "scheduler.h"
#include "schedule_calcs.h"
#include "ignition_contexts.h"

void test_nissan360_newIgn_12_trig0_1(void)
{
  //Test the set end tooth function. Conditions:
  //Trigger: 12/1
  //Advance: 10
  //triggerAngle=0
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  triggerSetup_Nissan360();
  configPage4.sparkMode = IGN_MODE_WASTED;
  configPage4.triggerAngle = 0; //No trigger offset

  ignition1.calculateIgnitionAngle(5, 0, 10);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(171, ignition1.endTooth);

  //Test again with 0 degrees advance
  ignition1.calculateIgnitionAngle(5, 0, 0);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(176, ignition1.endTooth);

  //Test again with 35 degrees advance
  ignition1.calculateIgnitionAngle(5, 0, 35);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(158, ignition1.endTooth);
}

void test_nissan360_newIgn_12_trig90_1(void)
{
  //Test the set end tooth function. Conditions:
  //Trigger: 12/1
  //Advance: 10
  //triggerAngle=90
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  triggerSetup_Nissan360();
  configPage4.sparkMode = IGN_MODE_WASTED;
  configPage4.triggerAngle = 90; //No trigger offset

  ignition1.calculateIgnitionAngle(5, 0, 10);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(126, ignition1.endTooth);
}

void test_nissan360_newIgn_12_trig180_1(void)
{
  //Test the set end tooth function. Conditions:
  //Trigger: 36-1
  //Advance: 10
  //triggerAngle=180
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  triggerSetup_Nissan360();
  configPage4.sparkMode = IGN_MODE_WASTED;
  configPage4.triggerAngle = 180; //No trigger offset

  ignition1.calculateIgnitionAngle(5, 0, 10);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(81, ignition1.endTooth);
}

void test_nissan360_newIgn_12_trig270_1(void)
{
  //Test the set end tooth function. Conditions:
  //Trigger: 36-1
  //Advance: 10
  //triggerAngle=270
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  triggerSetup_Nissan360();
  configPage4.sparkMode = IGN_MODE_WASTED;
  configPage4.triggerAngle = 270; //No trigger offset

  ignition1.calculateIgnitionAngle(5, 0, 10);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(36, ignition1.endTooth);
}

void test_nissan360_newIgn_12_trig360_1(void)
{
  //Test the set end tooth function. Conditions:
  //Trigger: 36-1
  //Advance: 10
  //triggerAngle=360
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  triggerSetup_Nissan360();
  configPage4.sparkMode = IGN_MODE_WASTED;
  configPage4.triggerAngle = 360; //No trigger offset

  ignition1.calculateIgnitionAngle(5, 0, 10);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(351, ignition1.endTooth);
}

void test_nissan360_newIgn_12_trigNeg90_1(void)
{
  //Test the set end tooth function. Conditions:
  //Trigger: 36-1
  //Advance: 10
  //triggerAngle=-90
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  triggerSetup_Nissan360();
  configPage4.sparkMode = IGN_MODE_WASTED;
  configPage4.triggerAngle = -90; //No trigger offset

  ignition1.calculateIgnitionAngle(5, 0, 10);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(216, ignition1.endTooth);
}

void test_nissan360_newIgn_12_trigNeg180_1(void)
{
  //Test the set end tooth function. Conditions:
  //Trigger: 36-1
  //Advance: 10
  //triggerAngle=-180
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  triggerSetup_Nissan360();
  configPage4.sparkMode = IGN_MODE_WASTED;
  configPage4.triggerAngle = -180; //No trigger offset

  ignition1.calculateIgnitionAngle(5, 0, 10);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(261, ignition1.endTooth);
}

void test_nissan360_newIgn_12_trigNeg270_1(void)
{
  //Test the set end tooth function. Conditions:
  //Trigger: 36-1
  //Advance: 10
  //triggerAngle=-270
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  triggerSetup_Nissan360();
  configPage4.sparkMode = IGN_MODE_WASTED;
  configPage4.triggerAngle = -270; //No trigger offset

  ignition1.calculateIgnitionAngle(5, 0, 10);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(306, ignition1.endTooth);
}

void test_nissan360_newIgn_12_trigNeg360_1(void)
{
  //Test the set end tooth function. Conditions:
  //Trigger: 36-1
  //Advance: 10
  //triggerAngle=-360
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  triggerSetup_Nissan360();
  configPage4.sparkMode = IGN_MODE_WASTED;
  configPage4.triggerAngle = -360; //No trigger offset

  ignition1.calculateIgnitionAngle(5, 0, 10);
  triggerSetEndTeeth_Nissan360();
  TEST_ASSERT_EQUAL(351, ignition1.endTooth);
}

void testNissan360(void)
{
  RUN_TEST(test_nissan360_newIgn_12_trig0_1);
  RUN_TEST(test_nissan360_newIgn_12_trig90_1);
  RUN_TEST(test_nissan360_newIgn_12_trig180_1);
  RUN_TEST(test_nissan360_newIgn_12_trig270_1);
  RUN_TEST(test_nissan360_newIgn_12_trig360_1);
  RUN_TEST(test_nissan360_newIgn_12_trigNeg90_1);
  RUN_TEST(test_nissan360_newIgn_12_trigNeg180_1);
  RUN_TEST(test_nissan360_newIgn_12_trigNeg270_1);
  RUN_TEST(test_nissan360_newIgn_12_trigNeg360_1);
}
