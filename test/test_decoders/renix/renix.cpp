#include <decoders.h>
#include <globals.h>
#include <unity.h>
#include "renix.h"
#include "schedule_calcs.h"

void test_setup_renix44(void)
{
  //Setup a renix 44 tooth wheel
  configPage4.TrigPattern = DECODER_RENIX;
  configPage2.nCylinders = 4;

  triggerSetup_Renix();
}

void test_setup_renix66(void)
{
  //Setup a renix 66 tooth wheel
  configPage4.TrigPattern = DECODER_RENIX;
  configPage2.nCylinders = 6;

  triggerSetup_Renix();
}

//************************************** Begin the new ignition setEndTooth tests **************************************
void test_Renix_newIgn_44_trig0_1(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=0
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition1.endAngle = 360 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 0; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(2, ignition1.endTooth);
}

void test_Renix_newIgn_44_trig90_1(void)
{
  //Test the set end tooth function. Conditions:

  //Advance: 10
  //triggerAngle=90
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition1.endAngle = 360 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 90; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(1, ignition1.endTooth);
}

void test_Renix_newIgn_44_trig180_1(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=180
  test_setup_renix44();
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition1.endAngle = 360 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 180; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(4, ignition1.endTooth);
}

void test_Renix_newIgn_44_trig270_1(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=270
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition1.endAngle = 360 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 270; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(3, ignition1.endTooth);
}

void test_Renix_newIgn_44_trig360_1(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=360
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition1.endAngle = 360 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 360; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(3, ignition1.endTooth);
}

void test_Renix_newIgn_44_trigNeg90_1(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=-90
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition1.endAngle = 360 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = -90; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(3, ignition1.endTooth);
}

void test_Renix_newIgn_44_trigNeg180_1(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=-180
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition1.endAngle = 360 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = -180; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(4, ignition1.endTooth);
}

void test_Renix_newIgn_44_trigNeg270_1(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=-270
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition1.endAngle = 360 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = -270; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(1, ignition1.endTooth);
}

void test_Renix_newIgn_44_trigNeg360_1(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=-360
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition1.endAngle = 360 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = -360; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(2, ignition1.endTooth);
}

// ******* CHannel 2 *******
void test_Renix_newIgn_44_trig0_2(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=0
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 0; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(4, ignition2.endTooth);
}

void test_Renix_newIgn_44_trig90_2(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=90
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 90; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(3, ignition2.endTooth);
}

void test_Renix_newIgn_44_trig180_2(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=180
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 180; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(3, ignition2.endTooth);
}

void test_Renix_newIgn_44_trig270_2(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=270
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 270; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(2, ignition2.endTooth);
}

void test_Renix_newIgn_44_trig366(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=360
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 360; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(1, ignition2.endTooth);
}

void test_Renix_newIgn_44_trigNeg90_2(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=-90
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = -90; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(1, ignition2.endTooth);
}

void test_Renix_newIgn_44_trigNeg180_2(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=-180
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = -180; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(2, ignition2.endTooth);
}

void test_Renix_newIgn_44_trigNeg270_2(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=-270
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = -270; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(3, ignition2.endTooth);
}

void test_Renix_newIgn_44_trigNeg366(void)
{
  //Test the set end tooth function. Conditions:
  //Advance: 10
  //triggerAngle=-360
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix44();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = -360; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(4, ignition2.endTooth);
}

void test_Renix_newIgn_66_trig0_2(void)
{
  //Test the set end tooth function. Conditions:

  //Advance: 10
  //triggerAngle=300
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix66();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 0; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(1, ignition2.endTooth);
}

void test_Renix_newIgn_66_trig181_2(void)
{
  //Test the set end tooth function. Conditions:

  //Advance: 10
  //triggerAngle=300
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  test_setup_renix66();
  configPage4.sparkMode = IGN_MODE_SINGLE;
  ignition2.endAngle = 180 - 10; //Set 10 degrees advance
  configPage4.triggerAngle = 181; //No trigger offset

  triggerSetEndTeeth_Renix();
  TEST_ASSERT_EQUAL(5, ignition2.endTooth);
}


void testRenix(void)
{
  RUN_TEST(test_Renix_newIgn_44_trig0_1);
  RUN_TEST(test_Renix_newIgn_44_trig90_1);
  RUN_TEST(test_Renix_newIgn_44_trig180_1);
  RUN_TEST(test_Renix_newIgn_44_trig270_1);
  RUN_TEST(test_Renix_newIgn_44_trig360_1);
  RUN_TEST(test_Renix_newIgn_44_trigNeg90_1);
  RUN_TEST(test_Renix_newIgn_44_trigNeg180_1);
  RUN_TEST(test_Renix_newIgn_44_trigNeg270_1);
  RUN_TEST(test_Renix_newIgn_44_trigNeg360_1);

  RUN_TEST(test_Renix_newIgn_44_trig0_2);
  RUN_TEST(test_Renix_newIgn_44_trig90_2);
  RUN_TEST(test_Renix_newIgn_44_trig180_2);
  RUN_TEST(test_Renix_newIgn_44_trig270_2);
  RUN_TEST(test_Renix_newIgn_44_trig366);
  RUN_TEST(test_Renix_newIgn_44_trigNeg90_2);
  RUN_TEST(test_Renix_newIgn_44_trigNeg180_2);
  RUN_TEST(test_Renix_newIgn_44_trigNeg270_2);
  RUN_TEST(test_Renix_newIgn_44_trigNeg366);

  RUN_TEST(test_Renix_newIgn_66_trig0_2);
  RUN_TEST(test_Renix_newIgn_66_trig181_2);

}
