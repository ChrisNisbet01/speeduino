
/** \file
 * Header file for the TunerStudio command handler
 * The command handler manages all the inputs FROM TS which are issued when a command button is clicked by the user
 */

#include "globals.h"
#include "TS_CommandButtonHandler.h"
#include "utilities.h"
#include "ignition_control.h"
#include "injector_control.h"
#include "sensors.h"
#include "storage.h"
#include "SD_logger.h"
#ifdef USE_MC33810
  #include "acc_mc33810.h"
#endif

static bool commandRequiresStoppedEngine(uint16_t buttonCommand)
{
  return ((buttonCommand >= TS_CMD_INJ1_ON) && (buttonCommand <= TS_CMD_IGN8_PULSED))
      || ((buttonCommand == TS_CMD_TEST_ENBL) || (buttonCommand == TS_CMD_TEST_DSBL));
}

/**
 * @brief
 *
 * @param buttonCommand The command number of the button that was clicked. See TS_CommendButtonHandler.h for a list of button IDs
 */
bool TS_CommandButtonsHandler(uint16_t buttonCommand)
{
  if (commandRequiresStoppedEngine(buttonCommand) && currentStatus.RPM != 0)
  {
    return false;
  }

  switch (buttonCommand)
  {
    case TS_CMD_TEST_DSBL: // cmd is stop
      BIT_CLEAR(currentStatus.testOutputs, 1);
      singleCoilEndCharge(ignition_id_1);
      singleCoilEndCharge(ignition_id_2);
      singleCoilEndCharge(ignition_id_3);
      singleCoilEndCharge(ignition_id_4);
      #if IGN_CHANNELS >= 5
      singleCoilEndCharge(ignition_id_5);
      #endif
      #if IGN_CHANNELS >= 6
      singleCoilEndCharge(ignition_id_6);
      #endif
      #if IGN_CHANNELS >= 7
      singleCoilEndCharge(ignition_id_7);
      #endif
      #if IGN_CHANNELS >= 8
      singleCoilEndCharge(ignition_id_8);
      #endif


      closeSingleInjector(injector_id_1);
      closeSingleInjector(injector_id_2);
      closeSingleInjector(injector_id_3);
      closeSingleInjector(injector_id_4);
      #if INJ_CHANNELS >= 5
      closeSingleInjector(injector_id_5);
      #endif
      #if INJ_CHANNELS >= 6
      closeSingleInjector(injector_id_6);
      #endif
      #if INJ_CHANNELS >= 7
      closeSingleInjector(injector_id_7);
      #endif
      #if INJ_CHANNELS >= 8
      closeSingleInjector(injector_id_8);
      #endif

      HWTest_INJ_Pulsed = 0;
      HWTest_IGN_Pulsed = 0;
      break;

    case TS_CMD_TEST_ENBL: // cmd is enable
      // currentStatus.testactive = 1;
      BIT_SET(currentStatus.testOutputs, 1);
      break;

    case TS_CMD_INJ1_ON: // cmd group is for injector1 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openSingleInjector(injector_id_1); }
      break;

    case TS_CMD_INJ1_OFF: // cmd group is for injector1 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeSingleInjector(injector_id_1); BIT_CLEAR(HWTest_INJ_Pulsed, INJ1_CMD_BIT); }
      break;

    case TS_CMD_INJ1_PULSED: // cmd group is for injector1 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ1_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ1_CMD_BIT)) { closeSingleInjector(injector_id_1); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ2_ON: // cmd group is for injector2 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openSingleInjector(injector_id_2); }
      break;

    case TS_CMD_INJ2_OFF: // cmd group is for injector2 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeSingleInjector(injector_id_2); BIT_CLEAR(HWTest_INJ_Pulsed, INJ2_CMD_BIT); }
      break;

    case TS_CMD_INJ2_PULSED: // cmd group is for injector2 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ2_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ2_CMD_BIT)) { closeSingleInjector(injector_id_2); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ3_ON: // cmd group is for injector3 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openSingleInjector(injector_id_3); }
      break;

    case TS_CMD_INJ3_OFF: // cmd group is for injector3 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeSingleInjector(injector_id_3); BIT_CLEAR(HWTest_INJ_Pulsed, INJ3_CMD_BIT); }
      break;

    case TS_CMD_INJ3_PULSED: // cmd group is for injector3 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ3_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ3_CMD_BIT)) { closeSingleInjector(injector_id_3); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ4_ON: // cmd group is for injector4 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openSingleInjector(injector_id_4); }
      break;

    case TS_CMD_INJ4_OFF: // cmd group is for injector4 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeSingleInjector(injector_id_4); BIT_CLEAR(HWTest_INJ_Pulsed, INJ4_CMD_BIT); }
      break;

    case TS_CMD_INJ4_PULSED: // cmd group is for injector4 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ4_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ4_CMD_BIT)) { closeSingleInjector(injector_id_4); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ5_ON: // cmd group is for injector5 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openSingleInjector(injector_id_5); }
      break;

    case TS_CMD_INJ5_OFF: // cmd group is for injector5 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeSingleInjector(injector_id_5); BIT_CLEAR(HWTest_INJ_Pulsed, INJ5_CMD_BIT); }
      break;

    case TS_CMD_INJ5_PULSED: // cmd group is for injector5 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ5_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ5_CMD_BIT)) { closeSingleInjector(injector_id_5); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ6_ON: // cmd group is for injector6 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openSingleInjector(injector_id_6); }
      break;

    case TS_CMD_INJ6_OFF: // cmd group is for injector6 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeSingleInjector(injector_id_6); BIT_CLEAR(HWTest_INJ_Pulsed, INJ6_CMD_BIT); }
      break;

    case TS_CMD_INJ6_PULSED: // cmd group is for injector6 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ6_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ6_CMD_BIT)) { closeSingleInjector(injector_id_6); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ7_ON: // cmd group is for injector7 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openSingleInjector(injector_id_7); }
      break;

    case TS_CMD_INJ7_OFF: // cmd group is for injector7 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeSingleInjector(injector_id_7); BIT_CLEAR(HWTest_INJ_Pulsed, INJ7_CMD_BIT); }
      break;

    case TS_CMD_INJ7_PULSED: // cmd group is for injector7 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ7_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ7_CMD_BIT)) { closeSingleInjector(injector_id_7); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ8_ON: // cmd group is for injector8 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openSingleInjector(injector_id_8); }
      break;

    case TS_CMD_INJ8_OFF: // cmd group is for injector8 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeSingleInjector(injector_id_8); BIT_CLEAR(HWTest_INJ_Pulsed, INJ8_CMD_BIT); }
      break;

    case TS_CMD_INJ8_PULSED: // cmd group is for injector8 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ8_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ8_CMD_BIT)) { closeSingleInjector(injector_id_8); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN1_ON: // cmd group is for spark1 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ singleCoilBeginCharge(ignition_id_1); }
      break;

    case TS_CMD_IGN1_OFF: // cmd group is for spark1 off actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilEndCharge(ignition_id_1); BIT_CLEAR(HWTest_IGN_Pulsed, IGN1_CMD_BIT); }
      break;

    case TS_CMD_IGN1_PULSED: // cmd group is for spark1 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN1_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN1_CMD_BIT)) { singleCoilEndCharge(ignition_id_1); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN2_ON: // cmd group is for spark2 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilBeginCharge(ignition_id_2); }
      break;

    case TS_CMD_IGN2_OFF: // cmd group is for spark2 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilEndCharge(ignition_id_2); BIT_CLEAR(HWTest_IGN_Pulsed, IGN2_CMD_BIT); }
      break;

    case TS_CMD_IGN2_PULSED: // cmd group is for spark2 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN2_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN2_CMD_BIT)) { singleCoilEndCharge(ignition_id_2); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN3_ON: // cmd group is for spark3 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilBeginCharge(ignition_id_3); }
      break;

    case TS_CMD_IGN3_OFF: // cmd group is for spark3 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilEndCharge(ignition_id_3); BIT_CLEAR(HWTest_IGN_Pulsed, IGN3_CMD_BIT); }
      break;

    case TS_CMD_IGN3_PULSED: // cmd group is for spark3 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN3_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN3_CMD_BIT)) { singleCoilEndCharge(ignition_id_3); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN4_ON: // cmd group is for spark4 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilBeginCharge(ignition_id_4); }
      break;

    case TS_CMD_IGN4_OFF: // cmd group is for spark4 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilEndCharge(ignition_id_4); BIT_CLEAR(HWTest_IGN_Pulsed, IGN4_CMD_BIT); }
      break;

    case TS_CMD_IGN4_PULSED: // cmd group is for spark4 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN4_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN4_CMD_BIT)) { singleCoilEndCharge(ignition_id_4); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN5_ON: // cmd group is for spark5 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilBeginCharge(ignition_id_5); }
      break;

    case TS_CMD_IGN5_OFF: // cmd group is for spark5 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilEndCharge(ignition_id_5); BIT_CLEAR(HWTest_IGN_Pulsed, IGN5_CMD_BIT); }
      break;

    case TS_CMD_IGN5_PULSED: // cmd group is for spark4 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN5_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN5_CMD_BIT)) { singleCoilEndCharge(ignition_id_5); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN6_ON: // cmd group is for spark6 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilBeginCharge(ignition_id_6); }
      break;

    case TS_CMD_IGN6_OFF: // cmd group is for spark6 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilEndCharge(ignition_id_6); BIT_CLEAR(HWTest_IGN_Pulsed, IGN6_CMD_BIT); }
      break;

    case TS_CMD_IGN6_PULSED: // cmd group is for spark6 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN6_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN6_CMD_BIT)) { singleCoilEndCharge(ignition_id_6); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN7_ON: // cmd group is for spark7 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilBeginCharge(ignition_id_7); }
      break;

    case TS_CMD_IGN7_OFF: // cmd group is for spark7 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilEndCharge(ignition_id_7); BIT_CLEAR(HWTest_IGN_Pulsed, IGN7_CMD_BIT); }
      break;

    case TS_CMD_IGN7_PULSED: // cmd group is for spark7 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN7_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN7_CMD_BIT)) { singleCoilEndCharge(ignition_id_7); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN8_ON: // cmd group is for spark8 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilBeginCharge(ignition_id_8); }
      break;

    case TS_CMD_IGN8_OFF: // cmd group is for spark8 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { singleCoilEndCharge(ignition_id_8); BIT_CLEAR(HWTest_IGN_Pulsed, IGN8_CMD_BIT); }
      break;

    case TS_CMD_IGN8_PULSED: // cmd group is for spark8 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN8_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN8_CMD_BIT)) { singleCoilEndCharge(ignition_id_8); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    //VSS Calibration routines
    case TS_CMD_VSS_60KMH:
      {
        if(configPage2.vssMode == 1)
        {
          //Calculate the ratio of VSS reading from Aux input and actual VSS (assuming that actual VSS is really 60km/h).
          configPage2.vssPulsesPerKm = (currentStatus.canin[configPage2.vssAuxCh] / 60);
          writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
          BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
        }
        else
        {
          //Calibrate the actual pulses per distance
          uint32_t calibrationGap = vssGetPulseGap(0);
          if( calibrationGap > 0 )
          {
            configPage2.vssPulsesPerKm = MICROS_PER_MIN / calibrationGap;
            writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
            BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
          }
        }
      }
      break;

    //Calculate the RPM to speed ratio for each gear
    case TS_CMD_VSS_RATIO1:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio1 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO2:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio2 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO3:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio3 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO4:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio4 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO5:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio5 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO6:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio6 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    //STM32 Commands
    case TS_CMD_STM32_REBOOT: //
      doSystemReset();
      break;

    case TS_CMD_STM32_BOOTLOADER: //
      jumpToBootloader();
      break;

#ifdef SD_LOGGING
    case TS_CMD_SD_FORMAT: //Format SD card
      formatExFat();
      break;
#endif

    default:
      return false;
      break;
  }

  return true;
}
