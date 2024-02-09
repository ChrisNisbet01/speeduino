#pragma once

#include "ignition_id.h"
#include "scheduler.h"
#include "maths.h"

typedef enum
{
  ignChannel1,
  ignChannel2,
  ignChannel3,
  ignChannel4,
  ignChannel5,
#if IGN_CHANNELS >= 6
  ignChannel6,
#endif
#if IGN_CHANNELS >= 6
  ignChannel7,
#endif
#if IGN_CHANNELS >= 6
  ignChannel8,
#endif
  ignChannelCount,
} ignitionChannelID_t;

typedef struct ignition_context_st
{
public:
  IgnitionSchedule * ignitionSchedule;
  int endAngle;
  int startAngle;
  uint16_t endTooth = 0;
  /* The number of crank degrees until the cylinder is at TDC.
   * (This is obviously 0 for cylinder 0 for virtually ALL engines,
   * but there's some weird ones).
   */
  int ignDegrees;

  void reset(void);

  bool adjustCrankAngle(int crankAngle, uint16_t currentTooth);

  void calculateIgnitionAngle(int dwellAngle, uint16_t channelIgnDegrees, int8_t advance);

  void calculateIgnitionAngle(int dwellAngle, int8_t advance);

  uint32_t calculateIgnitionTimeout(int crankAngle);

  void setIgnitionSchedule(unsigned long timeout, unsigned long durationMicrosecs);

  void applyOverDwellCheck(uint32_t targetOverdwellTime);

  void configure_coil_schedule(ignition_id_t const id);

  void configure_coil_schedule(ignition_id_t const id1, ignition_id_t const id2);

  void inhibit_coil_schedule(void);

} ignition_context_st;


typedef struct ignition_contexts_st
{
public:
  byte maxOutputs = 1; /**< Number of ignition outputs being used by the current tune configuration */
  uint8_t channelsOn;
  uint8_t channelsPending;

  ignition_context_st& ignition(ignitionChannelID_t ign)
  {
    return ignitions[ign];
  }

  void adjustCrankAngle(int16_t crankAngle, uint16_t currentTooth);

  void adjustStartAngle(int adjustment);

  void resetEndAngle(void);

  void setMaxIgnitions(byte const maxOutputs);

  void setAllOn(void);

  void setAllOff(void);

  void setOn(ignitionChannelID_t ign);

  void setOff(ignitionChannelID_t ign);

  bool isOperational(ignitionChannelID_t ign);

  byte channelsOnMask(void);

  void setChannelsOnMask(uint8_t mask);

  void applyIgnitionControl(ignitionChannelID_t ign, int crankAngle);

  void configure_coil_schedule(ignitionChannelID_t ign, ignition_id_t const id);

  void configure_coil_schedule(ignitionChannelID_t ign, ignition_id_t const id1, ignition_id_t const id2);

  void inhibit_coil_schedule(ignitionChannelID_t ign);

  void configure_rotary_fc_trailing_coil_schedules(void);

private:
  ignition_context_st ignitions[ignChannelCount];

  byte maxOutputMask = 0x01;
} ignition_contexts_st;

extern ignition_contexts_st ignitions;

