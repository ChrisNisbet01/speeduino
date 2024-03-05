#pragma once

#include "ignition_id.h"
#include "scheduler.h"
#include "maths.h"

#include <stddef.h>

typedef enum
{
  ignChannel1,
  ignChannel2,
  ignChannel3,
  ignChannel4,
#if IGN_CHANNELS >= 5
  ignChannel5,
#endif
#if IGN_CHANNELS >= 6
  ignChannel6,
#endif
#if IGN_CHANNELS >= 7
  ignChannel7,
#endif
#if IGN_CHANNELS >= 8
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

  void inhibit_coil_schedule(void);

} ignition_context_st;

extern ignition_context_st ignition_contexts[ignChannelCount];

typedef struct ignition_contexts_st
{
public:
  byte maxOutputs = 1; /**< Number of ignition outputs being used by the current tune configuration */
  uint8_t channelsOn = 0;
  uint8_t channelsPending = 0;

  ignition_context_st &ignition(ignitionChannelID_t ign)
  {
    return ignition_contexts[ign];
  }

  void adjustCrankAngle(int16_t crankAngle, uint16_t currentTooth)
  {
    for (size_t i = 0; i < ignChannelCount; i++)
    {
      if (ignition_contexts[i].adjustCrankAngle(crankAngle, currentTooth))
      {
        break;
      }
    }
  }

  void adjustStartAngle(int adjustment)
  {
    for (size_t i = 0; i < ignChannelCount; i++)
    {
      ignition_contexts[i].startAngle += adjustment;
    }
  }

  void resetEndAngle(void)
  {
    for (size_t i = 0; i < ignChannelCount; i++)
    {
      ignition_context_st &ignition = ignition_contexts[i];

      ignition.endAngle = 0;
    }
  }

  void setMaxIgnitions(byte const maxOutputs);

  void setAllOn(void);

  void setAllOff(void);

  void setOn(ignitionChannelID_t ign);

  void setOff(ignitionChannelID_t ign);

  bool isOperational(ignitionChannelID_t ign);

  byte channelsOnMask(void);

  void setChannelsOnMask(uint8_t mask);

  void applyIgnitionControl(ignitionChannelID_t ign, int crankAngle);

  void inhibit_coil_schedule(ignitionChannelID_t ign);

private:
  //ignition_context_st ignitions[ignChannelCount];

  byte maxOutputMask = 0x01;
} ignition_contexts_st;

extern ignition_contexts_st ignitions;

