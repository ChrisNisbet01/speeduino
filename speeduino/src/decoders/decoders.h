#pragma once

#include "../../globals.h"
#include "decoder_structs.h"

#include <stdint.h>

#define DECODER_MISSING_TOOTH     0
#define DECODER_BASIC_DISTRIBUTOR 1
#define DECODER_DUAL_WHEEL        2
#define DECODER_GM7X              3
#define DECODER_4G63              4
#define DECODER_24X               5
#define DECODER_JEEP2000          6
#define DECODER_AUDI135           7
#define DECODER_HONDA_D17         8
#define DECODER_MIATA_9905        9
#define DECODER_MAZDA_AU          10
#define DECODER_NON360            11
#define DECODER_NISSAN_360        12
#define DECODER_SUBARU_67         13
#define DECODER_DAIHATSU_PLUS1    14
#define DECODER_HARLEY            15
#define DECODER_36_2_2_2          16
#define DECODER_36_2_1            17
#define DECODER_420A              18
#define DECODER_WEBER             19
#define DECODER_ST170             20
#define DECODER_DRZ400            21
#define DECODER_NGC               22
#define DECODER_VMAX              23
#define DECODER_RENIX             24
#define DECODER_ROVERMEMS		      25
#define DECODER_SUZUKI_K6A        26

#define BIT_DECODER_2ND_DERIV           0 //The use of the 2nd derivative calculation is limited to certain decoders. This is set to either true or false in each decoders setup routine
#define BIT_DECODER_IS_SEQUENTIAL       1 //Whether or not the decoder supports sequential operation
#define BIT_DECODER_UNUSED1             2
#define BIT_DECODER_HAS_SECONDARY       3 //Whether or not the decoder supports fixed cranking timing
#define BIT_DECODER_HAS_FIXED_CRANKING  4
#define BIT_DECODER_VALID_TRIGGER       5 //Is set true when the last trigger (Primary or secondary) was valid (ie passed filters)
#define BIT_DECODER_TOOTH_ANG_CORRECT   6 //Whether or not the triggerToothAngle variable is currently accurate. Some patterns have times when the triggerToothAngle variable cannot be accurately set.

//220 bytes free
extern volatile uint8_t decoderState;

/*
extern volatile bool validTrigger; //Is set true when the last trigger (Primary or secondary) was valid (ie passed filters)
extern volatile bool triggerToothAngleIsCorrect; //Whether or not the triggerToothAngle variable is currently accurate. Some patterns have times when the triggerToothAngle variable cannot be accurately set.
extern bool secondDerivEnabled; //The use of the 2nd derivative calculation is limited to certain decoders. This is set to either true or false in each decoders setup routine
extern bool decoderIsSequential; //Whether or not the decoder supports sequential operation
extern bool decoderHasSecondary; //Whether or not the pattern uses a secondary input
extern bool decoderHasFixedCrankingTiming;
*/

//This isn't to to filter out wrong pulses on triggers, but just to smooth out
//the cam angle reading for better closed loop VVT control.
#define ANGLE_FILTER(input, alpha, prior) (((long)(input) * (256 - (alpha)) + ((long)(prior) * (alpha)))) >> 8

void loggerPrimaryISR(void);
void loggerSecondaryISR(void);
void loggerTertiaryISR(void);

__attribute__((noinline))int crankingGetRPM(byte totalTeeth, bool isCamTeeth);

extern decoder_context_st decoder;

extern volatile uint32_t curTime;
extern volatile uint32_t curGap;
extern volatile uint32_t curTime2;
extern volatile uint32_t curGap2;
extern volatile uint32_t curTime3;
extern volatile uint32_t curGap3;
extern volatile uint32_t lastGap;
extern volatile uint32_t targetGap;
extern volatile uint32_t targetGap2;

//The maximum time (in uS) that the system will continue to function before the
//engine is considered stalled/stopped. This is unique to each decoder,
//depending on the number of teeth etc. 500000 (half a second) is used as the
//default value, most decoders will be much less.
extern uint32_t MAX_STALL_TIME;
extern volatile byte toothSystemCount; //Used for decoders such as Audi 135 where not every tooth is used for calculating crank angle. This variable stores the actual number of teeth, not the number being used to calculate crank angle
extern volatile uint32_t toothSystemLastToothTime; //As below, but used for decoders where not every tooth count is used for calculation
extern volatile uint32_t toothLastToothTime; //The time (micros()) that the last tooth was registered
extern volatile uint32_t toothLastSecToothTime; //The time (micros()) that the last tooth was registered on the secondary input
extern volatile uint32_t toothLastThirdToothTime; //The time (micros()) that the last tooth was registered on the second cam input
extern volatile uint32_t toothLastMinusOneToothTime; //The time (micros()) that the tooth before the last tooth was registered
extern volatile uint32_t toothLastMinusOneSecToothTime; //The time (micros()) that the tooth before the last tooth was registered on secondary input

extern volatile uint32_t toothOneTime; //The time (micros()) that tooth 1 last triggered
extern volatile uint32_t toothOneMinusOneTime; //The 2nd to last time (micros()) that tooth 1 last triggered
extern volatile bool revolutionOne; // For sequential operation, this tracks whether the current revolution is 1 or 2 (not 1)

extern volatile unsigned int secondaryToothCount; //Used for identifying the current secondary (Usually cam) tooth for patterns with multiple secondary teeth
extern volatile uint32_t secondaryLastToothTime; //The time (micros()) that the last tooth was registered (Cam input)
extern volatile uint32_t secondaryLastToothTime1; //The time (micros()) that the last tooth was registered (Cam input)

extern uint16_t triggerActualTeeth;
//The number of crank degrees that elapse per tooth
extern volatile uint16_t triggerToothAngle;
// The shortest time (in uS) that pulses will be accepted
// (Used for debounce filtering)
extern volatile uint32_t triggerFilterTime;
extern volatile uint32_t triggerSecFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the secondary input
extern unsigned int triggerSecFilterTime_duration; // The shortest valid time (in uS) pulse DURATION
extern byte checkSyncToothCount; //How many teeth must've been seen on this revolution before we try to confirm sync (Useful for missing tooth type decoders)
extern uint32_t lastVVTtime; //The time between the vvt reference pulse and the last crank pulse


//An array for storing fixed tooth angles. Currently sized at 24 for the GM 24X decoder,
//but may grow later if there are other decoders that use this style
extern int16_t toothAngles[24];

#define CRANK_SPEED 0U
#define CAM_SPEED   1U

typedef enum tooth_source_t
{
  TOOTH_CRANK,
  TOOTH_CAM_SECONDARY,
  TOOTH_CAM_TERTIARY,
} tooth_source_t;

// used by the ROVER MEMS pattern
#define ID_TOOTH_PATTERN 0 // have we identified teeth to skip for calculating RPM?
#define SKIP_TOOTH1 1
#define SKIP_TOOTH2 2
#define SKIP_TOOTH3 3
#define SKIP_TOOTH4 4

void initialise_trigger_handler(byte trigger_pattern);

