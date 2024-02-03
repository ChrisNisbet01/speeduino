#include "schedule_calcs.h"

int ignition1StartAngle;
int ignition1EndAngle;
int channel1IgnDegrees; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */

int ignition2StartAngle;
int ignition2EndAngle;
int channel2IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

int ignition3StartAngle;
int ignition3EndAngle;
int channel3IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

int ignition4StartAngle;
int ignition4EndAngle;
int channel4IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

#if (IGN_CHANNELS >= 5)
int ignition5StartAngle;
int ignition5EndAngle;
int channel5IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 6)
int ignition6StartAngle;
int ignition6EndAngle;
int channel6IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 7)
int ignition7StartAngle;
int ignition7EndAngle;
int channel7IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 8)
int ignition8StartAngle;
int ignition8EndAngle;
int channel8IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif

