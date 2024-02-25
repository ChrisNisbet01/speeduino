#pragma once

#include <stdint.h>

void triggerSetup_SuzukiK6A(bool initialisationComplete = false);

void triggerPri_SuzukiK6A(void);

void triggerSec_SuzukiK6A(void);

uint16_t getRPM_SuzukiK6A(void);

int getCrankAngle_SuzukiK6A(void);

void triggerSetEndTeeth_SuzukiK6A(void);

