#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_non_360;

void triggerSetup_non360(bool initialisationComplete = false);

uint16_t getRPM_non360(void);

int getCrankAngle_non360(void);

void triggerSetEndTeeth_non360(void);

