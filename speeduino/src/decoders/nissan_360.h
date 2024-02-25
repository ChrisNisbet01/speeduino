#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_nissan_360;

void triggerSetup_Nissan360(bool initialisationComplete = false);

void triggerPri_Nissan360(void);

void triggerSec_Nissan360(void);

uint16_t getRPM_Nissan360(void);

int getCrankAngle_Nissan360(void);

void triggerSetEndTeeth_Nissan360(void);

