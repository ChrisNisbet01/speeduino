#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_missing_tooth;

void triggerSetup_missingTooth(bool initialisationComplete = false);

void triggerPri_missingTooth(void);

void triggerSec_missingTooth(void);

void triggerThird_missingTooth(void);

uint16_t getRPM_missingTooth(void);

int getCrankAngle_missingTooth(void);

void triggerSetEndTeeth_missingTooth(void);

