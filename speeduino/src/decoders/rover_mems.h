#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_rover_mems;

void triggerSetup_RoverMEMS(bool initialisationComplete = false);

void triggerPri_RoverMEMS(void);

void triggerSec_RoverMEMS(void);

uint16_t getRPM_RoverMEMS(void);

int getCrankAngle_RoverMEMS(void);

void triggerSetEndTeeth_RoverMEMS(void);

