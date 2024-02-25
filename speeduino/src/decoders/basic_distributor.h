#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_basic_distributor;

void triggerSetup_BasicDistributor(bool initialisationComplete = false);

void triggerPri_BasicDistributor(void);

uint16_t getRPM_BasicDistributor(void);

int getCrankAngle_BasicDistributor(void);

void triggerSetEndTeeth_BasicDistributor(void);

