#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_mazda_au;

void triggerSetup_MazdaAU(bool initialisationComplete = false);

void triggerPri_MazdaAU(void);

void triggerSec_MazdaAU(void);

uint16_t getRPM_MazdaAU(void);

int getCrankAngle_MazdaAU(void);

void triggerSetEndTeeth_MazdaAU(void);

