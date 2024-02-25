#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_audi_135;

void triggerSetup_Audi135(bool initialisationComplete = false);

void triggerPri_Audi135(void);

void triggerSec_Audi135(void);

uint16_t getRPM_Audi135(void);

int getCrankAngle_Audi135(void);

