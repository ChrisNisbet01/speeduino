#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_daihatsu_plus1;

void triggerSetup_Daihatsu(bool initialisationComplete = false);

void triggerPri_Daihatsu(void);

void triggerSec_Daihatsu(void);

uint16_t getRPM_Daihatsu(void);

int getCrankAngle_Daihatsu(void);

