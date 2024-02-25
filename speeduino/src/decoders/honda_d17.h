#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_honda_d17;

void triggerSetup_HondaD17(bool initialisationComplete = false);

void triggerPri_HondaD17(void);

void triggerSec_HondaD17(void);

uint16_t getRPM_HondaD17(void);

int getCrankAngle_HondaD17(void);

