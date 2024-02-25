#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_36_minus_21;

void triggerSetup_ThirtySixMinus21(bool initialisationComplete = false);

void triggerPri_ThirtySixMinus21(void);

void triggerSec_ThirtySixMinus21(void);

uint16_t getRPM_ThirtySixMinus21(void);

int getCrankAngle_ThirtySixMinus21(void);

void triggerSetEndTeeth_ThirtySixMinus21(void);

