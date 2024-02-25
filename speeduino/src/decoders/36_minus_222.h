#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_36_minus_222;

void triggerSetup_ThirtySixMinus222(bool initialisationComplete = false);

void triggerPri_ThirtySixMinus222(void);

void triggerSec_ThirtySixMinus222(void);

uint16_t getRPM_ThirtySixMinus222(void);

int getCrankAngle_ThirtySixMinus222(void);

void triggerSetEndTeeth_ThirtySixMinus222(void);

