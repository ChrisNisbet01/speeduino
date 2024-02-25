#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_st170;

void triggerSetup_FordST170(bool initialisationComplete = false);

void triggerSec_FordST170(void);

uint16_t getRPM_FordST170(void);

int getCrankAngle_FordST170(void);

void triggerSetEndTeeth_FordST170(void);

