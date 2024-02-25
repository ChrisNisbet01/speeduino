#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_subaru_67;

void triggerSetup_Subaru67(bool initialisationComplete = false);

void triggerPri_Subaru67(void);

void triggerSec_Subaru67(void);

uint16_t getRPM_Subaru67(void);

int getCrankAngle_Subaru67(void);

void triggerSetEndTeeth_Subaru67(void);

