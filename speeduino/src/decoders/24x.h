#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_24X;

void triggerSetup_24X(bool initialisationComplete = false);

void triggerPri_24X(void);

void triggerSec_24X(void);

uint16_t getRPM_24X(void);

int getCrankAngle_24X(void);

