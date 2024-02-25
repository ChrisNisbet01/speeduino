#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_ngc_4;

extern decoder_handler_st const trigger_ngc_68;

void triggerSetup_NGC(bool initialisationComplete = false);

void triggerPri_NGC(void);

void triggerSec_NGC4(void);

void triggerSec_NGC68(void);

uint16_t getRPM_NGC(void);

void triggerSetEndTeeth_NGC(void);

