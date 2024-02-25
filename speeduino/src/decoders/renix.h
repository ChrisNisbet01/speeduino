#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_renix;

void triggerSetup_Renix(bool initialisationComplete = false);

void triggerPri_Renix(void);

void triggerSetEndTeeth_Renix(void);

