#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_4G63;

void triggerSetup_4G63(bool initialisationComplete);

void triggerPri_4G63(void);

void triggerSec_4G63(void);

uint16_t getRPM_4G63(void);

int getCrankAngle_4G63(void);

void triggerSetEndTeeth_4G63(void);

