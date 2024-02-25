#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_suzuki_k6a;

void triggerSetup_SuzukiK6A(bool initialisationComplete = false);

void triggerPri_SuzukiK6A(void);

void triggerSec_SuzukiK6A(void);

uint16_t getRPM_SuzukiK6A(void);

int getCrankAngle_SuzukiK6A(void);

void triggerSetEndTeeth_SuzukiK6A(void);

