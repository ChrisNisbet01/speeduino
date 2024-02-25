#pragma once

#include <stdint.h>

void triggerSetup_Miata9905(bool initialisationComplete);

void triggerPri_Miata9905(void);

void triggerSec_Miata9905(void);

uint16_t getRPM_Miata9905(void);

int getCrankAngle_Miata9905(void);

void triggerSetEndTeeth_Miata9905(void);

int getCamAngle_Miata9905(void);

