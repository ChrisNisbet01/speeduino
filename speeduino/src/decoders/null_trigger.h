#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_null;

void nullTriggerHandler(void);

uint16_t nullGetRPM(void);

int nullGetCrankAngle(void);

void nullSetEndTeeth(void);

