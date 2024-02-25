#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_drz400;

void triggerSetup_DRZ400(bool initialisationComplete = false);

void triggerSec_DRZ400(void);

