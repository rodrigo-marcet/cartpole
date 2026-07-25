#pragma once

#include "src/utils/hfsm_types.h"

#include "src/utils/as5600.h"

SequenceStatus as5600_calibration(AS5600CalibrationResult *result, const uint16_t idx = INNER_AS5600_IDX);
