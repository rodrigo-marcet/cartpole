#pragma once

#include <Arduino.h>

enum class SequenceState : uint8_t { RUNNING, DONE = 254, ERROR = 255 };
