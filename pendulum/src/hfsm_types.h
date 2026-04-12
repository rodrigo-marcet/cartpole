#pragma once

#include <Arduino.h>

enum class SequenceState : uint8_t { Running = 1, Done = 2, Error = 3 };
