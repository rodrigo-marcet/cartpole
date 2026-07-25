#pragma once

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "./src/models/swingup_perturbations_v3.h"

constexpr int kTensorArenaSize = 16 * 1024; // 32 32
extern uint8_t tensor_arena[kTensorArenaSize];

extern const tflite::Model *model;
extern tflite::MicroInterpreter *interpreter;
extern TfLiteTensor *input;
extern TfLiteTensor *output;

void scale_observations(float *obs, const int n);
