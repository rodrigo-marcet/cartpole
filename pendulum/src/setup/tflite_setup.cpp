#include "./src/setup/tflite_setup.h"

#include <Arduino.h>

#include "./src/setup/LED_setup.h"

#include "./src/models/pos_0.3.h"

#include "./src/utils/tflite.h"
#include "./src/utils/log_macros.h"

alignas(16) uint8_t tensor_arena[kTensorArenaSize];
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;

void init_tflite() {

	// Load model from the C array in policy_model.h
	model = tflite::GetModel(policy_model);
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		BOOT_ERROR("[TFLITE] Model schema mismatch!");
		halt_with_led(Color::PURPLE);
	}

	// Register only the ops this model uses
	// ELU + FullyConnected (which covers MatMul+Add internally)
	static tflite::MicroMutableOpResolver<5> resolver;
	resolver.AddFullyConnected();
	resolver.AddElu();
	resolver.AddReshape();
	resolver.AddQuantize(); // include just in case
	resolver.AddTanh();

	// Build interpreter
	static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
	interpreter = &static_interpreter;

	if (interpreter->AllocateTensors() != kTfLiteOk) {
		BOOT_ERROR("[TFLITE] AllocateTensors failed!");
		halt_with_led(Color::PURPLE);
	}

	input = interpreter->input(0);
	output = interpreter->output(0);

	BOOT_LOG("[TFLITE] Input dims:  [%d, %d]\n", input->dims->data[0], input->dims->data[1]);
	BOOT_LOG("[TFLITE] Output dims: [%d, %d]\n", output->dims->data[0], output->dims->data[1]);

	// --- Warm-up run ---
	float dummy[5] = {0.1f, 0.05f, 0.0f, 0.0f, 0.1f};
	for (int i = 0; i < 5; i++)
		input->data.f[i] = dummy[i];
	interpreter->Invoke();

	BOOT_LOG("Tflite OK");
}
