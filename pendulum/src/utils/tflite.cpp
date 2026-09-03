#include "src/utils/tflite.h"

void scale_observations(float *obs, const int n) {
	for (int i = 0; i < n; i++) {
		float scaled = (obs[i] - MODEL_INPUT_MEAN[i]) / (sqrtf(MODEL_INPUT_VAR[i]) + 1e-8f);
		// obs[i] = fminf(fmaxf(scaled, -5.0), 5.0); //Uncomment if used trained scales
		obs[i] = scaled;
	}
}
