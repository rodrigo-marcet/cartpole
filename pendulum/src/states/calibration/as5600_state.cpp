#include "src/states/calibration/as5600_state.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/utils/as5600.h"
#include "src/utils/log_macros.h"

constexpr double STABILITY_DURATION_US = 5'000'000.0;
constexpr double LOW_PASS_ALPHA = 0.5;
constexpr double VELOCITY_THRESHOLD = 0.4; // rad/s

SequenceStatus as5600_calibration(AS5600CalibrationResult *result) {
	static double angle_rad = 0;
	static double angle_rad_prev = 0;
	static double cos_angle = 0, cos_angle_prev = 0;
	static double sin_angle = 0, sin_angle_prev = 0;
	static double angular_vel_filtered = 0;

	static double avg_raw_angle = 0;
	static int avg_sample_count = 0;

	static unsigned long last_us = micros();
	static unsigned long stable_since_us = micros();

	unsigned long now_us = micros();
	unsigned long dt_us = now_us - last_us;
	last_us = now_us;

	if (dt_us == 0)
		return SequenceStatus::RUNNING;

	int raw_angle = as5600_read_raw();
	angle_rad_prev = angle_rad;
	angle_rad = raw_angle * (2.0 * PI / 4096.0);

	cos_angle_prev = cos_angle;
	sin_angle_prev = sin_angle;
	cos_angle = cos(angle_rad);
	sin_angle = sin(angle_rad);

	double dt_s = dt_us / 1'000'000.0;
	double angular_vel = ((sin_angle - sin_angle_prev) * cos_angle - (cos_angle - cos_angle_prev) * sin_angle) / dt_s;

	// angular_vel_filtered = LOW_PASS_ALPHA * angular_vel
	//                      + (1.0 - LOW_PASS_ALPHA) * angular_vel_filtered;

	bool is_stable = abs(angular_vel) < VELOCITY_THRESHOLD;

	if (!is_stable) {
		stable_since_us = now_us;
		avg_sample_count = 0;
		avg_raw_angle = 0;
	} else {
		avg_sample_count++;
		avg_raw_angle += (raw_angle - avg_raw_angle) / avg_sample_count;

		if (now_us - stable_since_us >= STABILITY_DURATION_US) {
			int offset = static_cast<int>(avg_raw_angle);
			result->raw_offset = offset;
			LOOP_LOG("AS5600 offset set to %d", offset);
			return SequenceStatus::DONE;
		}
	}

	return SequenceStatus::RUNNING;
}
