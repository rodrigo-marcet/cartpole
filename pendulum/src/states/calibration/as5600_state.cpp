#include "src/states/calibration/as5600_state.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/utils/as5600.h"
#include "src/utils/log_macros.h"

constexpr double STABILITY_DURATION_US = 10000000.0;
constexpr double LOW_PASS_ALPHA = 0.5;
constexpr double VELOCITY_THRESHOLD = 0.4; // rad/s

SequenceStatus as5600_calibration(AS5600CalibrationResult *result) {

	static unsigned long last_sample_time = micros();
	unsigned long t = micros();
	unsigned long dt = t - last_sample_time;

	if (dt < 100000) {
		return SequenceStatus::RUNNING;
	}

	last_sample_time = t;

	static double angle_rad = 0;
	static double angle_rad_prev = 0;
	static double cos_angle = 0, cos_angle_prev = 0;
	static double sin_angle = 0, sin_angle_prev = 0;
	static double angular_vel_filtered = 0;

	static double avg_raw_angle = 0;
	static int avg_sample_count = 0;

	static unsigned long stable_since_us = 0;

	int raw_angle = as5600_read_raw();
	angle_rad_prev = angle_rad;
	angle_rad = raw_angle * (2.0 * PI / 4096.0);

	cos_angle_prev = cos_angle;
	sin_angle_prev = sin_angle;
	cos_angle = cos(angle_rad);
	sin_angle = sin(angle_rad);

	double dt_s = dt / 1000000.0;
	double angular_vel = ((sin_angle - sin_angle_prev) * cos_angle - (cos_angle - cos_angle_prev) * sin_angle) / dt_s;

	// angular_vel_filtered = LOW_PASS_ALPHA * angular_vel
	//                      + (1.0 - LOW_PASS_ALPHA) * angular_vel_filtered;

	LOOP_LOG("[CALIBRATION] [AS5600] raw_angle : %i, angular_vel: %.2f, cos: %.2f, sin: %.2f", raw_angle, angular_vel,
	         cos_angle, sin_angle);
	bool is_stable = abs(angular_vel) < VELOCITY_THRESHOLD;
	if (!is_stable) {
		stable_since_us = t;
		avg_sample_count = 0;
		avg_raw_angle = 0;
	} else {
		if (stable_since_us == 0)
			stable_since_us = t;

		avg_sample_count++;
		avg_raw_angle += (raw_angle - avg_raw_angle) / avg_sample_count;

		if (t - stable_since_us >= STABILITY_DURATION_US) {
			int offset = static_cast<int>(avg_raw_angle);
			result->raw_offset = offset;
			LOOP_LOG("[CALIBRATION] [AS5600] stable for %lu", t - stable_since_us);
			LOOP_LOG("[CALIBRATION] [AS5600] offset set to %i", offset);
			return SequenceStatus::DONE;
		}
	}

	return SequenceStatus::RUNNING;
}
