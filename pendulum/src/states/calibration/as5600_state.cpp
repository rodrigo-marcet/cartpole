#include "as5600_state.h"

#include <Arduino.h>
#include <Wire.h>

#include "../../utils/as5600.h"

bool as5600_calibration() {
	static double rads = 0;
	static double rads_p = 0;
	static double avg_position = 0;
	static int n = 0;
	static double c = 0, c_p = 0;
	static double s = 0, s_p = 0;
	static double w_filtered = 0;
	static double epsilon = 4;

	static unsigned long last = micros();
	static unsigned long stable_since = micros();

	unsigned long now = micros();
	unsigned long dt = now - last;
	last = now;

	if (dt == 0)
		return false;

	int raw_angle = as5600_read_raw();
	rads_p = rads;
	rads = raw_angle * (2.0f * PI / 4096.0f);
	c_p = c;
	c = cos(rads);
	s_p = s;
	s = sin(rads);

	double dts = dt / 1000000.0;

	double w = ((s - s_p) * c - (c - c_p) * s) / dts;

	double alpha = 0.5;
	w_filtered = alpha * w + (1 - alpha) * w_filtered;

	if (now - stable_since >= 5000000 && abs(w_filtered) < epsilon) {
		set_as5600_offset(int(avg_position));
		return true;
	} else if (abs(w_filtered) >= epsilon) {
		stable_since = now;
		n = 0;
		avg_position = 0;
	} else {
		n++;
		avg_position = avg_position + (raw_angle - avg_position) / n;
	}

	return false;
}
