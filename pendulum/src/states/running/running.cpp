#include "running.h"

#include <Arduino.h>
#include <Wire.h>

#include "../../utils/as5600.h"

SequenceState running_sequence(){
	double rads = as5600_read_rads();
	double c = cos(rads);

	Serial.print(get_as5600_offset());
	Serial.print("         " );
	Serial.print(c, 6);
	Serial.print("\n");

	return SequenceState::Running;
}
