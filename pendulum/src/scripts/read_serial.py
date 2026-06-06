import serial
from datetime import datetime

# =========================================================
# CONFIGURATION
# =========================================================

SERIAL_PORT = "COM3"      # Change to your Arduino port
BAUD_RATE = 115200        # Must match Serial.begin(...)
OUTPUT_FILE = "arduino_log.txt"

# =========================================================
# OPEN SERIAL PORT
# =========================================================

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

print(f"Connected to {SERIAL_PORT}")
print(f"Saving data to {OUTPUT_FILE}")
print("Press Ctrl+C to stop.\n")

# Give Arduino time to reset after opening serial
ser.reset_input_buffer()

# =========================================================
# LOG DATA
# =========================================================

try:
    with open(OUTPUT_FILE, "a", buffering=1) as f:
        f.write(
            f"\n\n========== Session started {datetime.now()} ==========\n"
        )

        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if line:
                print(line)
                f.write(line + "\n")

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    ser.close()
    print("Serial port closed.")
