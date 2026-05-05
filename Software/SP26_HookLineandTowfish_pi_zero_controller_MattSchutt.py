#!/usr/bin/env python3
"""
Pi Zero controller/logger.

Workflow:
1. L1 turns on for 1.6 s.
2. L1 turns off.
3. Wait GAP_AFTER_L1_MS.
4. L2 turns on for 1.6 s.
5. At L2 on, tell the Pico to measure over UART0.
6. While L2 is on, collect IMU samples on the Zero for the full measurement.
7. After L2 off, read frequency/GPS from the Pico, correct frequency from the
   orientation change during the measurement, convert to magnetic field, and
   append the row to CSV.
8. Wait 2 s and repeat.
"""

import csv
import math
import time
from pathlib import Path

import serial

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus
    except ImportError:
        SMBus = None


# User parameters
L1_PIN = 17
L2_PIN = 27

L1_ON_S = 1.6
L2_ON_S = 1.6  # Match the single measurement duration
GAP_AFTER_L1_MS = 0
POST_CYCLE_WAIT_S = 2.0

UART_PORT = "/dev/serial0"
UART_BAUD = 9600
UART_TIMEOUT_S = 5.0  # Increased to accommodate 1.6s measurement + buffer

CSV_PATH = Path("magnetometer_log.csv")
IMU_SAMPLE_HZ = 100
I2C_BUS = 1
MPU6050_ADDR = 0x68
MAX_TILT_DEG = 45.0
IMU_BASELINE_SAMPLES = 5

GAMMA_P = 2.67522e8


# Hardware helpers
class LedOutput:
    def __init__(self, pin):
        self.pin = pin

    def setup(self):
        if GPIO is None:
            print("RPi.GPIO not installed; LED GPIO calls will be printed only.")
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

    def high(self):
        if GPIO is None:
            print(f"GPIO {self.pin} HIGH")
        else:
            GPIO.output(self.pin, GPIO.HIGH)

    def low(self):
        if GPIO is None:
            print(f"GPIO {self.pin} LOW")
        else:
            GPIO.output(self.pin, GPIO.LOW)


class ImuReader:
    """
    GY-521 / MPU-6050 reader.

    Assumes Pi Zero I2C bus 1 and address 0x68. If AD0 is tied high, change
    MPU6050_ADDR to 0x69.
    """

    def __init__(self):
        self.bus = None
        self.enabled = False

    def setup(self):
        if SMBus is None:
            print("smbus/smbus2 not installed; IMU rows will be blank.")
            return

        self.bus = SMBus(I2C_BUS)
        self.bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)  # wake sensor
        self.bus.write_byte_data(MPU6050_ADDR, 0x1A, 0x03)  # DLPF ~44 Hz accel, ~42 Hz gyro
        self.bus.write_byte_data(MPU6050_ADDR, 0x19, 0x09)  # 100 Hz sample rate
        self.bus.write_byte_data(MPU6050_ADDR, 0x1C, 0x00)  # accel +/-2 g
        self.bus.write_byte_data(MPU6050_ADDR, 0x1B, 0x00)  # gyro +/-250 deg/s
        self.enabled = True

    def read_sample(self):
        if not self.enabled:
            return {
                "ax": "",
                "ay": "",
                "az": "",
                "gx": "",
                "gy": "",
                "gz": "",
                "temp_c": "",
            }

        data = self.bus.read_i2c_block_data(MPU6050_ADDR, 0x3B, 14)

        ax_raw = self._word(data[0], data[1])
        ay_raw = self._word(data[2], data[3])
        az_raw = self._word(data[4], data[5])
        temp_raw = self._word(data[6], data[7])
        gx_raw = self._word(data[8], data[9])
        gy_raw = self._word(data[10], data[11])
        gz_raw = self._word(data[12], data[13])

        return {
            "ax": ax_raw / 16384.0,
            "ay": ay_raw / 16384.0,
            "az": az_raw / 16384.0,
            "gx": gx_raw / 131.0,
            "gy": gy_raw / 131.0,
            "gz": gz_raw / 131.0,
            "temp_c": (temp_raw / 340.0) + 36.53,
        }

    @staticmethod
    def _word(msb, lsb):
        value = (msb << 8) | lsb
        if value & 0x8000:
            value -= 65536
        return value


# Math and logging
def frequency_to_field_nt(freq_hz):
    if freq_hz <= 0:
        return None
    return (2.0 * math.pi * freq_hz / GAMMA_P) * 1e9


def mean_numeric(samples, key):
    values = []
    for sample in samples:
        try:
            value = sample.get(key)
            if value != "":
                values.append(float(value))
        except (TypeError, ValueError):
            pass
    return sum(values) / len(values) if values else None


def estimate_roll_pitch_deg(imu_samples):
    ax = mean_numeric(imu_samples, "ax")
    ay = mean_numeric(imu_samples, "ay")
    az = mean_numeric(imu_samples, "az")

    if ax is None or ay is None or az is None:
        return None, None

    roll_rad = math.atan2(ay, az)
    pitch_rad = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return math.degrees(roll_rad), math.degrees(pitch_rad)


def sample_roll_pitch_deg(sample):
    try:
        ax = float(sample.get("ax"))
        ay = float(sample.get("ay"))
        az = float(sample.get("az"))
    except (TypeError, ValueError):
        return None, None

    roll_rad = math.atan2(ay, az)
    pitch_rad = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return math.degrees(roll_rad), math.degrees(pitch_rad)


def orientation_samples_deg(imu_samples):
    orientations = []
    for sample in imu_samples:
        roll_deg, pitch_deg = sample_roll_pitch_deg(sample)
        if roll_deg is not None and pitch_deg is not None:
            orientations.append((roll_deg, pitch_deg))
    return orientations


def mean_pair(pairs):
    if not pairs:
        return None, None
    return (
        sum(pair[0] for pair in pairs) / len(pairs),
        sum(pair[1] for pair in pairs) / len(pairs),
    )


def angle_delta_deg(angle_deg, baseline_deg):
    return (angle_deg - baseline_deg + 180.0) % 360.0 - 180.0


def orientation_window_correction(imu_samples):
    """
    Build the field correction from relative orientation during the measurement.

    The first few samples define the start orientation. Every later sample is
    compared to that baseline, and the average projection factor is used because
    the Pico frequency is collected over the whole L2 window.
    """
    orientations = orientation_samples_deg(imu_samples)
    if not orientations:
        return None, None, None, "no_imu_delta"

    baseline_count = min(IMU_BASELINE_SAMPLES, len(orientations))
    baseline_roll, baseline_pitch = mean_pair(orientations[:baseline_count])

    projection_factors = []
    delta_roll_values = []
    delta_pitch_values = []

    for roll_deg, pitch_deg in orientations:
        delta_roll = angle_delta_deg(roll_deg, baseline_roll)
        delta_pitch = angle_delta_deg(pitch_deg, baseline_pitch)

        if abs(delta_roll) > MAX_TILT_DEG or abs(delta_pitch) > MAX_TILT_DEG:
            return None, delta_roll, delta_pitch, "large_orientation_delta"

        delta_roll_values.append(delta_roll)
        delta_pitch_values.append(delta_pitch)

        factor = tilt_correction_factor(delta_roll, delta_pitch)
        if factor is None:
            return None, delta_roll, delta_pitch, "invalid_orientation_delta"
        projection_factors.append(factor)

    if not projection_factors:
        return None, None, None, "no_imu_delta"

    correction_factor = sum(projection_factors) / len(projection_factors)
    mean_delta_roll = sum(delta_roll_values) / len(delta_roll_values)
    mean_delta_pitch = sum(delta_pitch_values) / len(delta_pitch_values)
    return correction_factor, mean_delta_roll, mean_delta_pitch, "ok"


def tilt_correction_factor(roll_deg, pitch_deg):
    if roll_deg is None or pitch_deg is None:
        return None

    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    factor = math.cos(pitch_rad) * math.cos(roll_rad)
    
    # Reject negative or near-zero factors (sensor upside down or on edge)
    if factor <= 0:
        return None
    
    return factor


def correct_frequency(freq_hz, imu_samples):
    """
    Differential-orientation correction for a single-axis, Z-oriented sensor.

    The Pico returns one frequency integrated over the measurement window, so
    IMU samples from that whole window are converted into roll/pitch deltas
    relative to the start of the measurement. Since B is proportional to
    frequency, the averaged projection factor can be applied to frequency before
    converting frequency to magnetic field.
    """
    factor, delta_roll_deg, delta_pitch_deg, status = orientation_window_correction(imu_samples)

    if factor is None or abs(factor) < 1e-6:
        return freq_hz, delta_roll_deg, delta_pitch_deg, factor, status

    return freq_hz / factor, delta_roll_deg, delta_pitch_deg, factor, "ok"


def collect_imu_for_window(imu, duration_s):
    samples = []
    sample_period_s = 1.0 / IMU_SAMPLE_HZ
    end_t = time.monotonic() + duration_s

    while time.monotonic() < end_t:
        sample_t = time.time()
        try:
            sample = imu.read_sample()
        except Exception as exc:
            sample = {"error": str(exc)}
        sample["timestamp"] = f"{sample_t:.6f}"
        samples.append(sample)
        time.sleep(sample_period_s)

    return samples


def ensure_csv(path):
    if path.exists():
        return
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "lat",
            "lon",
            "magnetic_field_nt",
        ])


def append_csv(path, row):
    with path.open("a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)


# Pico UART
def parse_pico_response(line):
    # Expected: DATA <lat> <lon> <freq_hz>
    parts = line.strip().split()
    if len(parts) != 4 or parts[0] != "DATA":
        raise ValueError(f"Unexpected Pico response: {line!r}")

    lat = parts[1]
    lon = parts[2]
    freq_hz = float(parts[3])

    return lat, lon, freq_hz


def wait_for_pico_data(uart):
    deadline = time.monotonic() + UART_TIMEOUT_S
    print(f"Waiting for Pico data (timeout={UART_TIMEOUT_S}s)...")
    while time.monotonic() < deadline:
        line = uart.readline().decode("utf-8", "ignore").strip()
        if line:
            print(f"Pico UART received: {line!r}")
        if line.startswith("DATA "):
            return parse_pico_response(line)
    raise TimeoutError("Timed out waiting for Pico DATA response.")


def main():
    l1 = LedOutput(L1_PIN)
    l2 = LedOutput(L2_PIN)
    imu = ImuReader()

    l1.setup()
    l2.setup()
    imu.setup()
    ensure_csv(CSV_PATH)

    uart = serial.Serial(UART_PORT, UART_BAUD, timeout=0.05)
    time.sleep(1.0)
    uart.reset_input_buffer()

    gap_after_l1_s = GAP_AFTER_L1_MS / 1000.0
    cycle = 0

    try:
        while True:
            cycle += 1
            print(f"Cycle {cycle}: L1 on")
            l1.high()
            l2.low()
            time.sleep(L1_ON_S)

            print(f"Cycle {cycle}: L1 off, gap {GAP_AFTER_L1_MS} ms")
            l1.low()
            if gap_after_l1_s > 0:
                time.sleep(gap_after_l1_s)

            print(f"Cycle {cycle}: L2 on, measuring")
            l2.high()
            uart.write(f"MEASURE {int(L2_ON_S * 1000)}\n".encode("ascii"))
            uart.flush()

            imu_samples = collect_imu_for_window(imu, L2_ON_S)
            l2.low()

            lat, lon, raw_freq = wait_for_pico_data(uart)
            print(f"Raw frequency: {raw_freq}")
            corrected_freq, roll_deg, pitch_deg, factor, status = correct_frequency(
                raw_freq,
                imu_samples,
            )
            print(
                "Corrected: "
                f"freq={corrected_freq}, delta_roll={roll_deg}, "
                f"delta_pitch={pitch_deg}, factor={factor}, status={status}"
            )
            field_nt = frequency_to_field_nt(corrected_freq)

            row = [
                lat,
                lon,
                f"{field_nt:.9f}" if field_nt is not None else "",
            ]
            append_csv(CSV_PATH, row)
            field_text = f"{field_nt:.3f} nT" if field_nt is not None else "invalid field"
            print(f"Cycle {cycle}: {lat},{lon} {field_text}")

            time.sleep(POST_CYCLE_WAIT_S)

    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        l1.low()
        l2.low()
        uart.close()
        if GPIO is not None:
            GPIO.cleanup()


if __name__ == "__main__":
    main()
