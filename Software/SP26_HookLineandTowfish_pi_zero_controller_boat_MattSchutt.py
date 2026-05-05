#!/usr/bin/env python3
"""
Modified Pi Zero controller/logger for the boat integration.

Workflow:
1. Run the normal L1/L2 measurement cycle from the standard controller.
2. Once the Pico sends real GPS coordinates, tell the boat "GO".
3. If the boat sends "charging", stop the magnetometer cycle and poll GPS
   every 10 s instead. This lets the boat sit still and charge without us
   hammering the measurement hardware.

This file intentionally does not edit the standard controller. It imports the
boring helper pieces from there so the old controller can stay as a clean
known-good reference.
"""

import time
from pathlib import Path

import serial

from SP26_HookLineandTowfish_pi_zero_controller_MattSchutt import (
    CSV_PATH,
    GAP_AFTER_L1_MS,
    GPIO,
    IMU_SAMPLE_HZ,
    L1_ON_S,
    L1_PIN,
    L2_ON_S,
    L2_PIN,
    POST_CYCLE_WAIT_S,
    UART_BAUD,
    UART_PORT,
    UART_TIMEOUT_S,
    ImuReader,
    LedOutput,
    append_csv,
    collect_imu_for_window,
    correct_frequency,
    ensure_csv,
    frequency_to_field_nt,
    parse_pico_response,
)


# Boat UART knobs
# UART0 still goes to the Pico. This one is for the boat's software.
# Change this to whatever Linux names your second UART after enabling it in
# config.txt. /dev/ttyAMA1 is a common Pi Zero 2 W answer, but check with
# ls /dev/ttyAMA* /dev/serial* if Linux decides to be helpful.
BOAT_UART_PORT = "/dev/ttyAMA1"
BOAT_UART_BAUD = 9600
BOAT_UART_TIMEOUT_S = 0.05

# Simple text protocol for the boat side. Newline terminated, because life is
# already hard enough without binary packet framing for two words.
BOAT_GO_MESSAGE = "GO\n"
BOAT_CHARGING_COMMANDS = {"charging", "charge", "idle_charge"}
BOAT_RUN_COMMANDS = {"run", "resume", "measuring", "measure"}


# Idle charge knobs
GPS_IDLE_POLL_S = 10.0
GPS_ONLY_TIMEOUT_S = 3.0


# Small protocol helpers
def valid_gps_fix(lat, lon):
    """
    Treat 0,0 as no fix because the Pico starts with that as its placeholder.
    This is not a perfect world-map opinion, just a practical startup check.
    """
    try:
        return abs(float(lat)) > 1e-6 or abs(float(lon)) > 1e-6
    except (TypeError, ValueError):
        return False


def send_boat_go_once(boat_uart, gps_go_sent, lat, lon):
    # GO is latched so we don't spam the boat every single measurement cycle.
    if gps_go_sent or not valid_gps_fix(lat, lon):
        return gps_go_sent

    boat_uart.write(BOAT_GO_MESSAGE.encode("ascii"))
    boat_uart.flush()
    print(f"Boat UART sent: {BOAT_GO_MESSAGE.strip()!r} after GPS fix {lat},{lon}")
    return True


def read_boat_command(boat_uart):
    if not boat_uart.in_waiting:
        return None

    raw = boat_uart.readline()
    if not raw:
        return None

    command = raw.decode("utf-8", "ignore").strip().lower()
    if command:
        print(f"Boat UART received: {command!r}")
    return command or None


def update_charge_state(boat_uart, idle_charging):
    """
    Boat owns this state. If it says charging, we park measurements. If it says
    run/resume/etc, we go back to the normal survey cycle.
    """
    command = read_boat_command(boat_uart)
    if command in BOAT_CHARGING_COMMANDS:
        if not idle_charging:
            print("Boat requested charging mode; stopping measurements.")
        return True
    if command in BOAT_RUN_COMMANDS:
        if idle_charging:
            print("Boat requested measurement mode; resuming cycles.")
        return False
    return idle_charging


def parse_gps_only_response(line):
    # Expected from pico_measure_uart_charge.py: GPS <lat> <lon>
    parts = line.strip().split()
    if len(parts) != 3 or parts[0] != "GPS":
        raise ValueError(f"Unexpected Pico GPS response: {line!r}")
    return parts[1], parts[2]


def wait_for_pico_line(uart, prefix, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = uart.readline().decode("utf-8", "ignore").strip()
        if line:
            print(f"Pico UART received: {line!r}")
        if line.startswith(prefix):
            return line
    raise TimeoutError(f"Timed out waiting for Pico {prefix.strip()} response.")


def wait_for_pico_data(uart):
    print(f"Waiting for Pico data (timeout={UART_TIMEOUT_S}s)...")
    line = wait_for_pico_line(uart, "DATA ", UART_TIMEOUT_S)
    return parse_pico_response(line)


def request_gps_only(uart):
    # Used during charging mode. The Pico just drains/parses GPS and returns
    # the latest fix; no PIO frequency measurement happens here.
    uart.write(b"GPS_ONLY\n")
    uart.flush()
    line = wait_for_pico_line(uart, "GPS ", GPS_ONLY_TIMEOUT_S)
    return parse_gps_only_response(line)


def sleep_with_boat_checks(boat_uart, seconds, idle_charging):
    """
    Sleep in small chunks so a charging command doesn't wait behind a whole
    post-cycle delay. Kinda clunky, but it keeps the control loop simple.
    """
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        idle_charging = update_charge_state(boat_uart, idle_charging)
        time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
    return idle_charging


def run_measurement_cycle(cycle, l1, l2, imu, pico_uart, boat_uart):
    gap_after_l1_s = GAP_AFTER_L1_MS / 1000.0

    print(f"Cycle {cycle}: L1 on")
    l1.high()
    l2.low()
    idle_charging = sleep_with_boat_checks(boat_uart, L1_ON_S, False)
    if idle_charging:
        l1.low()
        return None, None, True

    print(f"Cycle {cycle}: L1 off, gap {GAP_AFTER_L1_MS} ms")
    l1.low()
    if gap_after_l1_s > 0:
        idle_charging = sleep_with_boat_checks(boat_uart, gap_after_l1_s, False)
        if idle_charging:
            return None, None, True

    print(f"Cycle {cycle}: L2 on, measuring")
    l2.high()
    pico_uart.write(f"MEASURE {int(L2_ON_S * 1000)}\n".encode("ascii"))
    pico_uart.flush()

    imu_samples = collect_imu_for_window(imu, L2_ON_S)
    l2.low()

    lat, lon, raw_freq = wait_for_pico_data(pico_uart)
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

    return lat, lon, False


def main():
    l1 = LedOutput(L1_PIN)
    l2 = LedOutput(L2_PIN)
    imu = ImuReader()

    l1.setup()
    l2.setup()
    imu.setup()
    ensure_csv(Path(CSV_PATH))

    pico_uart = serial.Serial(UART_PORT, UART_BAUD, timeout=0.05)
    boat_uart = serial.Serial(BOAT_UART_PORT, BOAT_UART_BAUD, timeout=BOAT_UART_TIMEOUT_S)

    time.sleep(1.0)
    pico_uart.reset_input_buffer()
    boat_uart.reset_input_buffer()

    cycle = 0
    gps_go_sent = False
    idle_charging = False
    next_gps_poll_t = 0.0

    try:
        while True:
            idle_charging = update_charge_state(boat_uart, idle_charging)

            if idle_charging:
                l1.low()
                l2.low()

                now = time.monotonic()
                if now >= next_gps_poll_t:
                    print("Charging mode: polling GPS only.")
                    try:
                        lat, lon = request_gps_only(pico_uart)
                        gps_go_sent = send_boat_go_once(boat_uart, gps_go_sent, lat, lon)
                    except TimeoutError as exc:
                        print(exc)
                    next_gps_poll_t = now + GPS_IDLE_POLL_S

                time.sleep(0.1)
                continue

            cycle += 1
            lat, lon, idle_charging = run_measurement_cycle(
                cycle,
                l1,
                l2,
                imu,
                pico_uart,
                boat_uart,
            )
            if idle_charging:
                continue
            gps_go_sent = send_boat_go_once(boat_uart, gps_go_sent, lat, lon)
            idle_charging = sleep_with_boat_checks(boat_uart, POST_CYCLE_WAIT_S, idle_charging)

    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        l1.low()
        l2.low()
        pico_uart.close()
        boat_uart.close()
        if GPIO is not None:
            GPIO.cleanup()


if __name__ == "__main__":
    main()
