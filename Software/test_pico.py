# Test Pico - sends test messages without using PIO
# Use this to verify UART communication is working

from micropython import const
from machine import Pin, UART
import utime


# ----------------------- Status LED -----------------------
try:
    STATUS_LED = Pin("LED", Pin.OUT)
except Exception:
    STATUS_LED = Pin(25, Pin.OUT)

STATUS_LED.value(0)
HEARTBEAT_MS = const(1000)
last_heartbeat_ms = utime.ticks_ms()


def update_status_led():
    global last_heartbeat_ms

    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_heartbeat_ms) >= HEARTBEAT_MS:
        STATUS_LED.toggle()
        last_heartbeat_ms = now


# ----------------------- UART and GPS -----------------------
# UART0 = Pi Zero: Pico GP0 TX -> Zero RX, Pico GP1 RX <- Zero TX
# UART1 = GT-U7 GPS: Pico GP5 RX <- GPS TX. Pico GP4 TX is available if needed.
uart_zero = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1), timeout=50)
uart_gps = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), timeout=50)

gps_buffer = b""
last_lat = "0.000000"
last_lon = "0.000000"


def parse_gprmc(sentence):
    try:
        parts = sentence.split(",")
        if parts[0] not in ("$GPRMC", "$GNRMC") or parts[2] != "A":
            return None

        raw_lat = parts[3]
        lat_dir = parts[4]
        raw_lon = parts[5]
        lon_dir = parts[6]

        lat_deg = int(float(raw_lat) / 100)
        lat = lat_deg + (float(raw_lat) - lat_deg * 100) / 60.0
        if lat_dir == "S":
            lat = -lat

        lon_deg = int(float(raw_lon) / 100)
        lon = lon_deg + (float(raw_lon) - lon_deg * 100) / 60.0
        if lon_dir == "W":
            lon = -lon

        return f"{lat:.6f}", f"{lon:.6f}"
    except Exception:
        return None


def drain_gps(max_ms=25):
    global gps_buffer, last_lat, last_lon

    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < max_ms:
        if not uart_gps.any():
            utime.sleep_ms(1)
            continue

        data = uart_gps.read()
        if not data:
            continue

        gps_buffer += data
        while b"\n" in gps_buffer:
            line, gps_buffer = gps_buffer.split(b"\n", 1)
            decoded = line.decode("utf-8", "ignore").strip()
            parsed = parse_gprmc(decoded)
            if parsed:
                last_lat, last_lon = parsed


def read_command():
    if not uart_zero.any():
        return None
    raw = uart_zero.readline()
    if not raw:
        return None
    try:
        decoded = raw.decode("utf-8", "ignore").strip()
        if not decoded:
            return None
        return decoded
    except Exception as e:
        # Discard bad data and continue
        return None


def command_duration_ms(command):
    # Expected command: MEASURE 1400
    parts = command.split()
    if not parts or parts[0] != "MEASURE":
        return None
    if len(parts) == 1:
        return 1600
    try:
        return max(10, int(parts[1]))
    except ValueError:
        return None


# Test message cycling for debugging
test_cycle = 0
TEST_MESSAGES = [
    "DATA 40.712800 -74.006000 1234567.890123",
    "DATA 34.052200 -118.243700 9876543.210987",
    "DATA 51.507400 -0.127800 5555555.555555",
]


print("TEST PICO READY")

while True:
    update_status_led()
    drain_gps(5)

    command = read_command()
    if command:
        print(f"Received command: {command}")
    duration_ms = command_duration_ms(command) if command else None

    if duration_ms is not None:
        print(f"Processing MEASURE command, duration={duration_ms}ms")
        drain_gps(50)
        
        # Simulate measurement delay (shorter than real measurement)
        utime.sleep_ms(100)
        
        # Send test message cycling through different values
        response = TEST_MESSAGES[test_cycle % len(TEST_MESSAGES)] + "\n"
        test_cycle += 1
        
        uart_zero.write(response)
        print("TEST: " + response.strip())
        print("Response sent!")

    utime.sleep_ms(5)