from micropython import const
import rp2
from rp2 import PIO, asm_pio
from machine import Pin, UART
import utime
import uarray as array


# Onboard LED blink for confirming Pico power from the Pi Zero during debugging.
try:
    ONBOARD_LED = Pin("LED", Pin.OUT)
except Exception:
    ONBOARD_LED = Pin(25, Pin.OUT)

ONBOARD_LED.value(0)
LED_BLINK_MS = const(1000)
last_led_blink_ms = utime.ticks_ms()


def blink_onboard_led():
    global last_led_blink_ms

    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_led_blink_ms) >= LED_BLINK_MS:
        ONBOARD_LED.toggle()
        last_led_blink_ms = now


# PIO frequency measurement
@asm_pio(sideset_init=PIO.OUT_HIGH)
def gate():
    mov(x, osr)
    wait(0, pin, 0)
    wait(1, pin, 0)
    label("loopstart")
    jmp(x_dec, "loopstart") .side(0)
    wait(0, pin, 0)
    wait(1, pin, 0) .side(1)
    irq(block, 0)
    wait(1, irq, 4)
    wait(1, irq, 5)


@asm_pio()
def clock_count():
    mov(x, osr)
    wait(1, pin, 0)
    wait(0, pin, 0)
    label("counter")
    jmp(pin, "output")
    jmp(x_dec, "counter")
    label("output")
    mov(isr, x)
    push()
    irq(block, 4)


@asm_pio(sideset_init=PIO.OUT_HIGH)
def pulse_count():
    mov(x, osr)
    wait(1, pin, 0)
    wait(0, pin, 0) .side(0)
    label("counter")
    wait(0, pin, 1)
    wait(1, pin, 1)
    jmp(pin, "output")
    jmp(x_dec, "counter")
    label("output")
    mov(isr, x) .side(1)
    push()
    irq(block, 5)


MAX_COUNT = const((1 << 32) - 1)
PIO_FREQ = 125_000_000

INPUT_PIN = Pin(15, Pin.IN)
GATE_PIN = Pin(14, Pin.OUT)
PULSE_FIN_PIN = Pin(13, Pin.OUT)

GATE_PIN.value(1)
PULSE_FIN_PIN.value(1)

sm0 = rp2.StateMachine(0, gate, freq=PIO_FREQ, in_base=INPUT_PIN, sideset_base=GATE_PIN)
sm1 = rp2.StateMachine(1, clock_count, freq=PIO_FREQ, in_base=GATE_PIN, jmp_pin=PULSE_FIN_PIN)
sm2 = rp2.StateMachine(2, pulse_count, freq=PIO_FREQ, in_base=GATE_PIN, sideset_base=PULSE_FIN_PIN, jmp_pin=GATE_PIN)

_freq_data = array.array("I", [0, 0])
_freq_ready = False


def _counter_handler(sm):
    global _freq_ready
    if not _freq_ready:
        sm0.put(125_000)
        sm0.exec("pull()")
        _freq_data[0] = sm1.get()
        _freq_data[1] = sm2.get()
        _freq_ready = True


def measure_frequency(collection_time_ms):
    global _freq_ready

    gate_ticks = int(PIO_FREQ * collection_time_ms / 1000)
    _freq_ready = False

    sm1.active(0)
    sm2.active(0)
    sm0.active(0)

    while sm0.rx_fifo():
        sm0.get()
    while sm1.rx_fifo():
        sm1.get()
    while sm2.rx_fifo():
        sm2.get()

    sm0.put(gate_ticks)
    sm0.exec("pull()")
    sm1.put(MAX_COUNT)
    sm1.exec("pull()")
    sm2.put(MAX_COUNT - 1)
    sm2.exec("pull()")

    sm0.irq(_counter_handler)
    sm1.active(1)
    sm2.active(1)
    sm0.active(1)

    timeout = collection_time_ms * 3
    start = utime.ticks_ms()
    while not _freq_ready:
        blink_onboard_led()
        if utime.ticks_diff(utime.ticks_ms(), start) > timeout:
            return -1.0
        utime.sleep_ms(1)

    clock_ticks = 2 * (MAX_COUNT - _freq_data[0] + 1)
    pulse_ticks = MAX_COUNT - _freq_data[1]
    return pulse_ticks * (PIO_FREQ / clock_ticks)


# UART and GPS
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
    return raw.decode("utf-8", "ignore").strip()


def command_duration_ms(command):
    # Expected command: MEASURE 1400
    parts = command.split()
    if not parts or parts[0] != "MEASURE":
        return None
    if len(parts) == 1:
        return 1600  # Single 1.6 second measurement
    try:
        return max(10, int(parts[1]))
    except ValueError:
        return None


print("PICO READY")

while True:
    blink_onboard_led()
    drain_gps(5)

    command = read_command()
    duration_ms = command_duration_ms(command) if command else None

    if duration_ms is not None:
        drain_gps(50)
        freq_hz = measure_frequency(duration_ms)
        drain_gps(100)

        response = "DATA {} {} {:.12f}\n".format(
            last_lat,
            last_lon,
            freq_hz,
        )
        uart_zero.write(response)
        print(response.strip())

    utime.sleep_ms(5)
