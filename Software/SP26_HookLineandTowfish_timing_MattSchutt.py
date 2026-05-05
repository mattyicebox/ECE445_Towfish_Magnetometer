import machine
import utime

uart = machine.UART(0, baudrate=9600, tx=machine.Pin(0), rx=machine.Pin(1), timeout=100)
print("GT-U7 Raw Monitor Starting...")

last_rx_time = utime.ticks_ms()

while True:
    now = utime.ticks_ms()

    if uart.any():
        line = uart.readline()
        last_rx_time = now
        if line:
            try:
                print(line.decode().strip())
            except:
                print("RAW BYTES:", line)

    if utime.ticks_diff(now, last_rx_time) > 3000:
        print("NO SIGNAL")
        last_rx_time = now