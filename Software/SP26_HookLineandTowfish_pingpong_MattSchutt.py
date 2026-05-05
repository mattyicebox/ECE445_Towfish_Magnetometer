from machine import UART, Pin
import time

# UART1 = GPS (GT-U7)
# GP4 = TX (Pico -> GPS not used usually)
# GP5 = RX (GPS -> Pico data)
uart_gps = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

# UART0 = Pi Zero comms
# GP0 = TX (Pico -> Pi Zero)
# GP1 = RX (Pi Zero -> Pico)
uart_zero = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

print("GPS RAW DUMP + Pi Zero COMMS STARTED")
buffer = b""
count = 0
time.sleep(5)  # Wait for Pi Zero to boot

while True:
    # Read GPS data
    if uart_gps.any():
        data = uart_gps.read()
        if data:
            buffer += data
            try:
                print(data.decode('utf-8', 'ignore'), end="")
            except:
                print(data)
    
    # Handle Pi Zero ping-pong
    if uart_zero.any():
        line = uart_zero.readline().decode('utf-8', 'ignore').strip()
        if line.startswith("PING"):
            parts = line.split()
            if len(parts) == 2:
                ping_count = parts[1]
                response = f"PONG {ping_count}\n"
                uart_zero.write(response)
                print(f"[PICO RESPONSE] {response.strip()}")
    
    time.sleep(0.05)