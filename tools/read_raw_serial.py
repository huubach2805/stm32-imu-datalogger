"""Print raw bytes from the serial port - useful to check the STM32 is sending."""
import argparse
import time

import serial

p = argparse.ArgumentParser()
p.add_argument("--port", default="COM17")
p.add_argument("--baud", type=int, default=115200)
args = p.parse_args()

ser = serial.Serial(args.port, args.baud, timeout=0.1)
print("Listening raw data on", args.port)

last_time = time.time()

try:
    while True:
        data = ser.read(ser.in_waiting or 1)

        if data:
            print("RAW:", data)
            last_time = time.time()
        elif time.time() - last_time > 1:
            print("No data...")
            last_time = time.time()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
