"""
STM32 IMU Data Logger - PC side.

Reads lines "tick_ms,ax,ay,az,roll,pitch,yaw,gx,gy,gz" from the STM32
over UART, plots them in real time and logs them to CSV.

Usage:
    python main.py                      # uses SERIAL_PORT from config.py
    python main.py --port COM5
    python main.py --port /dev/ttyUSB0 --verbose
"""
import argparse
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import config
from imu_parser import parse_sensor_data, is_error_message
from logger import DataLogger
from plotter import Plotter
from serial_reader import SerialReader


def parse_args():
    p = argparse.ArgumentParser(description="STM32 IMU real-time plotter & CSV logger")
    p.add_argument("--port", default=config.SERIAL_PORT, help="serial port (default: %(default)s)")
    p.add_argument("--baud", type=int, default=config.BAUD_RATE, help="baud rate (default: %(default)s)")
    p.add_argument("--interval", type=int, default=config.UPDATE_INTERVAL, help="plot refresh in ms (default: %(default)s)")
    p.add_argument("--verbose", action="store_true", help="print every received line")
    return p.parse_args()


def main():
    args = parse_args()

    reader = SerialReader(args.port, args.baud, config.SERIAL_TIMEOUT)
    if not reader.connect():
        return 1

    plotter = Plotter()
    logger = DataLogger()
    print(f"Logging to {logger.path}")
    print("Waiting for STM32 data... (close the plot window to stop)")

    def update(frame):
        new_data = False

        # Read ALL lines received since the last frame, so the plot
        # does not fall behind the 50 Hz stream from the STM32.
        for line in reader.read_lines():
            if args.verbose:
                print("STM32:", line)

            if is_error_message(line):
                print("[STM32 ERROR]", line[4:])
                continue

            sample = parse_sensor_data(line)
            if sample is None:
                print("Invalid data:", line)
                continue

            plotter.add_data(sample)
            logger.save(sample)
            new_data = True

        if new_data:
            plotter.refresh()

        return plotter.artists()

    ani = FuncAnimation(  # noqa: F841 - must keep a reference
        plotter.fig,
        update,
        interval=args.interval,
        cache_frame_data=False,
    )

    try:
        plt.tight_layout()
        plt.show()
    finally:
        reader.disconnect()
        logger.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
