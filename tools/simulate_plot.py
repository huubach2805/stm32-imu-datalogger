"""
Demo without hardware: generate fake firmware lines (same format as the STM32)
and push them through the real parser + plotter.

Run from the repo root:  python tools/simulate_plot.py
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.animation import FuncAnimation       # noqa: E402

from imu_parser import parse_sensor_data             # noqa: E402
from plotter import Plotter                          # noqa: E402

PERIOD_MS = 20          # same 50 Hz rate as the firmware
tick = 0


def fake_firmware_line(t_ms):
    """Board slowly tilting around X, with some sensor noise."""
    t = t_ms / 1000.0
    roll = 30.0 * math.sin(2 * math.pi * 0.2 * t)                 # deg
    roll_rate = 30.0 * 2 * math.pi * 0.2 * math.cos(2 * math.pi * 0.2 * t)
    r = math.radians(roll)
    n = lambda s: random.gauss(0, s)                               # noqa: E731

    ax, ay, az = n(0.01), math.sin(r) + n(0.01), math.cos(r) + n(0.01)
    gx, gy, gz = roll_rate + n(0.5), n(0.5), n(0.5)
    return (f"{t_ms},{int(ax*1000)},{int(ay*1000)},{int(az*1000)},"
            f"{int(roll*100)},0,0,{int(gx*100)},{int(gy*100)},{int(gz*100)}")


plotter = Plotter()


def update(frame):
    global tick
    for _ in range(5):                 # 5 samples per 100 ms frame
        tick += PERIOD_MS
        plotter.add_data(parse_sensor_data(fake_firmware_line(tick)))
    plotter.refresh()
    return plotter.artists()


ani = FuncAnimation(plotter.fig, update, interval=100, cache_frame_data=False)
plt.tight_layout()
plt.show()
