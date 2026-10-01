from collections import namedtuple

from config import ACC_SCALE, ANGLE_SCALE, GYRO_SCALE

IMUSample = namedtuple(
    "IMUSample",
    ["tick_ms", "ax", "ay", "az", "roll", "pitch", "yaw", "gx", "gy", "gz"],
)
"""
tick_ms        : STM32 HAL_GetTick() when the sample was taken
ax, ay, az     : acceleration (g)
roll, pitch    : complementary-filter angles (deg)
yaw            : integrated gyro Z (deg) - drifts, no magnetometer
gx, gy, gz     : angular rate (deg/s)
"""


def parse_sensor_data(line):
    """
    Parse one line sent by the firmware (10 integer fields):

        tick_ms,ax,ay,az,roll,pitch,yaw,gx,gy,gz
        e.g. 15320,12,-8,1003,-45,120,310,25,-14,3

    Returns an IMUSample in physical units, or None if the line is invalid.
    """
    parts = line.strip().split(",")

    if len(parts) != 10:
        return None

    try:
        values = [int(x) for x in parts]
    except ValueError:
        return None

    tick, ax, ay, az, roll, pitch, yaw, gx, gy, gz = values

    return IMUSample(
        tick_ms=tick,
        ax=ax / ACC_SCALE, ay=ay / ACC_SCALE, az=az / ACC_SCALE,
        roll=roll / ANGLE_SCALE, pitch=pitch / ANGLE_SCALE, yaw=yaw / ANGLE_SCALE,
        gx=gx / GYRO_SCALE, gy=gy / GYRO_SCALE, gz=gz / GYRO_SCALE,
    )


def is_error_message(line):
    """Firmware reports problems as lines starting with 'ERR,'."""
    return line.startswith("ERR,")
