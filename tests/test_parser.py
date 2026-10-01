import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from imu_parser import parse_sensor_data, is_error_message  # noqa: E402


def test_firmware_line_is_scaled():
    s = parse_sensor_data("15320,12,-8,1003,-45,120,310,25,-14,3")
    assert s.tick_ms == 15320
    assert (s.ax, s.ay, s.az) == pytest.approx((0.012, -0.008, 1.003))
    assert (s.roll, s.pitch, s.yaw) == pytest.approx((-0.45, 1.20, 3.10))
    assert (s.gx, s.gy, s.gz) == pytest.approx((0.25, -0.14, 0.03))


def test_crlf_is_ignored():
    assert parse_sensor_data("1,0,0,1000,0,0,0,0,0,0\r\n") is not None


def test_old_6_field_format_is_rejected():
    assert parse_sensor_data("25,-12,1010,520,-210,1050") is None


def test_non_numeric():
    assert parse_sensor_data("a,b,c,d,e,f,g,h,i,j") is None


def test_empty_line():
    assert parse_sensor_data("") is None


def test_error_message():
    assert is_error_message("ERR,MPU6050 not found")
    assert not is_error_message("1,0,0,1000,0,0,0,0,0,0")
