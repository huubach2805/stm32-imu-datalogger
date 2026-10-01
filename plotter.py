from collections import deque
import matplotlib.pyplot as plt

from config import MAX_SAMPLES


class Plotter:
    """Live plot: acceleration, angular rate and filtered angles."""

    def __init__(self, max_samples=MAX_SAMPLES):

        self.t = deque(maxlen=max_samples)
        self.data = {
            name: deque(maxlen=max_samples)
            for name in ("ax", "ay", "az", "gx", "gy", "gz", "roll", "pitch", "yaw")
        }

        self.fig, (self.acc_plot, self.gyro_plot, self.angle_plot) = plt.subplots(
            3, 1, sharex=True, figsize=(9, 8)
        )

        groups = [
            (self.acc_plot, ("ax", "ay", "az"), "Acceleration", "g"),
            (self.gyro_plot, ("gx", "gy", "gz"), "Angular rate", "deg/s"),
            (self.angle_plot, ("roll", "pitch", "yaw"), "Angle (complementary filter)", "deg"),
        ]

        self.lines = {}
        for axis, names, title, unit in groups:
            for name in names:
                self.lines[name], = axis.plot([], [], label=name)
            axis.set_title(title)
            axis.set_ylabel(unit)
            axis.grid(True)
            axis.legend(loc="upper left")

        self.angle_plot.set_xlabel("STM32 time (s)")
        self.plots = (self.acc_plot, self.gyro_plot, self.angle_plot)

    def add_data(self, sample):
        self.t.append(sample.tick_ms / 1000.0)
        for name, buf in self.data.items():
            buf.append(getattr(sample, name))

    def refresh(self):
        for name, line in self.lines.items():
            line.set_data(self.t, self.data[name])

        for axis in self.plots:
            axis.relim()
            axis.autoscale_view()

    def artists(self):
        return tuple(self.lines.values())
