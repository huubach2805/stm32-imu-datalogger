import csv
import os
import time
from datetime import datetime

from config import LOG_DIR, FLUSH_EVERY
from imu_parser import IMUSample


class DataLogger:
    """Write each sample to a new timestamped CSV file."""

    def __init__(self, log_dir=LOG_DIR, flush_every=FLUSH_EVERY):
        os.makedirs(log_dir, exist_ok=True)
        filename = datetime.now().strftime("sensor_%Y%m%d_%H%M%S.csv")
        self.path = os.path.join(log_dir, filename)

        self.flush_every = flush_every
        self._rows = 0

        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["pc_time_s", *IMUSample._fields])

    def save(self, sample):
        self.writer.writerow([f"{time.time():.3f}", *sample])
        self._rows += 1

        # Flush regularly so data is not lost if the program is killed
        if self._rows % self.flush_every == 0:
            self.file.flush()

    def close(self):
        if not self.file.closed:
            self.file.flush()
            self.file.close()
            print(f"[OK] Saved {self._rows} samples to {self.path}")
