# ===========================
# UART Configuration
# ===========================

SERIAL_PORT = "COM17"      # default port, can be overridden with --port
BAUD_RATE = 115200         # must match USART1 in the firmware
SERIAL_TIMEOUT = 0.1       # s

# ===========================
# Plot Configuration
# ===========================

MAX_SAMPLES = 250          # samples shown on screen (250 x 20 ms = 5 s)
UPDATE_INTERVAL = 100      # ms between plot refreshes

# ===========================
# CSV Logger
# ===========================

LOG_DIR = "logs"           # each run creates a new timestamped CSV here
FLUSH_EVERY = 50           # flush to disk every N rows

# ===========================
# Firmware output scaling
# ===========================
# Firmware sends integers: acc x1000 (g), angles x100 (deg), gyro x100 (deg/s)
ACC_SCALE = 1000.0
ANGLE_SCALE = 100.0
GYRO_SCALE = 100.0
