import serial

MAX_BUFFER = 4096   # bytes; drop garbage if no newline arrives


class SerialReader:
    """Non-blocking line reader for the STM32 UART stream."""

    def __init__(self, port, baud, timeout=0.1):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self._buffer = b""

    def connect(self):
        try:
            # serial_for_url accepts normal ports (COM17, /dev/ttyUSB0)
            # and test URLs such as "loop://"
            self.ser = serial.serial_for_url(
                self.port, self.baud, timeout=self.timeout
            )
        except serial.SerialException as e:
            print(f"[ERROR] Cannot open {self.port}: {e}")
            return False

        print(f"[OK] Connected to {self.port} @ {self.baud} baud")
        return True

    def read_lines(self):
        """Return every complete line currently waiting in the RX buffer."""
        if self.ser is None:
            return []

        try:
            waiting = self.ser.in_waiting
            if waiting:
                self._buffer += self.ser.read(waiting)
        except serial.SerialException as e:
            print(f"[ERROR] Serial read failed: {e}")
            return []

        if b"\n" not in self._buffer:
            if len(self._buffer) > MAX_BUFFER:
                self._buffer = b""
            return []

        *lines, self._buffer = self._buffer.split(b"\n")
        return [
            l.decode(errors="ignore").strip()
            for l in lines
            if l.strip()
        ]

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[OK] Serial closed")
