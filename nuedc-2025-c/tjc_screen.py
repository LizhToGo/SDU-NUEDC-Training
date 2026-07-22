"""TJC/Nextion X2 serial touch-screen protocol for CanMV MicroPython.

Screen -> K230 frames are fixed seven-byte packets::

    55 CMD DATA0 DATA1 FF FF FF

K230 -> screen commands are ASCII statements terminated by three ``0xFF``
bytes.  The adapter accepts either a native ``machine.UART`` object or the
Yahboom ``YbUart`` wrapper used by this project.
"""


class TJCFrame:
    def __init__(self, command, data0, data1):
        self.command = int(command)
        self.data0 = int(data0)
        self.data1 = int(data1)

    def __repr__(self):
        return (
            "TJCFrame(command=0x%02X, data0=%d, data1=%d)"
            % (self.command, self.data0, self.data1)
        )


class TJCScreen:
    END = b"\xff\xff\xff"
    HEADER = 0x55
    FRAME_LENGTH = 7

    # The TJC virtual-float controls receive an integer and place the
    # decimal point according to each control's ``vvs1`` setting.  The
    # current screen project uses two decimal places for distance/size,
    # three for current, and two for power/Pmax.
    DISTANCE_SCALE = 100
    SIZE_SCALE = 100
    CURRENT_SCALE = 1000
    POWER_SCALE = 100

    CMD_START_MEASURE = 0x01
    CMD_RESET_PMAX = 0x02
    CMD_SCREEN_READY = 0x03
    CMD_REQUEST_REFRESH = 0x04

    MODE_AUTO = 0x00
    MODE_MIN_SQUARE = 0x01
    MODE_NUMBERED_SQUARE = 0x02
    MODE_TILTED = 0x03

    STATUS_IDLE = 0
    STATUS_MEASURING = 1
    STATUS_SUCCESS = 2
    STATUS_FAILED = 3
    STATUS_NO_TARGET = 4
    STATUS_SHAPE_FAILED = 5
    STATUS_NUMBER_NOT_FOUND = 6
    STATUS_SYSTEM_ERROR = 7

    def __init__(self, uart, page="main", max_rx_buffer=256):
        self.uart = uart
        self.page = page
        self.max_rx_buffer = max(16, int(max_rx_buffer))
        self.rx_buffer = bytearray()

    def _raw_uart(self):
        nested = getattr(self.uart, "uart", None)
        return nested if nested is not None else self.uart

    def _write(self, payload):
        raw = self._raw_uart()
        writer = getattr(raw, "write", None)
        if writer is not None:
            written = writer(payload)
        else:
            sender = getattr(self.uart, "send", None)
            if sender is None:
                raise AttributeError("UART object has no write/send method")
            written = sender(payload)
        if written is not None and written != len(payload):
            raise OSError("UART write incomplete")
        return written

    def _read(self):
        reader = getattr(self.uart, "read", None)
        if reader is None:
            reader = getattr(self._raw_uart(), "read", None)
        if reader is None:
            raise AttributeError("UART object has no read method")
        data = reader()
        if not data:
            return None
        if isinstance(data, str):
            converted = bytearray(len(data))
            for index in range(len(data)):
                converted[index] = ord(data[index]) & 0xFF
            return converted
        return data

    # ---------- K230 -> screen ----------

    def send_command(self, command):
        if not isinstance(command, str):
            raise TypeError("command must be str")
        try:
            payload = command.encode("ascii") + self.END
        except UnicodeEncodeError:
            raise ValueError("screen command must contain ASCII only")
        self._write(payload)

    def send_commands(self, commands):
        payload = bytearray()
        for command in commands:
            if not isinstance(command, str):
                raise TypeError("commands must contain str only")
            try:
                payload.extend(command.encode("ascii"))
            except UnicodeEncodeError:
                raise ValueError("screen commands must contain ASCII only")
            payload.extend(self.END)
        if payload:
            self._write(payload)

    def set_value(self, object_name, value, page=None):
        page_name = self.page if page is None else page
        self.send_command(
            "%s.%s.val=%d" % (page_name, object_name, int(value))
        )

    def set_status(self, status_code):
        status_code = int(status_code)
        if status_code < self.STATUS_IDLE or status_code > self.STATUS_SYSTEM_ERROR:
            raise ValueError("invalid status code")
        self.set_value("nStatus", status_code)

    def clear_measurement_values(self):
        self.send_commands((
            "%s.xD.val=0" % self.page,
            "%s.xSize.val=0" % self.page,
        ))

    def reset_display(self):
        self.send_commands((
            "%s.xD.val=0" % self.page,
            "%s.xSize.val=0" % self.page,
            "%s.xCurrent.val=0" % self.page,
            "%s.xPower.val=0" % self.page,
            "%s.xPmax.val=0" % self.page,
            "%s.nStatus.val=0" % self.page,
        ))

    def update_result(
        self,
        distance_cm,
        size_cm,
        current_a,
        power_w,
        pmax_w,
        status_code=STATUS_SUCCESS,
    ):
        self.send_commands((
            "%s.xD.val=%d" % (
                self.page, int(round(float(distance_cm) * self.DISTANCE_SCALE))
            ),
            "%s.xSize.val=%d" % (
                self.page, int(round(float(size_cm) * self.SIZE_SCALE))
            ),
            "%s.xCurrent.val=%d" % (
                self.page, int(round(float(current_a) * self.CURRENT_SCALE))
            ),
            "%s.xPower.val=%d" % (
                self.page, int(round(float(power_w) * self.POWER_SCALE))
            ),
            "%s.xPmax.val=%d" % (
                self.page, int(round(float(pmax_w) * self.POWER_SCALE))
            ),
            "%s.nStatus.val=%d" % (self.page, int(status_code)),
        ))

    def update_power(self, current_a, power_w, pmax_w):
        self.send_commands((
            "%s.xCurrent.val=%d" % (
                self.page, int(round(float(current_a) * self.CURRENT_SCALE))
            ),
            "%s.xPower.val=%d" % (
                self.page, int(round(float(power_w) * self.POWER_SCALE))
            ),
            "%s.xPmax.val=%d" % (
                self.page, int(round(float(pmax_w) * self.POWER_SCALE))
            ),
        ))

    # ---------- screen -> K230 ----------

    def _append_rx_data(self, data):
        if not data:
            return
        self.rx_buffer.extend(data)
        if len(self.rx_buffer) > self.max_rx_buffer:
            keep = self.FRAME_LENGTH - 1
            # CanMV MicroPython's bytearray does not implement item/slice
            # deletion.  Rebuild the short suffix instead of using ``del``.
            # Keeping FRAME_LENGTH-1 bytes preserves a possibly fragmented
            # frame header for the next UART read.
            self.rx_buffer = bytearray(self.rx_buffer[-keep:])

    def _extract_frames(self):
        frames = []
        cursor = 0
        buffer_length = len(self.rx_buffer)
        while buffer_length - cursor >= self.FRAME_LENGTH:
            if self.rx_buffer[cursor] != self.HEADER:
                cursor += 1
                continue
            if (
                self.rx_buffer[cursor + 4] != 0xFF
                or self.rx_buffer[cursor + 5] != 0xFF
                or self.rx_buffer[cursor + 6] != 0xFF
            ):
                cursor += 1
                continue
            frames.append(TJCFrame(
                self.rx_buffer[cursor + 1],
                self.rx_buffer[cursor + 2],
                self.rx_buffer[cursor + 3],
            ))
            cursor += self.FRAME_LENGTH

        if cursor > 0:
            # Preserve only the incomplete suffix.  Assignment is supported
            # by the board firmware even though bytearray deletion is not.
            self.rx_buffer = bytearray(self.rx_buffer[cursor:])
        return frames

    def poll(self):
        data = self._read()
        if data:
            self._append_rx_data(data)
        return self._extract_frames()

    def discard_input(self, reads=4):
        self.rx_buffer = bytearray()
        for _ in range(max(1, int(reads))):
            if not self._read():
                break
