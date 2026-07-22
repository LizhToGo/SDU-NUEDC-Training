"""Low-overhead P/Pmax state and font-free preview overlay.

Physical input power cannot be inferred from camera pixels.  The production
path uses the supplied INA226 backend; a USB power meter, INA219-class sensor
or touch-screen controller can still provide UART samples as ``W=1.23`` or
``I=0.823,W=4.12``.  ``PowerMonitor`` keeps the latest values and maximum and
draws both without ``draw_string()`` (which requires FreeType on CanMV).
"""

import time


# The desktop test environment does not provide CanMV's ``machine`` module.
# Keep the INA226 driver optional there, while making it the production
# backend on K230 when the supplied driver is present.
_INA226_IMPORT_ERROR = None
try:
    from ina226_power_monitor_fixed import (
        INA226PowerMonitor,
        create_power_monitor as create_ina226_power_monitor,
    )
except Exception as _error:
    INA226PowerMonitor = None
    create_ina226_power_monitor = None
    _INA226_IMPORT_ERROR = _error


_FONT = {
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "111", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "001", "010", "010"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
    "P": ("110", "101", "110", "100", "100"),
    "M": ("10001", "11011", "10101", "10101", "10101"),
    "A": ("010", "101", "111", "101", "101"),
    "X": ("101", "101", "010", "101", "101"),
    "W": ("10101", "10101", "10101", "10101", "01010"),
    ":": ("0", "1", "0", "1", "0"),
    ".": ("0", "0", "0", "0", "1"),
    "-": ("000", "000", "111", "000", "000"),
    " ": ("0", "0", "0", "0", "0"),
}


def create_ina226_monitor(**kwargs):
    """Create the supplied INA226 backend or raise a useful error."""
    if create_ina226_power_monitor is None:
        raise OSError(
            "INA226 driver unavailable: %s" % repr(_INA226_IMPORT_ERROR)
        )
    return create_ina226_power_monitor(**kwargs)


def parse_power_sample(command):
    """Return watts from ``W1.23``, ``W=1.23`` or ``POWER=1.23``."""
    text = command.strip().upper().replace(" ", "")
    value_text = None
    if text.startswith("POWER="):
        value_text = text[6:]
    elif text.startswith("W="):
        value_text = text[2:]
    elif text.startswith("W") and len(text) > 1:
        value_text = text[1:]
    if value_text is None or not value_text:
        return None
    try:
        value = float(value_text)
    except Exception:
        return None
    if value < 0.0 or value > 100.0:
        return None
    return value


def parse_current_power_sample(command):
    """Return ``(current_a, watts)`` from a compact ASCII sample.

    Accepted forms are ``I=0.823,W=4.12`` and ``I0.823;W4.12``.  A missing
    field is returned as ``None`` so a current-only or power-only sensor can
    update the same monitor state.
    """
    text = command.strip().upper().replace(" ", "")
    if not text or ("I" not in text and "W" not in text):
        return None
    current = None
    watts = None
    fields = text.replace(";", ",").split(",")
    for field in fields:
        if not field:
            continue
        if field.startswith("I="):
            value_text = field[2:]
        elif field.startswith("I") and len(field) > 1:
            value_text = field[1:]
        else:
            value_text = None
        if value_text is not None:
            try:
                value = float(value_text)
            except Exception:
                return None
            if value < 0.0 or value > 20.0:
                return None
            current = value
            continue
        if field.startswith("W="):
            value_text = field[2:]
        elif field.startswith("W") and len(field) > 1:
            value_text = field[1:]
        else:
            value_text = None
        if value_text is not None:
            try:
                value = float(value_text)
            except Exception:
                return None
            if value < 0.0 or value > 100.0:
                return None
            watts = value
    if current is None and watts is None:
        return None
    return current, watts


class PowerMonitor:
    def __init__(
        self,
        smoothing=0.25,
        hardware_monitor=None,
        hardware_poll_interval_ms=100,
        current_offset_a=0.0,
    ):
        self.smoothing = float(smoothing)
        self.current_a = None
        self.raw_current_a = None
        self.current_w = None
        self.maximum_w = 0.0
        self.raw_w = None
        self.sample_count = 0
        self.last_update_ms = None
        self.hardware_monitor = hardware_monitor
        self.hardware_poll_interval_ms = max(
            0, int(hardware_poll_interval_ms)
        )
        # Optional empirical correction for a known INA226 current bias.  It
        # is deliberately applied only to hardware samples, never to UART
        # manual input.  Keep the raw chip value in hardware_sample for
        # diagnostics while exposing the corrected value to the UI/Pmax path.
        self.current_offset_a = float(current_offset_a)
        self.last_hardware_poll_ms = None
        self.hardware_sample = None
        self.hardware_error = None
        self.hardware_warning = None
        self.current_direction = 0
        self.reverse_current_samples = 0

    @staticmethod
    def _now_ms():
        try:
            return time.ticks_ms()
        except AttributeError:
            return int(time.time() * 1000.0)

    @staticmethod
    def _elapsed_ms(now, previous):
        try:
            return time.ticks_diff(now, previous)
        except AttributeError:
            return now - previous

    def attach_hardware(self, hardware_monitor):
        """Attach an INA226-like object exposing ``read_sample()``."""
        self.hardware_monitor = hardware_monitor
        self.last_hardware_poll_ms = None
        self.hardware_sample = None
        self.hardware_error = None
        self.hardware_warning = None
        self.current_direction = 0
        self.reverse_current_samples = 0
        return self.poll_hardware(force=True)

    def poll_hardware(self, force=False, timestamp_ms=None):
        """Read the hardware backend at a bounded rate.

        The camera preview runs much faster than the INA226 needs to be
        sampled.  Throttling I2C reads prevents the power monitor from
        reducing preview FPS, while ``force=True`` gives measurement/status
        events an up-to-date value immediately.
        """
        monitor = self.hardware_monitor
        if monitor is None:
            return False

        now = self._now_ms() if timestamp_ms is None else timestamp_ms
        if (
            not force
            and self.last_hardware_poll_ms is not None
            and self._elapsed_ms(now, self.last_hardware_poll_ms)
            < self.hardware_poll_interval_ms
        ):
            return False

        self.last_hardware_poll_ms = now
        try:
            sample = monitor.read_sample()
            self.hardware_sample = sample
            if sample.get("overflow"):
                self.hardware_error = "INA226_OVERFLOW"
                return False

            current_a = sample.get("current_a")
            watts = sample.get("power_w")
            if current_a is None and watts is None:
                raise ValueError("INA226 sample has no current/power")

            # INA226's current register is signed.  A negative value normally
            # means the module's IN+/IN- current path is wired opposite to the
            # driver's positive direction; a small negative value can also be
            # zero-current offset.  Input-power display needs the magnitude,
            # so retain the raw signed sample for diagnostics but feed a
            # non-negative magnitude into the UI/Pmax state.
            self.hardware_warning = None
            if current_a is not None:
                signed_current_a = float(current_a)
                if signed_current_a < 0.0:
                    self.current_direction = -1
                    self.reverse_current_samples += 1
                    self.hardware_warning = "INA226_REVERSED_CURRENT"
                    current_a = -signed_current_a
                elif signed_current_a > 0.0:
                    self.current_direction = 1
                else:
                    self.current_direction = 0
                # The measured current is known to be low by a fixed
                # empirical amount.  Apply the correction after taking the
                # magnitude so reversed INA226 wiring is handled identically.
                current_a = max(0.0, float(current_a)) + self.current_offset_a
                # When bus voltage is available, derive corrected input power
                # from corrected current.  Otherwise retain the INA226 power
                # register value rather than inventing a voltage.
                bus_voltage_v = sample.get("bus_voltage_v")
                if (
                    (
                        self.current_offset_a != 0.0
                        or self.current_direction < 0
                    )
                    and bus_voltage_v is not None
                ):
                    watts = abs(float(bus_voltage_v) * current_a)
            if watts is not None and float(watts) < 0.0:
                watts = -float(watts)

            self.update_sample(
                current_a=current_a,
                watts=watts,
                timestamp_ms=now,
            )

            # Keep the wrapper peak synchronized with the driver's peak.
            hardware_peak = (
                sample.get("pmax_w")
                if self.reverse_current_samples == 0
                else None
            )
            if (
                hardware_peak is not None
                and float(hardware_peak) > self.maximum_w
            ):
                self.maximum_w = float(hardware_peak)
            self.hardware_error = None
            return True
        except Exception as error:
            self.hardware_error = repr(error)
            return False

    def hardware_status(self):
        """Return the latest raw INA226 sample and backend error, if any."""
        return {
            "available": self.hardware_monitor is not None,
            "sample": self.hardware_sample,
            "error": self.hardware_error,
            "warning": self.hardware_warning,
            "current_direction": self.current_direction,
        }

    def update_sample(self, current_a=None, watts=None, timestamp_ms=None):
        if current_a is None and watts is None:
            raise ValueError("at least current or power must be provided")
        alpha = self.smoothing
        if current_a is not None:
            current_a = float(current_a)
            if current_a < 0.0:
                raise ValueError("current must be non-negative")
            self.raw_current_a = current_a
            if self.current_a is None:
                self.current_a = current_a
            else:
                self.current_a = (
                    alpha * current_a + (1.0 - alpha) * self.current_a
                )
        if watts is not None:
            watts = float(watts)
            if watts < 0.0:
                raise ValueError("power must be non-negative")
            self.raw_w = watts
            if self.current_w is None:
                self.current_w = watts
            else:
                self.current_w = (
                    alpha * watts + (1.0 - alpha) * self.current_w
                )
            if watts > self.maximum_w:
                self.maximum_w = watts
        self.sample_count += 1
        if timestamp_ms is None:
            timestamp_ms = self._now_ms()
        self.last_update_ms = timestamp_ms
        return self.status()

    def update_watts(self, watts, timestamp_ms=None):
        """Backward-compatible power-only update."""
        return self.update_sample(watts=watts, timestamp_ms=timestamp_ms)

    def reset_maximum(self):
        if self.hardware_monitor is not None:
            try:
                self.hardware_monitor.reset_peak()
            except Exception as error:
                self.hardware_error = repr(error)
        self.maximum_w = self.raw_w if self.raw_w is not None else 0.0

    def status(self):
        return {
            "current_a": self.current_a,
            "raw_current_a": self.raw_current_a,
            "current_w": self.current_w,
            "raw_w": self.raw_w,
            "maximum_w": self.maximum_w,
            "sample_count": self.sample_count,
            "last_update_ms": self.last_update_ms,
            "source": (
                "INA226"
                if self.hardware_monitor is not None
                else "UART_OR_MANUAL"
            ),
            "hardware_error": self.hardware_error,
            "hardware_warning": self.hardware_warning,
            "current_direction": self.current_direction,
            "reverse_current_samples": self.reverse_current_samples,
        }

    def status_line(self):
        current_text = (
            "NONE" if self.current_a is None else "%.3f" % self.current_a
        )
        power_text = (
            "NONE" if self.current_w is None else "%.3f" % self.current_w
        )
        line = "POWER I=%s P=%s PMAX=%.3f SAMPLES=%d SRC=%s" % (
            current_text,
            power_text,
            self.maximum_w,
            self.sample_count,
            "INA226" if self.hardware_monitor is not None else "UART",
        )
        if self.hardware_error is not None:
            line += " ERR=%s" % self.hardware_error
        if self.hardware_warning is not None:
            line += " WARN=%s" % self.hardware_warning
        return line

    @staticmethod
    def _draw_text(image, text, x, y, color, scale=2):
        cursor_x = int(x)
        origin_y = int(y)
        scale = max(1, int(scale))
        for character in text:
            pattern = _FONT.get(character, _FONT[" "])
            width = len(pattern[0])
            for row in range(len(pattern)):
                column = 0
                while column < width:
                    if pattern[row][column] != "1":
                        column += 1
                        continue
                    run_start = column
                    while column + 1 < width and pattern[row][column + 1] == "1":
                        column += 1
                    run_end = column
                    pixel_x = cursor_x + run_start * scale
                    pixel_x_end = cursor_x + (run_end + 1) * scale - 1
                    pixel_y = origin_y + row * scale
                    try:
                        image.draw_line(
                            pixel_x,
                            pixel_y,
                            pixel_x_end,
                            pixel_y,
                            color=color,
                            thickness=scale,
                        )
                    except Exception:
                        return
                    column += 1
            cursor_x += (width + 1) * scale

    def draw_overlay(self, image, x=6, y=6, color=(0, 255, 120), scale=2):
        if self.current_w is None:
            text = "P:-- PMAX:--"
        else:
            text = "P:%.2f PMAX:%.2fW" % (
                self.current_w,
                self.maximum_w,
            )
        self._draw_text(image, text, x, y, color, scale)
