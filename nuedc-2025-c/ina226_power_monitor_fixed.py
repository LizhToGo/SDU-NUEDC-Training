"""INA226 voltage/current/power driver for K230 CanMV MicroPython.

Default wiring for the 12-pin GPIO board shown by the user:
    GPIO34 -> IIC1_SCL
    GPIO35 -> IIC1_SDA

Important for an R100 (0.1 ohm) module:
    current_lsb = 25e-6 A/bit
    calibration = 0x0800
    maximum measurable current is about 0.819 A

The driver does not reset Pmax automatically. Call reset_peak() only when the
whole-test peak power should be cleared.
"""

import time
from machine import FPIOA, I2C


class INA226PowerMonitor:
    # Register map
    REG_CONFIG = 0x00
    REG_SHUNT_VOLTAGE = 0x01
    REG_BUS_VOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05
    REG_MASK_ENABLE = 0x06
    REG_ALERT_LIMIT = 0x07
    REG_MANUFACTURER_ID = 0xFE
    REG_DIE_ID = 0xFF

    # Mask/Enable register bits
    MASK_OVF = 1 << 2
    MASK_CVRF = 1 << 3
    MASK_AFF = 1 << 4

    # INA226 constants
    SHUNT_VOLTAGE_LSB_V = 2.5e-6
    BUS_VOLTAGE_LSB_V = 1.25e-3
    SHUNT_FULL_SCALE_V = 81.92e-3
    POWER_LSB_MULTIPLIER = 25.0
    CALIBRATION_CONSTANT = 0.00512

    # 16 averages, 1.1 ms bus + 1.1 ms shunt, continuous mode
    DEFAULT_CONFIG = 0x4527

    # R100 module defaults
    R100_SHUNT_OHM = 0.1
    R100_CURRENT_LSB_A = 25e-6

    def __init__(
        self,
        i2c_id=1,
        scl_pin=34,
        sda_pin=35,
        address=0x40,
        freq=100000,
        r_shunt=R100_SHUNT_OHM,
        current_lsb=R100_CURRENT_LSB_A,
        config=DEFAULT_CONFIG,
        verify=True,
        max_expected_current=None,
    ):
        if i2c_id < 0:
            raise ValueError("i2c_id must be non-negative")
        if not 0x03 <= address <= 0x77:
            raise ValueError("address must be a valid 7-bit I2C address")
        if freq <= 0:
            raise ValueError("freq must be positive")
        if r_shunt <= 0:
            raise ValueError("r_shunt must be positive")
        if current_lsb <= 0:
            raise ValueError("current_lsb must be positive")

        self.i2c_id = int(i2c_id)
        self.scl_pin = None if scl_pin is None else int(scl_pin)
        self.sda_pin = None if sda_pin is None else int(sda_pin)
        self.freq = int(freq)
        self.address = int(address)
        self.r_shunt = float(r_shunt)
        self.current_lsb = float(current_lsb)
        self.power_lsb = self.current_lsb * self.POWER_LSB_MULTIPLIER
        self.config = int(config) & 0x7FFF

        self.calibration = self._calculate_calibration()
        self.max_current_by_register_a = 32767.0 * self.current_lsb
        self.max_current_by_shunt_a = (
            self.SHUNT_FULL_SCALE_V / self.r_shunt
        )
        self.max_measurable_current_a = min(
            self.max_current_by_register_a,
            self.max_current_by_shunt_a,
        )

        if (
            max_expected_current is not None
            and float(max_expected_current) > self.max_measurable_current_a
        ):
            raise ValueError(
                "expected current %.3f A exceeds measurable range %.3f A"
                % (
                    float(max_expected_current),
                    self.max_measurable_current_a,
                )
            )

        self.init_stage = "FPIOA"
        try:
            configure_i2c_gpio(
                self.i2c_id,
                self.scl_pin,
                self.sda_pin,
            )
        except Exception as error:
            raise OSError(
                "INA226 FPIOA setup failed I2C%d SCL=%s SDA=%s: %s"
                % (
                    self.i2c_id,
                    self.scl_pin,
                    self.sda_pin,
                    repr(error),
                )
            )

        self.init_stage = "I2C_OPEN"
        self.i2c = create_i2c_bus(
            self.i2c_id,
            self.scl_pin,
            self.sda_pin,
            self.freq,
        )

        # Keep the scan result in the object so a failed startup can report
        # whether the bus sees anything at all.  This is especially useful on
        # CanMV, where a missing INA226 otherwise appears only as OSError(1).
        self.init_stage = "I2C_SCAN"
        self.detected_devices = None
        self.scan_error = None
        try:
            self.detected_devices = self.scan()
        except Exception as error:
            self.scan_error = repr(error)

        self.pmax_w = 0.0
        self.last_sample = None

        if verify:
            self.init_stage = "VERIFY"
            # INA226 address straps allow 0x40..0x4F.  Keep 0x40 as the
            # preferred address, but use a detected INA226 address when the
            # module is strapped differently.
            self._select_detected_address()
            try:
                self.verify_device()
            except Exception as error:
                raise OSError(
                    "INA226 verify failed addr=0x%02X scan=%s: %s"
                    % (
                        self.address,
                        self._format_scan(),
                        repr(error),
                    )
                )

        self.init_stage = "CONFIGURE"
        try:
            self.configure()
        except Exception as error:
            raise OSError(
                "INA226 register configuration failed addr=0x%02X: %s"
                % (self.address, repr(error))
            )
        self.init_stage = "READY"

    def _format_scan(self):
        if self.detected_devices is None:
            return "ERROR:%s" % self.scan_error
        return "[%s]" % ",".join(
            "0x%02X" % int(address)
            for address in self.detected_devices
        )

    def _select_detected_address(self):
        devices = self.detected_devices
        if not devices:
            return self.address

        # Test the preferred address first, then all other INA226 strap
        # addresses visible on the bus.  This avoids selecting an unrelated
        # 0x40-class I2C peripheral when more than one device is connected.
        candidates = []
        if self.address in devices:
            candidates.append(self.address)
        for candidate in range(0x40, 0x50):
            if candidate in devices and candidate not in candidates:
                candidates.append(candidate)

        original_address = self.address
        for candidate in candidates:
            self.address = candidate
            try:
                manufacturer = self.read_u16(self.REG_MANUFACTURER_ID)
                die_id = self.read_u16(self.REG_DIE_ID)
                if (
                    manufacturer == 0x5449
                    and (die_id >> 4) == 0x226
                ):
                    return candidate
            except Exception:
                pass
        self.address = original_address
        return self.address

    @staticmethod
    def _to_s16(value):
        value &= 0xFFFF
        return value - 0x10000 if value & 0x8000 else value

    def _calculate_calibration(self):
        calibration = int(
            self.CALIBRATION_CONSTANT
            / (self.current_lsb * self.r_shunt)
        )

        if calibration < 1 or calibration > 0x7FFF:
            raise ValueError(
                "INA226 calibration out of range: %d" % calibration
            )

        return calibration

    def scan(self):
        return self.i2c.scan()

    def _read_mem(self, register, length):
        """Read with compatibility across CanMV API naming variants."""
        try:
            return self.i2c.readfrom_mem(
                self.address,
                register,
                length,
                addrsize=8,
            )
        except TypeError:
            try:
                return self.i2c.readfrom_mem(
                    self.address,
                    register,
                    length,
                    mem_size=8,
                )
            except TypeError:
                return self.i2c.readfrom_mem(
                    self.address,
                    register,
                    length,
                )

    def _write_mem(self, register, data):
        """Write with compatibility across CanMV API naming variants."""
        try:
            return self.i2c.writeto_mem(
                self.address,
                register,
                data,
                addrsize=8,
            )
        except TypeError:
            try:
                return self.i2c.writeto_mem(
                    self.address,
                    register,
                    data,
                    mem_size=8,
                )
            except TypeError:
                return self.i2c.writeto_mem(
                    self.address,
                    register,
                    data,
                )

    def read_u16(self, register):
        data = self._read_mem(register, 2)

        if data is None or len(data) != 2:
            raise OSError("INA226 read length error")

        return (data[0] << 8) | data[1]

    def read_s16(self, register):
        return self._to_s16(self.read_u16(register))

    def write_u16(self, register, value):
        value &= 0xFFFF
        data = bytes([
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])
        self._write_mem(register, data)

    def verify_device(self):
        manufacturer = self.read_u16(
            self.REG_MANUFACTURER_ID
        )
        die_id = self.read_u16(self.REG_DIE_ID)

        if manufacturer != 0x5449:
            raise OSError(
                "INA226 manufacturer mismatch: 0x%04X"
                % manufacturer
            )

        if (die_id >> 4) != 0x226:
            raise OSError(
                "INA226 die id mismatch: 0x%04X"
                % die_id
            )

        return manufacturer, die_id

    def configure(self):
        """
        Program calibration first, then restart conversion configuration.

        Writing configuration after calibration ensures the first complete
        conversion uses the intended current and power scaling.
        """
        self.write_u16(
            self.REG_CALIBRATION,
            self.calibration,
        )
        self.write_u16(
            self.REG_CONFIG,
            self.config,
        )

    def reset(self, timeout_ms=250):
        """Reset INA226 and restore calibration/configuration."""
        self.write_u16(self.REG_CONFIG, 0x8000)
        time.sleep_ms(2)
        self.configure()

        if not self.wait_conversion_ready(timeout_ms):
            raise OSError(
                "INA226 conversion timeout after reset"
            )

    def wait_conversion_ready(
        self,
        timeout_ms=250,
        poll_interval_ms=1,
    ):
        """
        Wait until the conversion-ready flag is observed.

        Reading REG_MASK_ENABLE clears CVRF, so use this only as an event
        check, not as a permanently latched data-valid flag.
        """
        start = time.ticks_ms()

        while (
            time.ticks_diff(time.ticks_ms(), start)
            < timeout_ms
        ):
            status = self.read_u16(
                self.REG_MASK_ENABLE
            )

            if status & self.MASK_CVRF:
                return True

            time.sleep_ms(poll_interval_ms)

        return False

    def reset_peak(self):
        """Clear software Pmax. Do not call this for every measurement."""
        self.pmax_w = 0.0

    def range_info(self):
        return {
            "max_current_by_register_a": (
                self.max_current_by_register_a
            ),
            "max_current_by_shunt_a": (
                self.max_current_by_shunt_a
            ),
            "max_measurable_current_a": (
                self.max_measurable_current_a
            ),
            "current_lsb_a": self.current_lsb,
            "power_lsb_w": self.power_lsb,
            "r_shunt_ohm": self.r_shunt,
            "calibration": self.calibration,
        }

    def read_sample(self):
        shunt_raw = self.read_s16(
            self.REG_SHUNT_VOLTAGE
        )
        bus_raw = self.read_u16(
            self.REG_BUS_VOLTAGE
        )
        current_raw = self.read_s16(
            self.REG_CURRENT
        )
        power_raw = self.read_u16(
            self.REG_POWER
        )
        status = self.read_u16(
            self.REG_MASK_ENABLE
        )

        bus_voltage_v = (
            bus_raw * self.BUS_VOLTAGE_LSB_V
        )
        shunt_voltage_v = (
            shunt_raw * self.SHUNT_VOLTAGE_LSB_V
        )
        current_a = (
            current_raw * self.current_lsb
        )
        power_w = (
            power_raw * self.power_lsb
        )

        manual_current_a = (
            shunt_voltage_v / self.r_shunt
        )
        manual_power_w = (
            bus_voltage_v * manual_current_a
        )

        overflow = bool(status & self.MASK_OVF)

        # Do not update Pmax from an overflowed sample.
        if not overflow and power_w > self.pmax_w:
            self.pmax_w = power_w

        sample = {
            "bus_voltage_v": bus_voltage_v,
            "shunt_voltage_v": shunt_voltage_v,
            "current_a": current_a,
            "power_w": power_w,
            "manual_current_a": manual_current_a,
            "manual_power_w": manual_power_w,
            "pmax_w": self.pmax_w,
            "overflow": overflow,
            "alert": bool(status & self.MASK_AFF),
            "conversion_ready": bool(
                status & self.MASK_CVRF
            ),
            "calibration": self.calibration,
            "max_measurable_current_a": (
                self.max_measurable_current_a
            ),
        }

        self.last_sample = sample
        return sample

    def format_sample(self, sample=None):
        if sample is None:
            sample = self.last_sample

        if sample is None:
            return "INA226 power=NONE"

        return (
            "INA226 V=%.4fV I=%.4fA P=%.4fW "
            "Pmax=%.4fW OVF=%d"
            % (
                sample["bus_voltage_v"],
                sample["current_a"],
                sample["power_w"],
                sample["pmax_w"],
                1 if sample["overflow"] else 0,
            )
        )


def create_power_monitor(
    # GPIO34/35 on the shown board are IIC1_SCL/IIC1_SDA.
    i2c_id=1,
    scl_pin=34,
    sda_pin=35,
    address=0x40,
    freq=100000,
    r_shunt=INA226PowerMonitor.R100_SHUNT_OHM,
    current_lsb=INA226PowerMonitor.R100_CURRENT_LSB_A,
    config=INA226PowerMonitor.DEFAULT_CONFIG,
    verify=True,
    max_expected_current=None,
    first_conversion_timeout_ms=250,
):
    monitor = INA226PowerMonitor(
        i2c_id=i2c_id,
        scl_pin=scl_pin,
        sda_pin=sda_pin,
        address=address,
        freq=freq,
        r_shunt=r_shunt,
        current_lsb=current_lsb,
        config=config,
        verify=verify,
        max_expected_current=max_expected_current,
    )

    if not monitor.wait_conversion_ready(
        timeout_ms=first_conversion_timeout_ms
    ):
        raise OSError(
            "INA226 first conversion timeout"
        )

    monitor.read_sample()
    return monitor


def _get_i2c_fpioa_function(i2c_id, names):
    for name in names:
        attr = "IIC%d_%s" % (i2c_id, name)

        if hasattr(FPIOA, attr):
            return getattr(FPIOA, attr)

    raise AttributeError(
        "I2C%d FPIOA function not found: %s"
        % (i2c_id, names)
    )


def _set_i2c_pin_function(fpioa, pin, function):
    """Configure an I2C pin across CanMV FPIOA API revisions.

    The Yahboom examples use the expanded electrical options below.  The
    previous driver only passed ``pin`` and ``function``; on some K230 images
    that leaves the input path/pull-up disabled and every transaction returns
    the opaque ``OSError(1)``.  Older firmware accepts fewer keyword
    arguments, so progressively fall back to the smaller signatures.
    """
    keyword_variants = (
        {"oe": 1, "ie": 1, "pu": 1, "st": 1, "ds": 15},
        {"oe": 1, "ie": 1, "pu": 1},
        {},
    )
    last_type_error = None
    for kwargs in keyword_variants:
        try:
            fpioa.set_function(pin, function, **kwargs)
            return
        except TypeError as error:
            last_type_error = error
    if last_type_error is not None:
        raise last_type_error


def create_i2c_bus(i2c_id, scl_pin=None, sda_pin=None, freq=100000):
    """Create a CanMV I2C object with constructor compatibility fallbacks."""
    attempts = []
    constructors = []

    if scl_pin is not None and sda_pin is not None:
        constructors.append(
            lambda: I2C(
                i2c_id,
                scl=scl_pin,
                sda=sda_pin,
                freq=freq,
            )
        )
    constructors.append(lambda: I2C(i2c_id, freq=freq))
    constructors.append(lambda: I2C(i2c_id))

    for constructor in constructors:
        try:
            return constructor()
        except Exception as error:
            attempts.append(repr(error))

    raise OSError(
        "I2C%d open failed SCL=%s SDA=%s freq=%s attempts=%s"
        % (
            i2c_id,
            scl_pin,
            sda_pin,
            freq,
            ";".join(attempts),
        )
    )


def configure_i2c_gpio(
    i2c_id,
    scl_pin,
    sda_pin,
):
    if scl_pin is None and sda_pin is None:
        return

    if scl_pin is None or sda_pin is None:
        raise ValueError(
            "scl_pin and sda_pin must be set together"
        )

    fpioa = FPIOA()

    # Prefer the exact names used by Yahboom's K230 examples.  Some firmware
    # exposes SCLK as an alias, hence the second fallback name.
    scl_func = _get_i2c_fpioa_function(
        i2c_id,
        ("SCL", "SCLK"),
    )
    sda_func = _get_i2c_fpioa_function(
        i2c_id,
        ("SDA",),
    )

    _set_i2c_pin_function(fpioa, scl_pin, scl_func)
    _set_i2c_pin_function(fpioa, sda_pin, sda_func)


if __name__ == "__main__":
    # Minimal on-board diagnostic.
    monitor = create_power_monitor(
        i2c_id=1,
        scl_pin=34,
        sda_pin=35,
        address=0x40,
        freq=100000,
        r_shunt=0.1,
        current_lsb=25e-6,
        config=0x4527,
        verify=True,
        # Set this only after confirming the expected current is within range.
        max_expected_current=None,
    )

    print(
        "I2C devices:",
        [hex(address) for address in monitor.scan()],
    )
    print("Range:", monitor.range_info())

    # Keep Pmax across the entire program run.
    monitor.reset_peak()

    while True:
        sample = monitor.read_sample()
        print(monitor.format_sample(sample))
        time.sleep_ms(30)
