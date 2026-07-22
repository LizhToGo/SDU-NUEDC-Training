"""Minimal INA226/I2C diagnostic for Yahboom K230 CanMV MicroPython.

Copy this file together with ``ina226_power_monitor_fixed.py`` to ``/sdcard``
and run it directly when the production program reports INA226_INIT_ERROR.
It does not initialize the camera, display or touch-screen UART.
"""

from ina226_power_monitor_fixed import (
    configure_i2c_gpio,
    create_i2c_bus,
)


I2C_ID = 1
SCL_PIN = 34
SDA_PIN = 35
FREQ = 100000


def read_u16(i2c, address, register):
    try:
        data = i2c.readfrom_mem(
            address,
            register,
            2,
            addrsize=8,
        )
    except TypeError:
        try:
            data = i2c.readfrom_mem(
                address,
                register,
                2,
                mem_size=8,
            )
        except TypeError:
            data = i2c.readfrom_mem(address, register, 2)
    if data is None or len(data) != 2:
        raise OSError("read length error")
    return (data[0] << 8) | data[1]


print(
    "INA226_PROBE_START I2C%d GPIO%d=SCL GPIO%d=SDA freq=%d"
    % (I2C_ID, SCL_PIN, SDA_PIN, FREQ)
)

configure_i2c_gpio(I2C_ID, SCL_PIN, SDA_PIN)
i2c = create_i2c_bus(I2C_ID, SCL_PIN, SDA_PIN, FREQ)

try:
    devices = i2c.scan()
except Exception as error:
    print("I2C_SCAN_ERROR:", repr(error))
    raise

print("I2C_SCAN:", ["0x%02X" % address for address in devices])

if not devices:
    print(
        "NO_I2C_ACK: check INA226 3.3V/GND, common ground, "
        "GPIO34=SCL, GPIO35=SDA and pull-ups."
    )
else:
    found_ina226 = False
    for address in devices:
        try:
            manufacturer = read_u16(i2c, address, 0xFE)
            die_id = read_u16(i2c, address, 0xFF)
            is_ina226 = (
                manufacturer == 0x5449
                and (die_id >> 4) == 0x226
            )
            print(
                "DEVICE addr=0x%02X manufacturer=0x%04X die=0x%04X INA226=%s"
                % (
                    address,
                    manufacturer,
                    die_id,
                    "YES" if is_ina226 else "NO",
                )
            )
            if is_ina226:
                found_ina226 = True
        except Exception as error:
            print(
                "DEVICE_READ_ERROR addr=0x%02X error=%s"
                % (address, repr(error))
            )
    if not found_ina226:
        print("NO_INA226_ID_FOUND")

print("INA226_PROBE_DONE")
