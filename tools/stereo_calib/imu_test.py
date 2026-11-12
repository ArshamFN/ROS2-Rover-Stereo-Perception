#!/usr/bin/env python3
"""
imu_test.py

Acceptance test for the ICM20948 on the IMX219-83 stereo board, wired to the Jetson
40-pin header (SDA pin 3, SCL pin 5 = I2C bus 7 on the Orin Nano).

Checks, in order:
  1. Identity: WHO_AM_I must read 0xEA at 0x68 or 0x69.
  2. Accel + gyro + temperature: live readings. At rest, |accel| ~1.00 g, gyro ~0 deg/s.
  3. Magnetometer (AK09916, reached through the ICM20948 bypass): WIA2 must read 0x09,
     field magnitude should be roughly 25 to 65 uT.

Usage: python3 imu_test.py [bus]      (Ctrl+C to stop; sudo if permission denied)
"""

import math
import sys
import time

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus
    except ImportError:
        sys.exit("Install an SMBus library first:  pip3 install smbus2")

BUS = int(sys.argv[1]) if len(sys.argv) > 1 else 7
MAG = 0x0C

# ICM20948 user bank 0 registers
WHO_AM_I, PWR_MGMT_1, PWR_MGMT_2, INT_PIN_CFG, USER_CTRL = 0x00, 0x06, 0x07, 0x0F, 0x03
ACCEL_XOUT_H, REG_BANK_SEL = 0x2D, 0x7F
# AK09916 registers
WIA2, ST1, HXL, ST2, CNTL2 = 0x01, 0x10, 0x11, 0x18, 0x31


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def find_imu(bus):
    for addr in (0x68, 0x69):
        try:
            who = bus.read_byte_data(addr, WHO_AM_I)
        except OSError:
            continue
        print(f"0x{addr:02X}: WHO_AM_I = 0x{who:02X} {'(ICM20948 OK)' if who == 0xEA else '(UNEXPECTED)'}")
        if who == 0xEA:
            return addr
    return None


def main():
    with SMBus(BUS) as bus:
        addr = find_imu(bus)
        if addr is None:
            sys.exit(f"No ICM20948 found on bus {BUS}. Check wiring, then run: sudo i2cdetect -y -r {BUS}")

        bus.write_byte_data(addr, REG_BANK_SEL, 0x00)  # bank 0
        bus.write_byte_data(addr, PWR_MGMT_1, 0x80)    # reset
        time.sleep(0.1)
        bus.write_byte_data(addr, REG_BANK_SEL, 0x00)
        bus.write_byte_data(addr, PWR_MGMT_1, 0x01)    # wake, auto clock
        bus.write_byte_data(addr, PWR_MGMT_2, 0x00)    # accel + gyro on
        time.sleep(0.05)

        # expose the magnetometer on the main bus
        bus.write_byte_data(addr, USER_CTRL, 0x00)     # I2C master off
        bus.write_byte_data(addr, INT_PIN_CFG, 0x02)   # bypass on
        time.sleep(0.01)
        mag_ok = False
        try:
            wia = bus.read_byte_data(MAG, WIA2)
            mag_ok = wia == 0x09
            print(f"0x0C: AK09916 WIA2 = 0x{wia:02X} {'(magnetometer OK)' if mag_ok else '(UNEXPECTED)'}")
            if mag_ok:
                bus.write_byte_data(MAG, CNTL2, 0x08)  # continuous 100 Hz
        except OSError:
            print("0x0C: magnetometer not responding (bypass failed or AK09916 dead)")

        print("\nLeave it still for the first readings, then tilt and rotate it.")
        print(f"{'accel g (x y z)':>26} {'|a|':>5}   {'gyro deg/s (x y z)':>24}   "
              f"{'temp C':>6}   {'mag uT |B|':>10}")
        while True:
            d = bus.read_i2c_block_data(addr, ACCEL_XOUT_H, 14)
            ax, ay, az = (s16(d[i], d[i + 1]) / 16384.0 for i in (0, 2, 4))   # +-2 g
            gx, gy, gz = (s16(d[i], d[i + 1]) / 131.0 for i in (6, 8, 10))    # +-250 dps
            temp = (s16(d[12], d[13]) - 21) / 333.87 + 21
            amag = math.sqrt(ax * ax + ay * ay + az * az)
            mag = "      n/a"
            if mag_ok and bus.read_byte_data(MAG, ST1) & 0x01:
                m = bus.read_i2c_block_data(MAG, HXL, 6)
                bus.read_byte_data(MAG, ST2)               # required to release the data
                mx, my, mz = (s16(m[i + 1], m[i]) * 0.15 for i in (0, 2, 4))
                mag = f"{math.sqrt(mx * mx + my * my + mz * mz):9.1f}"
            print(f"{ax:+8.3f} {ay:+8.3f} {az:+8.3f} {amag:5.2f}   "
                  f"{gx:+8.2f} {gy:+8.2f} {gz:+8.2f}   {temp:6.1f}   {mag:>10}", end="\r")
            time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
