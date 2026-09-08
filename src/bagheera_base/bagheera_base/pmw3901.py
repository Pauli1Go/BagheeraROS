"""PMW3901 SPI transport and motion-burst decoding.

The initialization sequence follows the MIT-licensed Pimoroni PMW3901
implementation: https://github.com/pimoroni/pmw3901-python
"""

from __future__ import annotations

import struct
import time


WAIT = -1
REG_ID = 0x00
REG_DATA_READY = 0x02
REG_MOTION_BURST = 0x16
REG_POWER_UP_RESET = 0x3A


def transform_counts(
    delta_x: int,
    delta_y: int,
    *,
    swap_xy: bool,
    invert_x: bool,
    invert_y: bool,
) -> tuple[int, int]:
    """Map sensor counts into the robot-forward X and robot-left Y axes."""
    x, y = (delta_y, delta_x) if swap_xy else (delta_x, delta_y)
    return (-x if invert_x else x, -y if invert_y else y)


class Pmw3901:
    """Access a PMW3901 using a Linux spidev device and hardware chip select."""

    def __init__(self, bus: int = 0, chip_select: int = 0, speed_hz: int = 400_000):
        try:
            import spidev
        except ModuleNotFoundError as exc:
            raise RuntimeError("python3-spidev is required for PMW3901 access") from exc

        self.spi = spidev.SpiDev()
        self.spi.open(bus, chip_select)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0
        self._write(REG_POWER_UP_RESET, 0x5A)
        time.sleep(0.02)
        for offset in range(5):
            self._read(REG_DATA_READY + offset)
        self._initialize_registers()
        product, revision = self.identity()
        inverse = self._read(0x5F)
        if product != 0x49 or revision not in (0x00, 0x01) or inverse != 0xB6:
            self.close()
            raise RuntimeError(
                "Unexpected PMW3901 identity "
                f"0x{product:02x}/0x{revision:02x}/0x{inverse:02x}"
            )

    def close(self) -> None:
        """Release the SPI device."""
        if self.spi is not None:
            self.spi.close()
            self.spi = None

    def identity(self) -> tuple[int, int]:
        """Return product and revision IDs."""
        return self._read(REG_ID), self._read(REG_ID + 1)

    def read_motion(self) -> tuple[int, int, int]:
        """Return delta X, delta Y and surface quality for one polling period.

        A burst without the motion bit is a valid zero-velocity observation,
        not missing data. Publishing it keeps downstream estimators informed
        that the chassis is stationary.
        """
        data = self.spi.xfer2([REG_MOTION_BURST] + [0] * 12)
        (_, motion, _observation, delta_x, delta_y, quality, *_rest) = struct.unpack(
            "<BBBhhBBBBBB", bytearray(data)
        )
        if not motion & 0x80:
            delta_x = 0
            delta_y = 0
        return delta_x, delta_y, quality

    def _write(self, register: int, value: int) -> None:
        self.spi.xfer2([register | 0x80, value])
        time.sleep(0.00005)

    def _read(self, register: int) -> int:
        result = self.spi.xfer2([register & 0x7F, 0])
        time.sleep(0.00005)
        return result[1]

    def _bulk_write(self, values: list[int]) -> None:
        for index in range(0, len(values), 2):
            register, value = values[index : index + 2]
            if register == WAIT:
                time.sleep(value / 1000.0)
            else:
                self._write(register, value)

    def _initialize_registers(self) -> None:
        self._bulk_write([0x7F, 0x00, 0x55, 0x01, 0x50, 0x07, 0x7F, 0x0E, 0x43, 0x10])
        self._write(0x48, 0x04 if self._read(0x67) & 0x80 else 0x02)
        self._bulk_write([0x7F, 0x00, 0x51, 0x7B, 0x50, 0x00, 0x55, 0x00, 0x7F, 0x0E])
        if self._read(0x73) == 0x00:
            first = self._read(0x70)
            second = self._read(0x71)
            first += 14 if first <= 28 else 11
            first = max(0, min(0x3F, first))
            second = (second * 45) // 100
            self._bulk_write([0x7F, 0x00, 0x61, 0xAD, 0x51, 0x70, 0x7F, 0x0E])
            self._write(0x70, first)
            self._write(0x71, second)

        # The final 0x14/0x6f writes enable the module's LED_N illumination
        # pulses. No interrupt line is required; motion is polled over SPI.
        self._bulk_write(
            [
                0x7F, 0x00, 0x61, 0xAD, 0x7F, 0x03, 0x40, 0x00,
                0x7F, 0x05, 0x41, 0xB3, 0x43, 0xF1, 0x45, 0x14,
                0x5B, 0x32, 0x5F, 0x34, 0x7B, 0x08, 0x7F, 0x06,
                0x44, 0x1B, 0x40, 0xBF, 0x4E, 0x3F, 0x7F, 0x08,
                0x65, 0x20, 0x6A, 0x18, 0x7F, 0x09, 0x4F, 0xAF,
                0x5F, 0x40, 0x48, 0x80, 0x49, 0x80, 0x57, 0x77,
                0x60, 0x78, 0x61, 0x78, 0x62, 0x08, 0x63, 0x50,
                0x7F, 0x0A, 0x45, 0x60, 0x7F, 0x00, 0x4D, 0x11,
                0x55, 0x80, 0x74, 0x21, 0x75, 0x1F, 0x4A, 0x78,
                0x4B, 0x78, 0x44, 0x08, 0x45, 0x50, 0x64, 0xFF,
                0x65, 0x1F, 0x7F, 0x14, 0x65, 0x67, 0x66, 0x08,
                0x63, 0x70, 0x7F, 0x15, 0x48, 0x48, 0x7F, 0x07,
                0x41, 0x0D, 0x43, 0x14, 0x4B, 0x0E, 0x45, 0x0F,
                0x44, 0x42, 0x4C, 0x80, 0x7F, 0x10, 0x5B, 0x02,
                0x7F, 0x07, 0x40, 0x41, 0x70, 0x00, WAIT, 0x0A,
                0x32, 0x44, 0x7F, 0x07, 0x40, 0x40, 0x7F, 0x06,
                0x62, 0xF0, 0x63, 0x00, 0x7F, 0x0D, 0x48, 0xC0,
                0x6F, 0xD5, 0x7F, 0x00, 0x5B, 0xA0, 0x4E, 0xA8,
                0x5A, 0x50, 0x40, 0x80, WAIT, 0xF0, 0x7F, 0x14,
                0x6F, 0x1C, 0x7F, 0x00,
            ]
        )
