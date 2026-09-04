"""Mowgli USB CDC protocol encoder, parser, and typed telemetry decoders."""

import struct
from dataclasses import dataclass
from typing import List, Tuple


MAGIC = b"MW"
VERSION = 1
MAX_PAYLOAD = 324
HEADER_SIZE = 8
FRAME_OVERHEAD = 10

GET_INFO = 0x01
SET_WHEEL_SPEEDS = 0x10
STOP = 0x11
SET_DRIVE_ENABLE = 0x12
SET_BLADE_ENABLE = 0x13
CONFIG_GET = 0x20
CONFIG_SET = 0x21
SET_LED = 0x22
REBOOT = 0x23
PING = 0x24

DRIVE = 0x40
STATUS = 0x41
EXTERNAL_IMU = 0x42
ONBOARD_IMU = 0x43
PANEL = 0x44
INFO = 0x80
ACK = 0x81
CONFIG_VALUE = 0x82
PONG = 0x83

FEATURE_DRIVE = 1 << 0
FEATURE_BLADE = 1 << 1
FEATURE_EXTERNAL_IMU = 1 << 2
FEATURE_ONBOARD_IMU = 1 << 3
FEATURE_PANEL = 1 << 4
FEATURE_CONFIG = 1 << 5
FEATURE_CHARGING = 1 << 6
FEATURE_SAFETY_DISABLED = 1 << 7

ACK_OK = 0
ACK_NAMES = {
    0: "OK",
    1: "INVALID_LENGTH",
    2: "INVALID_VALUE",
    3: "UNSUPPORTED",
    4: "NOT_FOUND",
    5: "BUSY",
    6: "BAD_VERSION",
    7: "STORAGE_ERROR",
}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    message_type: int
    sequence: int
    payload: bytes = b""
    version: int = VERSION


def encode_frame(message_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if not 0 <= message_type <= 0xFF:
        raise ValueError("message_type must fit uint8")
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("sequence must fit uint16")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = struct.pack("<BBHH", VERSION, message_type, sequence, len(payload)) + payload
    return MAGIC + body + struct.pack("<H", crc16_ccitt_false(body))


class StreamParser:
    """Incremental parser that resynchronizes after noise and malformed frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.length_errors = 0
        self.version_errors = 0
        self.discarded_bytes = 0

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames: List[Frame] = []
        while True:
            magic_at = self._buffer.find(MAGIC)
            if magic_at < 0:
                keep = 1 if self._buffer.endswith(MAGIC[:1]) else 0
                self.discarded_bytes += len(self._buffer) - keep
                if keep:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break
            if magic_at:
                self.discarded_bytes += magic_at
                del self._buffer[:magic_at]
            if len(self._buffer) < HEADER_SIZE:
                break
            version, message_type, sequence, payload_length = struct.unpack_from(
                "<BBHH", self._buffer, 2
            )
            if payload_length > MAX_PAYLOAD:
                self.length_errors += 1
                del self._buffer[0]
                continue
            frame_length = FRAME_OVERHEAD + payload_length
            if len(self._buffer) < frame_length:
                break
            body = bytes(self._buffer[2:8 + payload_length])
            received_crc = struct.unpack_from("<H", self._buffer, 8 + payload_length)[0]
            if received_crc != crc16_ccitt_false(body):
                self.crc_errors += 1
                del self._buffer[0]
                continue
            del self._buffer[:frame_length]
            if version != VERSION:
                self.version_errors += 1
                continue
            frames.append(Frame(message_type, sequence, body[6:], version))
        return frames


@dataclass(frozen=True)
class Info:
    protocol_version: int
    firmware: Tuple[int, int, int]
    features: int
    max_wheel_mm_s: int
    ticks_per_meter: int
    wheel_track_mm: int
    drive_timeout_ms: int
    max_payload: int


@dataclass(frozen=True)
class Ack:
    command: int
    result: int
    detail: int


@dataclass(frozen=True)
class DriveTelemetry:
    uptime_ms: int
    left_ticks: int
    right_ticks: int
    measured_left_mm_s: int
    measured_right_mm_s: int
    commanded_left_mm_s: int
    commanded_right_mm_s: int
    command_age_ms: int
    flags: int
    left_power: int
    right_power: int


@dataclass(frozen=True)
class StatusTelemetry:
    uptime_ms: int
    battery_mv: int
    charge_mv: int
    charge_ma: int
    charge_pwm: int
    safety_flags: int
    system_flags: int
    temperature_centi_c: int
    firmware: Tuple[int, int, int]
    protocol_version: int
    crc_errors: int
    length_errors: int
    rx_overflows: int
    tx_drops: int


@dataclass(frozen=True)
class ExternalImuTelemetry:
    uptime_ms: int
    validity: int
    acceleration: Tuple[float, float, float]
    angular_velocity: Tuple[float, float, float]
    magnetic_field: Tuple[float, float, float]
    raw_magnetic_field: Tuple[float, float, float]


@dataclass(frozen=True)
class OnboardImuTelemetry:
    uptime_ms: int
    validity: int
    acceleration: Tuple[float, float, float]
    temperature_c: float


def _unpack_exact(fmt: str, payload: bytes) -> Tuple[object, ...]:
    expected = struct.calcsize(fmt)
    if len(payload) != expected:
        raise ValueError(f"expected {expected} payload bytes, got {len(payload)}")
    return struct.unpack(fmt, payload)


def decode_info(payload: bytes) -> Info:
    values = _unpack_exact("<BBBBIhHHHH", payload)
    return Info(values[0], (values[1], values[2], values[3]), *values[4:])


def decode_ack(payload: bytes) -> Ack:
    return Ack(*_unpack_exact("<BBB", payload))


def decode_drive(payload: bytes) -> DriveTelemetry:
    return DriveTelemetry(*_unpack_exact("<IiihhhhHHBB", payload))


def decode_status(payload: bytes) -> StatusTelemetry:
    values = _unpack_exact("<IHHHHHHhBBBBHHHH", payload)
    return StatusTelemetry(
        *values[:8], (values[8], values[9], values[10]), values[11], *values[12:]
    )


def decode_external_imu(payload: bytes) -> ExternalImuTelemetry:
    values = _unpack_exact("<IH12f", payload)
    return ExternalImuTelemetry(
        values[0], values[1], tuple(values[2:5]), tuple(values[5:8]),
        tuple(values[8:11]), tuple(values[11:14])
    )


def decode_onboard_imu(payload: bytes) -> OnboardImuTelemetry:
    values = _unpack_exact("<IH4f", payload)
    return OnboardImuTelemetry(values[0], values[1], tuple(values[2:5]), values[5])


def decode_panel(payload: bytes) -> Tuple[int, Tuple[int, ...]]:
    if len(payload) < 4 or (len(payload) - 4) % 2:
        raise ValueError("invalid panel payload length")
    uptime = struct.unpack_from("<I", payload)[0]
    count = (len(payload) - 4) // 2
    buttons = struct.unpack_from(f"<{count}H", payload, 4) if count else ()
    return uptime, buttons


def wheel_speed_payload(left_mm_s: int, right_mm_s: int) -> bytes:
    if not -32768 <= left_mm_s <= 32767 or not -32768 <= right_mm_s <= 32767:
        raise ValueError("wheel speed must fit int16")
    return struct.pack("<hh", left_mm_s, right_mm_s)


def drive_enable_payload(enabled: bool) -> bytes:
    return struct.pack("<B", bool(enabled))


def set_led_payload(index: int, mode: int, clear_all: bool = False, chirp: bool = False) -> bytes:
    if not 0 <= index <= 0xFF or not 0 <= mode <= 3:
        raise ValueError("invalid LED command")
    return struct.pack("<BBB", index, mode, int(clear_all) | (int(chirp) << 1))


def config_get_payload(name: str) -> bytes:
    encoded = name.encode("utf-8")
    if not 1 <= len(encoded) <= 63:
        raise ValueError("configuration name must encode to 1..63 bytes")
    return bytes((len(encoded),)) + encoded


def config_set_payload(value_type: int, name: str, data: bytes) -> bytes:
    encoded = name.encode("utf-8")
    if not 0 <= value_type <= 5 or not 1 <= len(encoded) <= 63 or len(data) > 255:
        raise ValueError("invalid configuration value")
    required = {0: 4, 1: 4, 2: 4, 3: 8}.get(value_type)
    if required is not None and len(data) != required:
        raise ValueError(f"configuration type {value_type} requires {required} bytes")
    return bytes((value_type, len(encoded), len(data))) + encoded + data
