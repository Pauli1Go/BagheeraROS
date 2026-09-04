import struct
import unittest

from bagheera_base.protocol import (
    GET_INFO,
    INFO,
    StreamParser,
    config_set_payload,
    crc16_ccitt_false,
    decode_drive,
    decode_info,
    encode_frame,
)


class ProtocolTest(unittest.TestCase):
    def test_documented_get_info_vector(self):
        self.assertEqual(
            encode_frame(GET_INFO, 1),
            bytes.fromhex("4d570101010000005597"),
        )

    def test_standard_crc_vector(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_fragmented_and_noisy_stream(self):
        expected = encode_frame(INFO, 7, bytes(range(18)))
        parser = StreamParser()
        result = []
        for fragment in (b"noiseM", b"x", expected[:3], expected[3:9], expected[9:]):
            result.extend(parser.feed(fragment))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].message_type, INFO)
        self.assertEqual(result[0].sequence, 7)
        self.assertEqual(result[0].payload, bytes(range(18)))
        self.assertGreater(parser.discarded_bytes, 0)

    def test_bad_crc_resynchronizes(self):
        bad = bytearray(encode_frame(GET_INFO, 1))
        bad[-1] ^= 0x80
        good = encode_frame(GET_INFO, 2)
        parser = StreamParser()
        frames = parser.feed(bytes(bad) + good)
        self.assertEqual([frame.sequence for frame in frames], [2])
        self.assertEqual(parser.crc_errors, 1)

    def test_info_decoder(self):
        payload = struct.pack("<BBBBIhHHHH", 1, 2, 0, 0, 0xE5, 600, 250, 285, 1000, 324)
        info = decode_info(payload)
        self.assertEqual(info.firmware, (2, 0, 0))
        self.assertEqual(info.max_wheel_mm_s, 600)
        self.assertEqual(info.wheel_track_mm, 285)

    def test_drive_decoder(self):
        payload = struct.pack("<IiihhhhHHBB", 10, -5, 8, -100, 101, -90, 91, 12, 7, 1, 2)
        drive = decode_drive(payload)
        self.assertEqual(drive.left_ticks, -5)
        self.assertEqual(drive.commanded_right_mm_s, 91)
        self.assertEqual(drive.flags, 7)

    def test_config_numeric_lengths_are_enforced(self):
        with self.assertRaises(ValueError):
            config_set_payload(2, "gain", b"123")
        self.assertEqual(config_set_payload(2, "gain", b"1234")[:3], b"\x02\x04\x04")


if __name__ == "__main__":
    unittest.main()
