from __future__ import annotations

import unittest

from tools.espnow_ota import cobs_decode, cobs_encode, crc16


class EspnowOtaFramingTest(unittest.TestCase):
    def test_crc16_ccitt_false_reference_vector(self) -> None:
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_cobs_round_trip_preserves_binary_data(self) -> None:
        cases = [b"", b"\x00", b"abc", b"a\x00b", bytes(range(256)), b"\x00" * 300]
        for payload in cases:
            with self.subTest(length=len(payload)):
                encoded = cobs_encode(payload)
                self.assertNotIn(0, encoded)
                self.assertEqual(cobs_decode(encoded), payload)

    def test_cobs_rejects_zero_code(self) -> None:
        with self.assertRaises(ValueError):
            cobs_decode(b"\x00")


if __name__ == "__main__":
    unittest.main()
