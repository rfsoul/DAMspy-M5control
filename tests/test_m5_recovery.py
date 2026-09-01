import struct
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from m5_recovery import decode_status


class RecoveryStatusTest(unittest.TestCase):
    def test_decodes_diagnostic_status(self):
        version = b"ota-poc-9"
        body = bytearray(37 + len(version))
        body[0] = 1
        body[1] = 0x1F
        body[2] = 3
        body[3] = 1
        body[4] = 0
        body[5] = 3
        body[6] = 5
        body[7] = 3
        body[8] = 1
        body[9] = 1
        struct.pack_into("<IIHH", body, 10, 42, 9000, 0x19F7, 0x0058)
        body[22] = (-61) & 0xFF
        body[23] = 2
        struct.pack_into("<III", body, 24, 1234, 41, 1200)
        body[36] = len(version)
        body[37:] = version

        status = decode_status("node", bytes(body))
        self.assertEqual(status["version"], "ota-poc-9")
        self.assertEqual(status["state"], "usb-operation")
        self.assertEqual(status["last_rssi"], -61)
        self.assertEqual(status["busy_ms"], 1234)
        self.assertEqual(status["previous"]["request_id"], 41)

    def test_rejects_truncated_status(self):
        with self.assertRaises(RuntimeError):
            decode_status("node", b"\x01")


if __name__ == "__main__":
    unittest.main()
