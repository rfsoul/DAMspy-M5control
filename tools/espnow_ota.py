#!/usr/bin/env python3
"""Upload a CoreS3 application image through StickS3 serial and ESP-NOW."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import struct
import time

import serial

MAGIC = 0xD3
HEADER = struct.Struct("<BBIH")
CRC = struct.Struct("<H")
BEGIN_REQUEST, BEGIN_RESPONSE = 0x01, 0x02
DATA_REQUEST, DATA_RESPONSE = 0x03, 0x04
END_REQUEST, END_RESPONSE = 0x05, 0x06
ABORT_REQUEST, ABORT_RESPONSE = 0x07, 0x08
STATUS_REQUEST, STATUS_RESPONSE = 0x09, 0x0A
RESULT_OK = 0
MAX_CHUNK = 238


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def cobs_encode(data: bytes) -> bytes:
    output = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0 or index + code > len(data) + 1:
            raise ValueError("invalid COBS frame")
        index += 1
        output.extend(data[index:index + code - 1])
        index += code - 1
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


class Uploader:
    def __init__(self, port: str, retries: int) -> None:
        self.serial = serial.Serial(port, 115200, timeout=0.2, write_timeout=2)
        self.request_id = int(time.time()) & 0xFFFFFFFF or 1
        self.retries = retries
        self._receive_buffer = bytearray()
        self.serial.reset_input_buffer()

    def close(self) -> None:
        self.serial.close()

    def exchange(self, message_type: int, response_type: int, body: bytes,
                 timeout: float) -> bytes:
        request_id = self.request_id
        self.request_id = (self.request_id + 1) & 0xFFFFFFFF or 1
        inner = HEADER.pack(MAGIC, message_type, request_id, len(body)) + body
        wire = cobs_encode(inner + CRC.pack(crc16(inner))) + b"\x00"
        for attempt in range(1, self.retries + 1):
            self.serial.write(wire)
            self.serial.flush()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                delimiter = self._receive_buffer.find(0)
                if delimiter < 0:
                    available = self.serial.in_waiting
                    chunk = self.serial.read(available if available > 0 else 1)
                    if chunk:
                        self._receive_buffer.extend(chunk)
                    continue
                encoded = bytes(self._receive_buffer[:delimiter])
                del self._receive_buffer[:delimiter + 1]
                if not encoded:
                    continue
                try:
                    raw = cobs_decode(encoded)
                except ValueError:
                    continue
                if len(raw) < HEADER.size + CRC.size:
                    continue
                candidate, supplied = raw[:-2], CRC.unpack(raw[-2:])[0]
                if crc16(candidate) != supplied:
                    continue
                magic, kind, correlated_id, length = HEADER.unpack(candidate[:HEADER.size])
                response = candidate[HEADER.size:]
                if length != len(response):
                    continue
                if magic == MAGIC and kind == response_type and correlated_id == request_id:
                    return response
            print(f"retry {attempt}/{self.retries}: no response for type 0x{message_type:02X}")
        raise TimeoutError(f"no correlated response for type 0x{message_type:02X}")

    @staticmethod
    def ack(body: bytes) -> tuple[int, int]:
        if len(body) != 5:
            raise RuntimeError(f"invalid ACK length {len(body)}")
        return body[0], struct.unpack_from("<I", body, 1)[0]

    def status(self) -> tuple[bool, int, int, str]:
        body = self.exchange(STATUS_REQUEST, STATUS_RESPONSE, b"", 5)
        if len(body) < 11 or body[0] != RESULT_OK or len(body) != 11 + body[10]:
            raise RuntimeError("invalid OTA STATUS response")
        return bool(body[1]), struct.unpack_from("<I", body, 2)[0], \
            struct.unpack_from("<I", body, 6)[0], body[11:].decode(errors="replace")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--port", required=True)
    parser.add_argument("--retries", type=int, default=8)
    parser.add_argument("--expect-version")
    args = parser.parse_args()
    image = args.image.read_bytes()
    digest = hashlib.sha256(image).digest()
    print(f"image: {args.image} ({len(image)} bytes, sha256 {digest.hex()})")
    uploader = Uploader(args.port, args.retries)
    try:
        active, offset, total, version = uploader.status()
        print(f"Core before: version={version!r} active={active} {offset}/{total}")
        result, offset = uploader.ack(uploader.exchange(
            BEGIN_REQUEST, BEGIN_RESPONSE, struct.pack("<I", len(image)) + digest, 35))
        if result != RESULT_OK or offset > len(image):
            raise RuntimeError(f"BEGIN failed result={result} offset={offset}")
        started = time.monotonic()
        while offset < len(image):
            chunk = image[offset:offset + MAX_CHUNK]
            result, next_offset = uploader.ack(uploader.exchange(
                DATA_REQUEST, DATA_RESPONSE, struct.pack("<I", offset) + chunk, 12))
            if result != RESULT_OK or next_offset <= offset or next_offset > len(image):
                raise RuntimeError(f"DATA failed result={result} offset={offset} next={next_offset}")
            offset = next_offset
            if offset == len(image) or offset % (MAX_CHUNK * 100) < MAX_CHUNK:
                elapsed = max(time.monotonic() - started, 0.001)
                print(f"{offset}/{len(image)} bytes ({offset * 100 / len(image):.1f}%, {offset / elapsed:.0f} B/s)")
        result, final_offset = uploader.ack(uploader.exchange(
            END_REQUEST, END_RESPONSE, b"", 15))
        if result != RESULT_OK or final_offset != len(image):
            raise RuntimeError(f"END failed result={result} offset={final_offset}")
        print("Core accepted and verified image; waiting for reboot...")
        time.sleep(4)
        deadline = time.monotonic() + 20
        while True:
            try:
                active, offset, total, version = uploader.status()
                break
            except (TimeoutError, RuntimeError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(1)
        print(f"Core after: version={version!r} active={active} {offset}/{total}")
        if args.expect_version and version != args.expect_version:
            raise RuntimeError(f"expected version {args.expect_version!r}, got {version!r}")
        print("OTA PASS")
        return 0
    finally:
        uploader.close()


if __name__ == "__main__":
    raise SystemExit(main())
