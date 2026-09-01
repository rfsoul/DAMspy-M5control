#!/usr/bin/env python3
"""Query and recover the StickS3 Gateway and CoreS3 Node remotely."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct
import time

import serial

from espnow_ota import CRC, HEADER, cobs_decode, cobs_encode, crc16

MAGIC = 0xD1
NODE_DIAG_REQUEST, NODE_DIAG_RESPONSE = 0x07, 0x08
NODE_RESET_REQUEST, NODE_RESET_RESPONSE = 0x09, 0x0A
GATEWAY_DIAG_REQUEST, GATEWAY_DIAG_RESPONSE = 0x0B, 0x0C
GATEWAY_RESET_REQUEST, GATEWAY_RESET_RESPONSE = 0x0D, 0x0E
GATEWAY_SCAN_REQUEST, GATEWAY_SCAN_RESPONSE = 0x0F, 0x10
DIAG_FIXED_SIZE = 37

STATES = {
    0: "booting", 1: "ready", 2: "waiting", 3: "usb-operation",
    4: "ota", 5: "error", 6: "resetting",
}

RESET_REASONS = {
    1: "power-on", 3: "software", 4: "panic", 5: "interrupt-watchdog",
    6: "task-watchdog", 7: "other-watchdog", 8: "deep-sleep",
    9: "brownout", 10: "sdio", 11: "usb", 12: "jtag",
    13: "efuse", 14: "power-glitch", 15: "cpu-lockup",
}


class RecoveryLink:
    def __init__(self, port: str) -> None:
        self.port = port
        self.serial = serial.Serial(port, 115200, timeout=0.2, write_timeout=2)
        self.request_id = int(time.time()) & 0xFFFFFFFF or 1
        self.buffer = bytearray()
        self.serial.reset_input_buffer()

    def close(self) -> None:
        self.serial.close()

    def exchange(self, request_type: int, response_type: int,
                 timeout: float = 7.0) -> bytes:
        request_id = self.request_id
        self.request_id = (request_id + 1) & 0xFFFFFFFF or 1
        inner = HEADER.pack(MAGIC, request_type, request_id, 0)
        wire = cobs_encode(inner + CRC.pack(crc16(inner))) + b"\0"
        self.serial.write(wire)
        self.serial.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            delimiter = self.buffer.find(0)
            if delimiter < 0:
                chunk = self.serial.read(self.serial.in_waiting or 1)
                if chunk:
                    self.buffer.extend(chunk)
                continue
            encoded = bytes(self.buffer[:delimiter])
            del self.buffer[:delimiter + 1]
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
            magic, kind, correlated, length = HEADER.unpack(candidate[:HEADER.size])
            body = candidate[HEADER.size:]
            if (
                magic == MAGIC and kind == response_type and
                correlated == request_id and length == len(body)
            ):
                return body
        raise TimeoutError(f"no response for request 0x{request_type:02x}")


def decode_status(device: str, body: bytes) -> dict:
    if len(body) < DIAG_FIXED_SIZE or body[0] != 1:
        raise RuntimeError(f"invalid {device} diagnostic response ({len(body)} bytes)")
    version_length = body[36]
    if len(body) != DIAG_FIXED_SIZE + version_length:
        raise RuntimeError(f"invalid {device} diagnostic version length")
    flags = body[1]
    status = {
        "device": device,
        "version": body[37:].decode(errors="replace"),
        "state": STATES.get(body[2], f"unknown-{body[2]}"),
        "uptime_ms": struct.unpack_from("<I", body, 14)[0],
        "reset_reason": RESET_REASONS.get(body[5], f"unknown-{body[5]}"),
        "usb_connected": bool(flags & 0x01),
        "hid_ready": bool(flags & 0x02),
        "busy": bool(flags & 0x04),
        "busy_ms": struct.unpack_from("<I", body, 24)[0],
        "last_type": body[3],
        "last_result": body[4],
        "last_request_id": struct.unpack_from("<I", body, 10)[0],
        "rx_valid": bool(flags & 0x08),
        "last_rssi": struct.unpack_from("<b", body, 22)[0],
        "rx_age_seconds": body[23],
        "vid": struct.unpack_from("<H", body, 18)[0],
        "pid": struct.unpack_from("<H", body, 20)[0],
        "previous": {
            "valid": bool(flags & 0x10),
            "state": STATES.get(body[6], f"unknown-{body[6]}"),
            "last_type": body[7],
            "last_result": body[8],
            "reset_requested": bool(body[9]),
            "request_id": struct.unpack_from("<I", body, 28)[0],
            "busy_ms": struct.unpack_from("<I", body, 32)[0],
        },
    }
    if device == "gateway":
        status["station_mac"] = ":".join(f"{byte:02x}" for byte in body[28:34])
        status["tx_callback_valid"] = bool(body[6])
        status["tx_callback_status"] = "ok" if body[7] == 0 else "fail"
        status["tx_callback_count"] = struct.unpack_from("<H", body, 8)[0]
        status.pop("previous", None)
    return status


def append_snapshot(path: Path, event: str, statuses: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    record = {"time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
              "event": event, "statuses": statuses}
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, sort_keys=True) + "\n")


def print_status(status: dict) -> None:
    print(json.dumps(status, indent=2, sort_keys=True))


def query(link: RecoveryLink, device: str) -> dict:
    if device == "gateway":
        return decode_status(device, link.exchange(
            GATEWAY_DIAG_REQUEST, GATEWAY_DIAG_RESPONSE, 2))
    return decode_status(device, link.exchange(
        NODE_DIAG_REQUEST, NODE_DIAG_RESPONSE, 7))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=(
        "status", "scan-node", "reset-node", "reset-gateway", "hard-reset-gateway"))
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--log", type=Path,
        default=Path.home() / ".local/state/damspy/m5-recovery.jsonl")
    args = parser.parse_args()

    if args.command == "hard-reset-gateway":
        port = serial.Serial(args.port, 115200)
        port.dtr = False
        port.rts = True
        time.sleep(0.15)
        port.rts = False
        time.sleep(1)
        port.close()
        print("Gateway USB reset asserted; serial released")
        return 0

    link = RecoveryLink(args.port)
    try:
        if args.command == "scan-node":
            body = link.exchange(GATEWAY_SCAN_REQUEST, GATEWAY_SCAN_RESPONSE, 12)
            if len(body) != 9:
                raise RuntimeError(f"invalid scan response ({len(body)} bytes)")
            if body[0] != 0:
                print("No Core response on channels 1-13 using LR 250 kbps or 1 Mbps 11b")
                return 1
            mac = ":".join(f"{byte:02x}" for byte in body[3:9])
            mode = "LR 250 kbps" if body[2] else "11b 1 Mbps"
            print(f"Core discovered: mac={mac} channel={body[1]} mode={mode}")
            return 0

        if args.command == "status":
            statuses = []
            for device in ("gateway", "node"):
                try:
                    statuses.append(query(link, device))
                except Exception as error:
                    statuses.append({"device": device, "query_error": str(error)})
            append_snapshot(args.log, "status", statuses)
            for status in statuses:
                print_status(status)
            return 1 if any("query_error" in status for status in statuses) else 0

        target = "node" if args.command == "reset-node" else "gateway"
        statuses = []
        for device in ("gateway", "node"):
            try:
                statuses.append(query(link, device))
            except Exception as error:
                statuses.append({"device": device, "query_error": str(error)})
        append_snapshot(args.log, f"before-reset-{target}", statuses)
        print("Pre-reset snapshot saved:")
        for status in statuses:
            print_status(status)

        if target == "node":
            body = link.exchange(NODE_RESET_REQUEST, NODE_RESET_RESPONSE, 7)
            if body != b"\0":
                raise RuntimeError(f"invalid Node reset acknowledgement: {body.hex()}")
            print("Node reset acknowledged; waiting for reboot")
            time.sleep(3)
            deadline = time.monotonic() + 20
            while True:
                try:
                    after = query(link, "node")
                    break
                except Exception:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(1)
        else:
            body = link.exchange(GATEWAY_RESET_REQUEST, GATEWAY_RESET_RESPONSE, 2)
            if body != b"\0":
                raise RuntimeError(f"invalid Gateway reset acknowledgement: {body.hex()}")
            print("Gateway reset acknowledged; waiting for USB serial reboot")
            link.close()
            time.sleep(3)
            deadline = time.monotonic() + 20
            while True:
                try:
                    link = RecoveryLink(args.port)
                    after = query(link, "gateway")
                    break
                except Exception:
                    try:
                        link.close()
                    except Exception:
                        pass
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(1)

        append_snapshot(args.log, f"after-reset-{target}", [after])
        print("Post-reset status:")
        print_status(after)
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
