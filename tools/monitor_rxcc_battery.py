#!/usr/bin/env python3
"""Log RXCC battery telemetry from the rpicontrol web API."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import time
import urllib.error
import urllib.request


def read_battery(base_url: str) -> dict:
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}/api/battery/rxcc",
        data=b"{}",
        headers={"content-type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=20) as response:
        body = json.loads(response.read())
        if response.status != 200 or body.get("status") != "ok":
            raise RuntimeError(f"battery request failed: HTTP {response.status}: {body}")
        return body


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:8000")
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--normal-seconds", type=float, default=900)
    parser.add_argument("--drop-seconds", type=float, default=60)
    parser.add_argument("--fast-below-mv", type=int, default=3600)
    args = parser.parse_args()

    fields = [
        "timestamp",
        "elapsed_seconds",
        "battery_mv",
        "temperature_c",
        "charge_state",
        "charge_state_code",
        "charge_current_ma",
        "sample_interval_seconds",
        "state",
        "error",
    ]
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    new_file = not args.csv.exists() or args.csv.stat().st_size == 0
    started = time.monotonic()
    dropping = False

    with args.csv.open("a", newline="", buffering=1) as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        if new_file:
            writer.writeheader()

        while True:
            now = datetime.now().astimezone().isoformat(timespec="seconds")
            interval = args.drop_seconds if dropping else args.normal_seconds
            row = {
                "timestamp": now,
                "elapsed_seconds": round(time.monotonic() - started, 1),
                "sample_interval_seconds": interval,
                "state": "DROP_MONITOR" if dropping else "BASELINE",
                "error": "",
            }
            try:
                battery = read_battery(args.base_url)
                voltage = int(battery["battery_mv"])
                if not dropping and voltage <= args.fast_below_mv:
                    dropping = True
                    interval = args.drop_seconds
                    row["sample_interval_seconds"] = interval
                    row["state"] = "LOW_VOLTAGE_DETECTED"
                row.update(
                    battery_mv=voltage,
                    temperature_c=battery.get("temperature_c"),
                    charge_state=battery.get("charge_state"),
                    charge_state_code=battery.get("charge_state_code"),
                    charge_current_ma=battery.get("charge_current_ma"),
                )
                print(
                    f"{now} battery={voltage}mV fast_below={args.fast_below_mv}mV "
                    f"charge={battery.get('charge_state')} "
                    f"current={battery.get('charge_current_ma')}mA "
                    f"state={row['state']} next={interval:g}s",
                    flush=True,
                )
            except Exception as error:
                row["state"] = "ERROR"
                row["error"] = str(error)
                print(f"{now} ERROR {error}; next={interval:g}s", flush=True)
            writer.writerow(row)
            time.sleep(interval)


if __name__ == "__main__":
    raise SystemExit(main())
