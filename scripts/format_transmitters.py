#!/usr/bin/env python3
"""
Format transmitters from SatNOGS database JSON.

Copyright (c) 2026 Andrew C. Young <andrew@vaelen.org>
SPDX-License-Identifier: MIT
"""

import json
import os

def format_value(value):
    """Return empty string for None, otherwise str(value)."""
    return "" if value is None else str(value)

def format_frequency(freq_hz):
    """Convert Hz to kHz, return empty string if None."""
    if freq_hz is None:
        return ""
    return str(freq_hz / 1000)

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(script_dir, "..", "docs", "transmitters.json")

    with open(json_path, "r") as f:
        transmitters = json.load(f)

    # Filter and sort: by service, then by norad_cat_id
    filtered = [tx for tx in transmitters
                if tx.get("alive", False) and not tx.get("unconfirmed", False)]
    filtered.sort(key=lambda tx: (tx.get("service") or "", tx.get("norad_cat_id") or 0))

    for tx in filtered:
        norad_cat_id = format_value(tx.get("norad_cat_id"))
        service = format_value(tx.get("service"))
        description = format_value(tx.get("description"))
        invert = format_value(tx.get("invert"))
        downlink_low = format_frequency(tx.get("downlink_low"))
        mode = format_value(tx.get("mode"))
        baud = format_value(tx.get("baud"))
        uplink_low = tx.get("uplink_low")
        uplink_mode = format_value(tx.get("uplink_mode"))

        # Header line
        print(f"# {norad_cat_id} - {service} - {description} - (Invert: {invert})")

        # Downlink line
        print(f"track = {norad_cat_id}|Down|{downlink_low}|{mode}|{baud}|{mode}")

        # Uplink line (only if uplink_low is not null)
        if uplink_low is not None:
            uplink_freq = format_frequency(uplink_low)
            print(f"track = {norad_cat_id}|Up|{uplink_freq}|{uplink_mode}|{baud}|{uplink_mode}")

if __name__ == "__main__":
    main()
