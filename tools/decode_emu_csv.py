#!/usr/bin/env python3
"""Decode raw ECUMaster EMU Black CAN CSV logs.

Input format is the firmware's CAN-only raw log:
timestamp_ms,seq,std_id,dlc,d0,d1,d2,d3,d4,d5,d6,d7

The decoder follows the EMU Black default stream layout at base ID 0x600.
It writes one output row per input CAN frame. Fields that are not carried by
that frame are left blank.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Callable


BASE_ID = 0x600

META_FIELDS = ["timestamp_ms", "seq", "std_id", "dlc"]

DECODED_FIELDS = [
    "rpm",
    "tps_percent",
    "iat_c",
    "map_kpa",
    "injector_pw_ms",
    "ain1_v",
    "ain2_v",
    "ain3_v",
    "ain4_v",
    "vspd_kmh",
    "baro_kpa",
    "oil_temp_c",
    "oil_pressure_bar",
    "fuel_pressure_bar",
    "clt_c",
    "ignition_angle_deg",
    "dwell_ms",
    "lambda",
    "lambda_correction_percent",
    "egt1_c",
    "egt2_c",
    "gear",
    "ecu_temp_c",
    "battery_v",
    "error_flags",
    "flags1",
    "ethanol_percent",
    "dbw_position_percent",
    "dbw_target_percent",
    "tc_drpm_raw",
    "tc_drpm",
    "tc_torque_reduction_percent",
    "pit_limiter_torque_reduction_percent",
    "ain5_v",
    "ain6_v",
    "outflags1",
    "outflags2",
    "outflags3",
    "outflags4",
    "boost_target_kpa",
    "pwm1_dc_percent",
    "dsg_mode",
    "lambda_target",
    "pwm2_dc_percent",
    "fuel_used_l",
]


def u16_le(data: list[int], offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def s16_le(data: list[int], offset: int) -> int:
    value = u16_le(data, offset)
    return value - 0x10000 if value & 0x8000 else value


def s8(value: int) -> int:
    return value - 0x100 if value & 0x80 else value


def fmt(value: float | int | str, digits: int | None = None) -> str:
    if isinstance(value, float):
        if digits is None:
            return f"{value:g}"
        return f"{value:.{digits}f}"
    return str(value)


def hex_byte(value: int) -> str:
    return f"0x{value:02X}"


def hex_u16(value: int) -> str:
    return f"0x{value:04X}"


def decode_600(data: list[int]) -> dict[str, str]:
    return {
        "rpm": fmt(u16_le(data, 0)),
        "tps_percent": fmt(data[2] * 0.5, 1),
        "iat_c": fmt(s8(data[3])),
        "map_kpa": fmt(u16_le(data, 4)),
        "injector_pw_ms": fmt(u16_le(data, 6) / 62.0, 4),
    }


def decode_601(data: list[int]) -> dict[str, str]:
    return {
        "ain1_v": fmt(u16_le(data, 0) * 5.0 / 1024.0, 4),
        "ain2_v": fmt(u16_le(data, 2) * 5.0 / 1024.0, 4),
        "ain3_v": fmt(u16_le(data, 4) * 5.0 / 1024.0, 4),
        "ain4_v": fmt(u16_le(data, 6) * 5.0 / 1024.0, 4),
    }


def decode_602(data: list[int]) -> dict[str, str]:
    return {
        "vspd_kmh": fmt(u16_le(data, 0)),
        "baro_kpa": fmt(data[2]),
        "oil_temp_c": fmt(data[3]),
        "oil_pressure_bar": fmt(data[4] / 16.0, 4),
        "fuel_pressure_bar": fmt(data[5] / 16.0, 4),
        "clt_c": fmt(s16_le(data, 6)),
    }


def decode_603(data: list[int]) -> dict[str, str]:
    return {
        "ignition_angle_deg": fmt(s8(data[0]) * 0.5, 1),
        "dwell_ms": fmt(data[1] / 20.0, 3),
        "lambda": fmt(data[2] / 128.0, 4),
        "lambda_correction_percent": fmt(data[3] * 0.5, 1),
        "egt1_c": fmt(u16_le(data, 4)),
        "egt2_c": fmt(u16_le(data, 6)),
    }


def decode_604(data: list[int]) -> dict[str, str]:
    error_flags = u16_le(data, 4)
    return {
        "gear": fmt(data[0]),
        "ecu_temp_c": fmt(s8(data[1])),
        "battery_v": fmt(u16_le(data, 2) * 27.0 / 1000.0, 3),
        "error_flags": hex_u16(error_flags),
        "flags1": hex_byte(data[6]),
        "ethanol_percent": fmt(data[7]),
    }


def decode_605(data: list[int]) -> dict[str, str]:
    return {
        "dbw_position_percent": fmt(data[0] * 0.5, 1),
        "dbw_target_percent": fmt(data[1] * 0.5, 1),
        "tc_drpm_raw": fmt(s16_le(data, 2)),
        "tc_drpm": fmt(u16_le(data, 4)),
        "tc_torque_reduction_percent": fmt(data[6]),
        "pit_limiter_torque_reduction_percent": fmt(data[7]),
    }


def decode_606(data: list[int]) -> dict[str, str]:
    return {
        "ain5_v": fmt(u16_le(data, 0) * 5.0 / 1024.0, 4),
        "ain6_v": fmt(u16_le(data, 2) * 5.0 / 1024.0, 4),
        "outflags1": hex_byte(data[4]),
        "outflags2": hex_byte(data[5]),
        "outflags3": hex_byte(data[6]),
        "outflags4": hex_byte(data[7]),
    }


def decode_607(data: list[int]) -> dict[str, str]:
    return {
        "boost_target_kpa": fmt(u16_le(data, 0)),
        "pwm1_dc_percent": fmt(data[2]),
        "dsg_mode": fmt(data[3]),
        "lambda_target": fmt(data[4] / 100.0, 3),
        "pwm2_dc_percent": fmt(data[5]),
        "fuel_used_l": fmt(u16_le(data, 6) / 100.0, 2),
    }


DECODERS: dict[int, Callable[[list[int]], dict[str, str]]] = {
    BASE_ID + 0: decode_600,
    BASE_ID + 1: decode_601,
    BASE_ID + 2: decode_602,
    BASE_ID + 3: decode_603,
    BASE_ID + 4: decode_604,
    BASE_ID + 5: decode_605,
    BASE_ID + 6: decode_606,
    BASE_ID + 7: decode_607,
}


def parse_id(value: str) -> int:
    return int(value.strip(), 0)


def parse_data(row: dict[str, str]) -> list[int]:
    data = []
    for index in range(8):
        raw = row.get(f"d{index}", "").strip()
        data.append(int(raw, 16) if raw else 0)
    return data


def decode_file(input_path: Path, output_path: Path) -> None:
    with input_path.open(newline="") as input_file, output_path.open(
        "w", newline=""
    ) as output_file:
        reader = csv.DictReader(input_file)
        writer = csv.DictWriter(output_file, fieldnames=META_FIELDS + DECODED_FIELDS)
        writer.writeheader()

        for row in reader:
            std_id = parse_id(row["std_id"])
            decoded = {field: "" for field in DECODED_FIELDS}
            decoder = DECODERS.get(std_id)
            if decoder is not None:
                decoded.update(decoder(parse_data(row)))

            writer.writerow(
                {
                    "timestamp_ms": row.get("timestamp_ms", ""),
                    "seq": row.get("seq", ""),
                    "std_id": f"0x{std_id:03X}",
                    "dlc": row.get("dlc", ""),
                    **decoded,
                }
            )


def default_output_path(input_path: Path) -> Path:
    return input_path.with_name(f"{input_path.stem}_decoded.csv")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Decode raw EMU Black CAN CSV logs from the STM32 logger."
    )
    parser.add_argument("input", type=Path, help="Raw EMU.CSV/EMUxxx.CSV file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Decoded CSV path. Defaults to INPUT_STEM_decoded.csv.",
    )
    args = parser.parse_args()

    input_path = args.input
    output_path = args.output or default_output_path(input_path)
    decode_file(input_path, output_path)
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
