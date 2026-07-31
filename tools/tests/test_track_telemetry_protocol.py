#!/usr/bin/env python3
"""Independent host tests for the UART3 telemetry stream parser."""

from __future__ import annotations

from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from track_telemetry_protocol import (  # noqa: E402
    CSV_COLUMNS,
    CRC,
    FRAME_LENGTH,
    FRAME_TYPE_CONTROL,
    HEADER,
    PAYLOAD,
    PAYLOAD_LENGTH,
    PROTOCOL_VERSION,
    SYNC,
    TelemetryParser,
    crc16_ccitt,
    sample_to_csv_row,
)


def make_frame(sequence: int, *, event: int = 0, raw_error: int = -1234) -> bytes:
    payload = PAYLOAD.pack(
        1000,
        200,
        345,
        2,
        0,
        event,
        0xE7,
        0x18,
        2,
        3,
        1,
        6,
        4,
        0x53,
        20,
        7,
        raw_error,
        -1150,
        -1025,
        175,
        -23,
        4,
        -19,
        280,
        240,
        235,
        254,
        216,
        249,
        211,
        123,
        118,
        800,
        760,
    )
    header = HEADER.pack(
        SYNC, PROTOCOL_VERSION, FRAME_TYPE_CONTROL, PAYLOAD_LENGTH, sequence
    )
    without_crc = header + payload
    return without_crc + CRC.pack(crc16_ccitt(without_crc[2:]))


def main() -> None:
    assert PAYLOAD.size == PAYLOAD_LENGTH
    assert FRAME_LENGTH == 74

    parser = TelemetryParser()
    frame = make_frame(10, event=1)
    samples = []
    for chunk in (frame[:1], frame[1:7], frame[7:33], frame[33:]):
        samples.extend(parser.feed(chunk))
    assert len(samples) == 1
    sample = samples[0]
    assert sample.sequence == 10
    assert sample.packet_gap == 0
    assert sample.event == 1
    assert sample.raw_track_mask == 0xE7
    assert sample.track_mask == 0x18
    assert sample.raw_channels() == (1, 1, 1, 0, 0, 1, 1, 1)
    assert sample.black_channels() == (0, 0, 0, 1, 1, 0, 0, 0)
    assert sample.raw_error == -12.34
    assert sample.filtered_error == -10.25
    assert sample.requested_base_mps == 0.280
    assert sample.left_pwm == 800
    assert len(sample_to_csv_row(sample, "正常完成")) == len(CSV_COLUMNS)

    # Two skipped sequence numbers must be reported on the next valid frame.
    gap_sample = parser.feed(make_frame(13))[0]
    assert gap_sample.packet_gap == 2

    # Corrupted data followed by a good frame must resynchronize without a restart.
    corrupted = bytearray(make_frame(14))
    corrupted[40] ^= 0x80
    recovered = parser.feed(b"noise" + bytes(corrupted) + make_frame(15))
    assert len(recovered) == 1
    assert recovered[0].sequence == 15
    assert parser.crc_errors >= 1
    assert parser.discarded_bytes >= 5

    print("[OK] Python UART3 telemetry parser, CRC recovery and gap tracking passed.")


if __name__ == "__main__":
    main()
