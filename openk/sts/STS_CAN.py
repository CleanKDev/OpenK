import logging
import time
from dataclasses import dataclass
from typing import Dict

import numpy as np
from serial import Serial

logger = logging.getLogger(__name__)


@dataclass
class StsMotor:
    """Lightweight container for an STS motor id."""

    id: int


class StsController:
    """Low-level STS CAN protocol handler."""

    send_data_frame = np.array(
        [
            0x55,
            0xAA,
            0x1E,
            0x03,
            0x01,
            0x00,
            0x00,
            0x00,
            0x0A,
            0x00,
            0x00,
            0x00,
            0x00,
            0,
            0,
            0,
            0,
            0x00,
            0x08,
            0x00,
            0x00,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0x00,
        ],
        np.uint8,
    )

    def __init__(self, serial_device: Serial):
        self.serial_ = serial_device
        self.data_save = bytes()
        self.positions: Dict[int, int] = {}

        if self.serial_.is_open:
            self.serial_.close()

    def write_position(self, controller_id: int, motor_id: int, angle: int) -> None:
        data_buf = np.array([0x00] * 8, np.uint8)
        data_buf[0] = (motor_id >> 8) & 0xFF
        data_buf[1] = motor_id & 0xFF
        data_buf[2] = (angle >> 8) & 0xFF
        data_buf[3] = angle & 0xFF
        data_buf[4] = 0
        data_buf[5] = 0xFF
        data_buf[6] = 0
        data_buf[7] = 0
        self._send_data(controller_id, data_buf)

    def read_position(self, controller_id: int, motor_id: int, timeout: float = 0.1) -> int | None:
        data_buf = np.array([0x00] * 8, np.uint8)
        data_buf[0] = (motor_id >> 8) & 0xFF
        data_buf[1] = motor_id & 0xFF
        data_buf[4] = 0xFF
        self._send_data(controller_id, data_buf)

        start_time = time.time()
        while time.time() - start_time < timeout:
            self.recv()
            if motor_id in self.positions:
                return self.positions.pop(motor_id)
            time.sleep(0.005)
        return None

    def recv(self) -> None:
        data_recv = b"".join([self.data_save, self.serial_.read_all()])
        packets = self._extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]

            if CMD == 0x11 and (CANID & 0xFFFF) == 0xFF00:
                # STS response: data[0:2] is motor ID, data[2:4] is position
                resp_motor_id = (int(data[0]) << 8) | data[1]
                q_uint = (np.uint16(data[2]) << 8) | data[3]
                self.positions[resp_motor_id] = int(q_uint)

    def _send_data(self, motor_id: int, data: np.ndarray) -> None:
        self.send_data_frame[13] = motor_id & 0xFF
        self.send_data_frame[14] = (motor_id >> 8) & 0xFF
        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame.T))

    def _extract_packets(self, data: bytes) -> list[bytes]:
        frames = []
        header = 0xAA
        tail = 0x55
        frame_length = 16
        i = 0
        remainder_pos = 0

        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frames.append(data[i : i + frame_length])
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        self.data_save = data[remainder_pos:]
        return frames
