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
    """Low-level STS Standard TTL protocol handler."""

    def __init__(self, serial_device: Serial):
        self.serial_ = serial_device
        self.positions: Dict[int, int] = {}

        if self.serial_.is_open:
            # We assume the serial port is already configured correctly by the caller
            pass

    def _calculate_checksum(self, data: list[int]) -> int:
        return (~sum(data)) & 0xFF

    def write_position(self, controller_id: int, motor_id: int, angle: int) -> None:
        # Standard SCS Protocol Write Position
        
        # Packet: FF FF ID LEN INSTR P1...PN CHECKSUM
        
        data_buf = [
            0x2A,             # Address 42 (Goal Position)
            angle & 0xFF,     # Position Low
            (angle >> 8) & 0xFF # Position High
        ]
        
        length = len(data_buf) + 2 # Instruction + Checksum
        packet = [0xFF, 0xFF, motor_id, length, 0x03] + data_buf
        checksum = self._calculate_checksum(packet[2:])
        packet.append(checksum)
        
        self.serial_.write(bytes(packet))

    def read_position(self, controller_id: int, motor_id: int, timeout: float = 0.1) -> int | None:
        # Read Position from Address 56 (Present Position), Length 2
        val = self.read_register(motor_id, 56, 2, timeout)
        return val

    def write_register(self, motor_id: int, address: int, data: int | list[int]) -> None:
        if isinstance(data, int):
            if data <= 255:
                 data = [data]
            else:
                 data = [data & 0xFF, (data >> 8) & 0xFF]
        
        data_buf = [address] + data
        length = len(data_buf) + 2
        packet = [0xFF, 0xFF, motor_id, length, 0x03] + data_buf
        checksum = self._calculate_checksum(packet[2:])
        packet.append(checksum)
        
        self.serial_.write(bytes(packet))

    def read_register(self, motor_id: int, address: int, length: int, timeout: float = 0.1) -> int | None:
        # Read Instruction: 0x02
        data_buf = [address, length]
        packet_len = len(data_buf) + 2
        packet = [0xFF, 0xFF, motor_id, packet_len, 0x02] + data_buf
        checksum = self._calculate_checksum(packet[2:])
        packet.append(checksum)
        
        self.serial_.reset_input_buffer()
        self.serial_.write(bytes(packet))
        
        start_time = time.time()
        # Response: FF FF ID LEN ERROR P1...PN CHECKSUM
        expected_len = 6 + length # 2(Header)+1(ID)+1(Len)+1(Err)+N(Data)+1(Sum)
        
        while time.time() - start_time < timeout:
             if self.serial_.in_waiting >= expected_len:
                 data = self.serial_.read(expected_len)
                 if data[0] == 0xFF and data[1] == 0xFF and data[2] == motor_id:
                     # Checksum verification
                     if self._calculate_checksum(list(data[2:-1])) == data[-1]:
                         if length == 1:
                             return data[5]
                         elif length == 2:
                             return (data[6] << 8) | data[5]
                         else:
                             return list(data[5:5+length])
                 return None
             time.sleep(0.001)
        return None

    def recv(self) -> None:
        pass
