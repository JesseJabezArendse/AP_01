#!/usr/bin/env python3
"""
VCP Data Parser and Plotter
Parses data from STM32 Nucleo board over Virtual COM Port and plots all sensor data.

Supports all AP_01 implementations:
- IKS02A1 Source Code Full (IMU only)
- IKS02A1 Source Code LP Accel (Low Power Accelerometer)
- IKS02A1 VL53L1A1 Source Code Both (IMU + ToF)
- IKS02A1 VL53L8A1 Source Code Both (IMU + ToF 8x8)
- VL53L1A1 Source Code Full (ToF only)
- VL53L8A1 Source Code Full (ToF 8x8 only)

Author: Jesse Jabez Arendse
Date: 2025
"""

import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Dict, List
import argparse
import sys

import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec


@dataclass
class DataPacket:
    """Base class for data packets"""
    timestamp: float
    counter: int


@dataclass
class IKS02A1Packet(DataPacket):
    """IKS02A1 (IMU only) data packet"""
    accel1_x: float
    accel1_y: float
    accel1_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    accel2_x: float
    accel2_y: float
    accel2_z: float
    temperature: float
    mag_x: float
    mag_y: float
    mag_z: float
    fastest_odr: float


@dataclass
class VL53L1A1Packet(DataPacket):
    """VL53L1A1 (ToF single pixel) data packet"""
    tof_left_distance: float
    tof_left_ambient: float
    tof_left_signal: float
    tof_left_status: int
    tof_centre_distance: float
    tof_centre_ambient: float
    tof_centre_signal: float
    tof_centre_status: int
    tof_right_distance: float
    tof_right_ambient: float
    tof_right_signal: float
    tof_right_status: int


@dataclass
class VL53L8A1Packet(DataPacket):
    """VL53L8A1 (ToF 8x8 array) data packet"""
    tof_distance_matrix: np.ndarray  # 8x8 array


@dataclass
class CombinedPacket(DataPacket):
    """Combined IKS02A1 + VL53L1A1 data packet"""
    # ToF data
    tof_left_distance: float
    tof_left_ambient: float
    tof_left_signal: float
    tof_left_status: int
    tof_centre_distance: float
    tof_centre_ambient: float
    tof_centre_signal: float
    tof_centre_status: int
    tof_right_distance: float
    tof_right_ambient: float
    tof_right_signal: float
    tof_right_status: int
    # IMU data
    accel1_x: float
    accel1_y: float
    accel1_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    accel2_x: float
    accel2_y: float
    accel2_z: float
    temperature: float
    mag_x: float
    mag_y: float
    mag_z: float
    fastest_odr: float


class VCPDataParser:
    """Parses binary data from VCP"""

    HEADER = b'A_J'
    TERMINATOR = b'J_A'

    def __init__(self):
        self.buffer = bytearray()

    def bytes_to_float(self, b1: int, b2: int, b3: int, b4: int) -> float:
        """Convert 4 bytes to float"""
        return struct.unpack('<f', bytes([b1, b2, b3, b4]))[0]

    def bytes_to_int32(self, b1: int, b2: int, b3: int, b4: int) -> int:
        """Convert 4 bytes to int32"""
        return struct.unpack('<i', bytes([b1, b2, b3, b4]))[0]

    def bytes_to_uint32(self, b1: int, b2: int, b3: int, b4: int) -> int:
        """Convert 4 bytes to uint32"""
        return struct.unpack('<I', bytes([b1, b2, b3, b4]))[0]

    def add_data(self, data: bytes) -> List[DataPacket]:
        """Add serial data to buffer and extract complete packets"""
        self.buffer.extend(data)
        packets = []

        while len(self.buffer) >= 42:  # Minimum packet size
            # Find header
            header_idx = self.buffer.find(self.HEADER)
            if header_idx == -1:
                break

            if header_idx > 0:
                self.buffer = self.buffer[header_idx:]

            # Try to find terminator
            terminator_idx = self.buffer.find(self.TERMINATOR, 3)
            if terminator_idx == -1:
                break

            packet_data = bytes(self.buffer[:terminator_idx + 3])
            self.buffer = self.buffer[terminator_idx + 3:]

            try:
                packet = self._parse_packet(packet_data)
                if packet:
                    packets.append(packet)
            except Exception as e:
                print(f"Error parsing packet: {e}")

        return packets

    def _parse_packet(self, data: bytes) -> Optional[DataPacket]:
        """Parse a single packet and determine its type"""
        if len(data) < 9:  # Minimum: header(3) + terminator(3) + data(3)
            return None

        payload_size = len(data) - 6  # Subtract header and terminator

        # Detect packet type based on payload size
        if payload_size == 36:  # IKS02A1 only or VL53L1A1 only
            return self._parse_iks02a1_or_tof(data)
        elif payload_size == 76:  # Combined IKS02A1 + VL53L1A1
            return self._parse_combined(data)
        elif payload_size == 288:  # VL53L8A1 (64 floats for 8x8 matrix)
            return self._parse_vl53l8a1(data)
        else:
            print(f"Unknown packet size: {payload_size}")
            return None

    def _parse_iks02a1_or_tof(self, data: bytes) -> Optional[DataPacket]:
        """Parse IKS02A1 or VL53L1A1 only packet (36 bytes payload)"""
        payload = data[3:-3]

        # Check for ToF data pattern (3 sensors * 4 values * 4 bytes = 48 bytes)
        # or IMU data pattern (11 values * 4 bytes = 44 bytes)

        if len(payload) < 36:
            return None

        # Try to determine which type by checking if it looks like IMU data
        # IMU has temperature as float around room temp (20-40), and mag/accel/gyro as large integers
        try:
            # Assume IMU first
            accel1_x = self.bytes_to_int32(payload[0], payload[1], payload[2], payload[3])
            accel1_y = self.bytes_to_int32(payload[4], payload[5], payload[6], payload[7])
            accel1_z = self.bytes_to_int32(payload[8], payload[9], payload[10], payload[11])
            gyro_x = self.bytes_to_int32(payload[12], payload[13], payload[14], payload[15])
            gyro_y = self.bytes_to_int32(payload[16], payload[17], payload[18], payload[19])
            gyro_z = self.bytes_to_int32(payload[20], payload[21], payload[22], payload[23])
            accel2_x = self.bytes_to_int32(payload[24], payload[25], payload[26], payload[27])
            accel2_y = self.bytes_to_int32(payload[28], payload[29], payload[30], payload[31])
            temperature = self.bytes_to_float(payload[32], payload[33], payload[34], payload[35])

            # Check if temperature is reasonable (should be between -40 and 85°C)
            if -40 <= temperature <= 85:
                # This looks like partial IMU data (without mag and fastestODR)
                counter = 0
                return IKS02A1Packet(
                    timestamp=time.time(),
                    counter=counter,
                    accel1_x=accel1_x,
                    accel1_y=accel1_y,
                    accel1_z=accel1_z,
                    gyro_x=gyro_x,
                    gyro_y=gyro_y,
                    gyro_z=gyro_z,
                    accel2_x=accel2_x,
                    accel2_y=accel2_y,
                    accel2_z=accel2_z,
                    temperature=temperature,
                    mag_x=0,
                    mag_y=0,
                    mag_z=0,
                    fastest_odr=0
                )
        except Exception:
            pass

        # Try ToF data
        try:
            tof_left_distance = self.bytes_to_uint32(payload[0], payload[1], payload[2], payload[3])
            tof_left_ambient = self.bytes_to_uint32(payload[4], payload[5], payload[6], payload[7])
            tof_left_signal = self.bytes_to_uint32(payload[8], payload[9], payload[10], payload[11])
            tof_left_status = self.bytes_to_uint32(payload[12], payload[13], payload[14], payload[15])
            tof_centre_distance = self.bytes_to_uint32(payload[16], payload[17], payload[18], payload[19])
            tof_centre_ambient = self.bytes_to_uint32(payload[20], payload[21], payload[22], payload[23])
            tof_centre_signal = self.bytes_to_uint32(payload[24], payload[25], payload[26], payload[27])
            tof_centre_status = self.bytes_to_uint32(payload[28], payload[29], payload[30], payload[31])
            counter = self.bytes_to_uint32(payload[32], payload[33], payload[34], payload[35])

            return VL53L1A1Packet(
                timestamp=time.time(),
                counter=counter,
                tof_left_distance=tof_left_distance,
                tof_left_ambient=tof_left_ambient,
                tof_left_signal=tof_left_signal,
                tof_left_status=tof_left_status,
                tof_centre_distance=tof_centre_distance,
                tof_centre_ambient=tof_centre_ambient,
                tof_centre_signal=tof_centre_signal,
                tof_centre_status=tof_centre_status,
                tof_right_distance=0,
                tof_right_ambient=0,
                tof_right_signal=0,
                tof_right_status=0
            )
        except Exception:
            pass

        return None

    def _parse_combined(self, data: bytes) -> Optional[CombinedPacket]:
        """Parse combined IKS02A1 + VL53L1A1 packet (76 bytes payload)"""
        payload = data[3:-3]

        if len(payload) != 76:
            return None

        try:
            offset = 0

            # ToF data (48 bytes)
            tof_left_distance = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            tof_left_ambient = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            tof_left_signal = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            tof_left_status = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4

            tof_centre_distance = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            tof_centre_ambient = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            tof_centre_signal = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            tof_centre_status = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4

            tof_right_distance = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            tof_right_ambient = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            tof_right_signal = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            tof_right_status = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4

            # IMU data (28 bytes)
            accel1_x = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            accel1_y = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            accel1_z = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            gyro_x = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            gyro_y = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            gyro_z = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            accel2_x = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            accel2_y = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            accel2_z = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            temperature = self.bytes_to_float(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            mag_x = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            mag_y = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            mag_z = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4

            counter = self.bytes_to_int32(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )
            offset += 4
            fastest_odr = self.bytes_to_float(
                payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
            )

            return CombinedPacket(
                timestamp=time.time(),
                counter=counter,
                tof_left_distance=tof_left_distance,
                tof_left_ambient=tof_left_ambient,
                tof_left_signal=tof_left_signal,
                tof_left_status=tof_left_status,
                tof_centre_distance=tof_centre_distance,
                tof_centre_ambient=tof_centre_ambient,
                tof_centre_signal=tof_centre_signal,
                tof_centre_status=tof_centre_status,
                tof_right_distance=tof_right_distance,
                tof_right_ambient=tof_right_ambient,
                tof_right_signal=tof_right_signal,
                tof_right_status=tof_right_status,
                accel1_x=accel1_x,
                accel1_y=accel1_y,
                accel1_z=accel1_z,
                gyro_x=gyro_x,
                gyro_y=gyro_y,
                gyro_z=gyro_z,
                accel2_x=accel2_x,
                accel2_y=accel2_y,
                accel2_z=accel2_z,
                temperature=temperature,
                mag_x=mag_x,
                mag_y=mag_y,
                mag_z=mag_z,
                fastest_odr=fastest_odr
            )
        except Exception as e:
            print(f"Error parsing combined packet: {e}")
            return None

    def _parse_vl53l8a1(self, data: bytes) -> Optional[VL53L8A1Packet]:
        """Parse VL53L8A1 8x8 ToF array packet"""
        payload = data[3:-3]

        if len(payload) != 288:  # 64 floats * 4 bytes
            return None

        try:
            distance_matrix = np.zeros((8, 8), dtype=np.float32)

            for i in range(64):
                offset = i * 4
                value = self.bytes_to_float(
                    payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
                )
                distance_matrix[i // 8, i % 8] = value

            return VL53L8A1Packet(
                timestamp=time.time(),
                counter=0,
                tof_distance_matrix=distance_matrix
            )
        except Exception as e:
            print(f"Error parsing VL53L8A1 packet: {e}")
            return None


class DataPlotter:
    """Real-time plotter for sensor data"""

    def __init__(self, max_points: int = 500):
        self.max_points = max_points
        self.packets: deque = deque(maxlen=max_points)
        self.packet_type = None

        # Setup figure with multiple subplots
        self.fig = plt.figure(figsize=(16, 12))
        self.fig.suptitle('AP_01 Sensor Data Monitor', fontsize=16, fontweight='bold')

        self.setup_plots()

    def setup_plots(self):
        """Setup the plot layout"""
        gs = GridSpec(4, 3, figure=self.fig, hspace=0.35, wspace=0.3)

        # Accelerometer plots
        self.ax_accel1 = self.fig.add_subplot(gs[0, 0])
        self.ax_accel2 = self.fig.add_subplot(gs[0, 1])

        # Gyroscope plot
        self.ax_gyro = self.fig.add_subplot(gs[0, 2])

        # Temperature plot
        self.ax_temp = self.fig.add_subplot(gs[1, 0])

        # Magnetometer plot
        self.ax_mag = self.fig.add_subplot(gs[1, 1])

        # ToF distance plots
        self.ax_tof_dist = self.fig.add_subplot(gs[1, 2])

        # ToF ambient/signal plots
        self.ax_tof_ambient = self.fig.add_subplot(gs[2, 0])
        self.ax_tof_signal = self.fig.add_subplot(gs[2, 1])

        # Counter plot
        self.ax_counter = self.fig.add_subplot(gs[2, 2])

        # Status plot
        self.ax_status = self.fig.add_subplot(gs[3, 0])

        # ToF 8x8 heatmap (if applicable)
        self.ax_tof_heatmap = self.fig.add_subplot(gs[3, 1:])

        # Initialize line objects
        self.line_accel1_x, = self.ax_accel1.plot([], [], label='Accel1 X', color='r')
        self.line_accel1_y, = self.ax_accel1.plot([], [], label='Accel1 Y', color='g')
        self.line_accel1_z, = self.ax_accel1.plot([], [], label='Accel1 Z', color='b')
        self.ax_accel1.set_ylabel('Accel1 (mg)')
        self.ax_accel1.legend(loc='upper right', fontsize=8)
        self.ax_accel1.grid(True)

        self.line_accel2_x, = self.ax_accel2.plot([], [], label='Accel2 X', color='r')
        self.line_accel2_y, = self.ax_accel2.plot([], [], label='Accel2 Y', color='g')
        self.line_accel2_z, = self.ax_accel2.plot([], [], label='Accel2 Z', color='b')
        self.ax_accel2.set_ylabel('Accel2 (mg)')
        self.ax_accel2.legend(loc='upper right', fontsize=8)
        self.ax_accel2.grid(True)

        self.line_gyro_x, = self.ax_gyro.plot([], [], label='Gyro X', color='r')
        self.line_gyro_y, = self.ax_gyro.plot([], [], label='Gyro Y', color='g')
        self.line_gyro_z, = self.ax_gyro.plot([], [], label='Gyro Z', color='b')
        self.ax_gyro.set_ylabel('Gyro (dps)')
        self.ax_gyro.legend(loc='upper right', fontsize=8)
        self.ax_gyro.grid(True)

        self.line_temp, = self.ax_temp.plot([], [], label='Temperature', color='orange')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.legend(loc='upper right', fontsize=8)
        self.ax_temp.grid(True)

        self.line_mag_x, = self.ax_mag.plot([], [], label='Mag X', color='r')
        self.line_mag_y, = self.ax_mag.plot([], [], label='Mag Y', color='g')
        self.line_mag_z, = self.ax_mag.plot([], [], label='Mag Z', color='b')
        self.ax_mag.set_ylabel('Magnetometer (mGauss)')
        self.ax_mag.legend(loc='upper right', fontsize=8)
        self.ax_mag.grid(True)

        self.line_tof_left, = self.ax_tof_dist.plot([], [], label='ToF Left', color='r', marker='o')
        self.line_tof_center, = self.ax_tof_dist.plot([], [], label='ToF Center', color='g', marker='s')
        self.line_tof_right, = self.ax_tof_dist.plot([], [], label='ToF Right', color='b', marker='^')
        self.ax_tof_dist.set_ylabel('Distance (mm)')
        self.ax_tof_dist.legend(loc='upper right', fontsize=8)
        self.ax_tof_dist.grid(True)

        self.line_ambient_left, = self.ax_tof_ambient.plot([], [], label='Left', color='r')
        self.line_ambient_center, = self.ax_tof_ambient.plot([], [], label='Center', color='g')
        self.line_ambient_right, = self.ax_tof_ambient.plot([], [], label='Right', color='b')
        self.ax_tof_ambient.set_ylabel('Ambient Rate (kcps)')
        self.ax_tof_ambient.legend(loc='upper right', fontsize=8)
        self.ax_tof_ambient.grid(True)

        self.line_signal_left, = self.ax_tof_signal.plot([], [], label='Left', color='r')
        self.line_signal_center, = self.ax_tof_signal.plot([], [], label='Center', color='g')
        self.line_signal_right, = self.ax_tof_signal.plot([], [], label='Right', color='b')
        self.ax_tof_signal.set_ylabel('Signal Rate (kcps)')
        self.ax_tof_signal.legend(loc='upper right', fontsize=8)
        self.ax_tof_signal.grid(True)

        self.line_counter, = self.ax_counter.plot([], [], label='Packet Counter', color='purple')
        self.ax_counter.set_ylabel('Count')
        self.ax_counter.legend(loc='upper right', fontsize=8)
        self.ax_counter.grid(True)

        self.line_status_left, = self.ax_status.plot([], [], label='Left Status', color='r', marker='x')
        self.line_status_center, = self.ax_status.plot([], [], label='Center Status', color='g', marker='x')
        self.line_status_right, = self.ax_status.plot([], [], label='Right Status', color='b', marker='x')
        self.ax_status.set_ylabel('Range Status')
        self.ax_status.set_ylim(-1, 8)
        self.ax_status.legend(loc='upper right', fontsize=8)
        self.ax_status.grid(True)

        self.tof_heatmap = None

    def add_packet(self, packet: DataPacket):
        """Add a new packet and update plots"""
        self.packets.append(packet)
        if self.packet_type is None:
            self.packet_type = type(packet)

    def update(self, frame):
        """Update plot data"""
        if not self.packets:
            return []

        # Extract data from all packets
        indices = np.arange(len(self.packets))

        artists = []

        # Determine packet type and extract relevant data
        for i, packet in enumerate(self.packets):
            if isinstance(packet, (IKS02A1Packet, CombinedPacket)):
                # Update accelerometer 1
                if i == len(self.packets) - 1:  # Only update on last packet
                    pass

        # Create data arrays
        if isinstance(self.packets[-1], IKS02A1Packet):
            accel1_x_data = np.array([p.accel1_x for p in self.packets])
            accel1_y_data = np.array([p.accel1_y for p in self.packets])
            accel1_z_data = np.array([p.accel1_z for p in self.packets])
            gyro_x_data = np.array([p.gyro_x for p in self.packets])
            gyro_y_data = np.array([p.gyro_y for p in self.packets])
            gyro_z_data = np.array([p.gyro_z for p in self.packets])
            accel2_x_data = np.array([p.accel2_x for p in self.packets])
            accel2_y_data = np.array([p.accel2_y for p in self.packets])
            accel2_z_data = np.array([p.accel2_z for p in self.packets])
            temp_data = np.array([p.temperature for p in self.packets])
            mag_x_data = np.array([p.mag_x for p in self.packets])
            mag_y_data = np.array([p.mag_y for p in self.packets])
            mag_z_data = np.array([p.mag_z for p in self.packets])
            counter_data = np.array([p.counter for p in self.packets])

            # Update lines
            self.line_accel1_x.set_data(indices, accel1_x_data)
            self.line_accel1_y.set_data(indices, accel1_y_data)
            self.line_accel1_z.set_data(indices, accel1_z_data)
            artists.extend([self.line_accel1_x, self.line_accel1_y, self.line_accel1_z])

            self.line_accel2_x.set_data(indices, accel2_x_data)
            self.line_accel2_y.set_data(indices, accel2_y_data)
            self.line_accel2_z.set_data(indices, accel2_z_data)
            artists.extend([self.line_accel2_x, self.line_accel2_y, self.line_accel2_z])

            self.line_gyro_x.set_data(indices, gyro_x_data)
            self.line_gyro_y.set_data(indices, gyro_y_data)
            self.line_gyro_z.set_data(indices, gyro_z_data)
            artists.extend([self.line_gyro_x, self.line_gyro_y, self.line_gyro_z])

            self.line_temp.set_data(indices, temp_data)
            artists.append(self.line_temp)

            self.line_mag_x.set_data(indices, mag_x_data)
            self.line_mag_y.set_data(indices, mag_y_data)
            self.line_mag_z.set_data(indices, mag_z_data)
            artists.extend([self.line_mag_x, self.line_mag_y, self.line_mag_z])

            self.line_counter.set_data(indices, counter_data)
            artists.append(self.line_counter)

            # Rescale axes
            self._rescale_axes(
                [self.ax_accel1, self.ax_accel2, self.ax_gyro, self.ax_temp, self.ax_mag, self.ax_counter],
                indices
            )

        if isinstance(self.packets[-1], (VL53L1A1Packet, CombinedPacket)):
            tof_left_dist_data = np.array([p.tof_left_distance for p in self.packets])
            tof_center_dist_data = np.array([p.tof_centre_distance for p in self.packets])
            tof_right_dist_data = np.array([p.tof_right_distance for p in self.packets])

            tof_left_ambient_data = np.array([p.tof_left_ambient for p in self.packets])
            tof_center_ambient_data = np.array([p.tof_centre_ambient for p in self.packets])
            tof_right_ambient_data = np.array([p.tof_right_ambient for p in self.packets])

            tof_left_signal_data = np.array([p.tof_left_signal for p in self.packets])
            tof_center_signal_data = np.array([p.tof_centre_signal for p in self.packets])
            tof_right_signal_data = np.array([p.tof_right_signal for p in self.packets])

            tof_left_status_data = np.array([p.tof_left_status for p in self.packets])
            tof_center_status_data = np.array([p.tof_centre_status for p in self.packets])
            tof_right_status_data = np.array([p.tof_right_status for p in self.packets])

            # Update lines
            self.line_tof_left.set_data(indices, tof_left_dist_data)
            self.line_tof_center.set_data(indices, tof_center_dist_data)
            self.line_tof_right.set_data(indices, tof_right_dist_data)
            artists.extend([self.line_tof_left, self.line_tof_center, self.line_tof_right])

            self.line_ambient_left.set_data(indices, tof_left_ambient_data)
            self.line_ambient_center.set_data(indices, tof_center_ambient_data)
            self.line_ambient_right.set_data(indices, tof_right_ambient_data)
            artists.extend([self.line_ambient_left, self.line_ambient_center, self.line_ambient_right])

            self.line_signal_left.set_data(indices, tof_left_signal_data)
            self.line_signal_center.set_data(indices, tof_center_signal_data)
            self.line_signal_right.set_data(indices, tof_right_signal_data)
            artists.extend([self.line_signal_left, self.line_signal_center, self.line_signal_right])

            self.line_status_left.set_data(indices, tof_left_status_data)
            self.line_status_center.set_data(indices, tof_center_status_data)
            self.line_status_right.set_data(indices, tof_right_status_data)
            artists.extend([self.line_status_left, self.line_status_center, self.line_status_right])

            # Rescale axes
            self._rescale_axes(
                [self.ax_tof_dist, self.ax_tof_ambient, self.ax_tof_signal, self.ax_status],
                indices
            )

        if isinstance(self.packets[-1], VL53L8A1Packet):
            # Update heatmap for 8x8 ToF
            latest_packet = self.packets[-1]
            if self.tof_heatmap is None:
                self.tof_heatmap = self.ax_tof_heatmap.imshow(
                    latest_packet.tof_distance_matrix, 
                    cmap='hot', 
                    aspect='auto',
                    interpolation='nearest'
                )
                plt.colorbar(self.tof_heatmap, ax=self.ax_tof_heatmap, label='Distance (mm)')
                self.ax_tof_heatmap.set_title('VL53L8A1 8x8 Distance Matrix')
            else:
                self.tof_heatmap.set_data(latest_packet.tof_distance_matrix)
                self.tof_heatmap.set_clim(
                    vmin=latest_packet.tof_distance_matrix.min(),
                    vmax=latest_packet.tof_distance_matrix.max()
                )
            artists.append(self.tof_heatmap)

        return artists

    def _rescale_axes(self, axes_list, indices):
        """Rescale y-axis for a list of axes"""
        for ax in axes_list:
            ax.relim()
            ax.autoscale_view()
            if len(indices) > 0:
                ax.set_xlim(indices[0], indices[-1])


class VCPMonitor:
    """Main class for monitoring VCP data"""

    def __init__(self, port: str, baudrate: int = 1843200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.parser = VCPDataParser()
        self.plotter = DataPlotter()
        self.running = False
        self.read_thread = None

    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False

    def read_data(self):
        """Read data from serial port in background thread"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    packets = self.parser.add_data(data)
                    for packet in packets:
                        self.plotter.add_packet(packet)
            except Exception as e:
                print(f"Error reading data: {e}")
            time.sleep(0.01)

    def start(self):
        """Start monitoring"""
        if not self.connect():
            return False

        self.running = True
        self.read_thread = threading.Thread(target=self.read_data, daemon=True)
        self.read_thread.start()

        # Setup animation
        ani = FuncAnimation(
            self.plotter.fig,
            self.plotter.update,
            interval=100,
            blit=True,
            cache_frame_data=False
        )

        plt.show()
        return True

    def stop(self):
        """Stop monitoring"""
        self.running = False
        if self.read_thread:
            self.read_thread.join()
        if self.ser:
            self.ser.close()


def list_ports():
    """List available COM ports"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No COM ports found")
        return []

    print("Available COM ports:")
    for i, port in enumerate(ports):
        print(f"  {i}: {port.device} - {port.description}")
    return ports


def main():
    parser = argparse.ArgumentParser(
        description='VCP Data Parser and Plotter for AP_01 Sensor Board'
    )
    parser.add_argument(
        '--port',
        type=str,
        help='Serial port (e.g., COM3, /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=1843200,
        help='Baud rate (default: 1843200)'
    )
    parser.add_argument(
        '--list-ports',
        action='store_true',
        help='List available COM ports and exit'
    )

    args = parser.parse_args()

    if args.list_ports:
        list_ports()
        return

    if not args.port:
        print("Error: --port is required. Use --list-ports to see available ports.")
        sys.exit(1)

    monitor = VCPMonitor(args.port, args.baud)
    try:
        monitor.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        monitor.stop()


if __name__ == '__main__':
    main()
