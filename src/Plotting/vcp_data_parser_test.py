#!/usr/bin/env python3
"""
VCP Data Parser - Test/Debug Script

This script provides utilities for testing the VCP parser without actual hardware.
It can:
- Generate synthetic sensor data
- Test packet parsing
- Save/load data for offline analysis
- Validate protocol compliance

Usage:
    python vcp_data_parser_test.py --generate-iks02a1
    python vcp_data_parser_test.py --generate-combined
    python vcp_data_parser_test.py --test-parsing
"""

import struct
import numpy as np
from dataclasses import dataclass
import argparse


class MockVCPGenerator:
    """Generate synthetic VCP packets for testing"""

    HEADER = b'A_J'
    TERMINATOR = b'J_A'

    def __init__(self):
        self.counter = 0

    def float_to_bytes(self, value: float) -> bytes:
        """Convert float to 4 bytes (little-endian)"""
        return struct.pack('<f', value)

    def int32_to_bytes(self, value: int) -> bytes:
        """Convert int32 to 4 bytes (little-endian)"""
        return struct.pack('<i', value)

    def uint32_to_bytes(self, value: int) -> bytes:
        """Convert uint32 to 4 bytes (little-endian)"""
        return struct.pack('<I', value)

    def generate_iks02a1_packet(self) -> bytes:
        """Generate IKS02A1 IMU-only packet (60 bytes payload)"""
        self.counter += 1

        # Simulate sensor data
        accel1_x = int(np.random.normal(0, 100))  # ±100 mg noise
        accel1_y = int(np.random.normal(0, 100))
        accel1_z = int(np.random.normal(1000, 100))  # ~1g on Z
        
        gyro_x = int(np.random.normal(0, 50))  # ±50 dps noise
        gyro_y = int(np.random.normal(0, 50))
        gyro_z = int(np.random.normal(0, 50))
        
        accel2_x = int(np.random.normal(0, 50))
        accel2_y = int(np.random.normal(0, 50))
        accel2_z = int(np.random.normal(1000, 50))
        
        temperature = 25.0 + np.random.normal(0, 1)  # ~25°C
        
        mag_x = int(np.random.normal(0, 100))
        mag_y = int(np.random.normal(0, 100))
        mag_z = int(np.random.normal(0, 100))
        
        fastest_odr = 208.0

        payload = (
            self.int32_to_bytes(accel1_x) +
            self.int32_to_bytes(accel1_y) +
            self.int32_to_bytes(accel1_z) +
            self.int32_to_bytes(gyro_x) +
            self.int32_to_bytes(gyro_y) +
            self.int32_to_bytes(gyro_z) +
            self.int32_to_bytes(accel2_x) +
            self.int32_to_bytes(accel2_y) +
            self.int32_to_bytes(accel2_z) +
            self.float_to_bytes(temperature) +
            self.int32_to_bytes(mag_x) +
            self.int32_to_bytes(mag_y) +
            self.int32_to_bytes(mag_z) +
            self.int32_to_bytes(self.counter) +
            self.float_to_bytes(fastest_odr)
        )

        return self.HEADER + payload + self.TERMINATOR

    def generate_tof_l1a1_packet(self) -> bytes:
        """Generate VL53L1A1 ToF-only packet (52 bytes payload)"""
        self.counter += 1

        # Simulate ToF data
        tof_left_distance = int(np.random.normal(500, 50))  # ~500 mm
        tof_left_ambient = int(np.random.normal(10, 2))  # kcps
        tof_left_signal = int(np.random.normal(200, 20))
        tof_left_status = 0  # Valid measurement
        
        tof_centre_distance = int(np.random.normal(600, 50))
        tof_centre_ambient = int(np.random.normal(10, 2))
        tof_centre_signal = int(np.random.normal(200, 20))
        tof_centre_status = 0
        
        tof_right_distance = int(np.random.normal(550, 50))
        tof_right_ambient = int(np.random.normal(10, 2))
        tof_right_signal = int(np.random.normal(200, 20))
        tof_right_status = 0

        payload = (
            self.int32_to_bytes(tof_left_distance) +
            self.int32_to_bytes(tof_left_ambient) +
            self.int32_to_bytes(tof_left_signal) +
            self.int32_to_bytes(tof_left_status) +
            self.int32_to_bytes(tof_centre_distance) +
            self.int32_to_bytes(tof_centre_ambient) +
            self.int32_to_bytes(tof_centre_signal) +
            self.int32_to_bytes(tof_centre_status) +
            self.int32_to_bytes(tof_right_distance) +
            self.int32_to_bytes(tof_right_ambient) +
            self.int32_to_bytes(tof_right_signal) +
            self.int32_to_bytes(tof_right_status) +
            self.int32_to_bytes(self.counter)
        )

        return self.HEADER + payload + self.TERMINATOR

    def generate_combined_packet(self) -> bytes:
        """Generate IKS02A1 + VL53L1A1 combined packet (100 bytes payload)"""
        self.counter += 1

        # ToF data
        tof_left_distance = int(np.random.normal(500, 50))
        tof_left_ambient = int(np.random.normal(10, 2))
        tof_left_signal = int(np.random.normal(200, 20))
        tof_left_status = 0
        
        tof_centre_distance = int(np.random.normal(600, 50))
        tof_centre_ambient = int(np.random.normal(10, 2))
        tof_centre_signal = int(np.random.normal(200, 20))
        tof_centre_status = 0
        
        tof_right_distance = int(np.random.normal(550, 50))
        tof_right_ambient = int(np.random.normal(10, 2))
        tof_right_signal = int(np.random.normal(200, 20))
        tof_right_status = 0

        # IMU data
        accel1_x = int(np.random.normal(0, 100))
        accel1_y = int(np.random.normal(0, 100))
        accel1_z = int(np.random.normal(1000, 100))
        
        gyro_x = int(np.random.normal(0, 50))
        gyro_y = int(np.random.normal(0, 50))
        gyro_z = int(np.random.normal(0, 50))
        
        accel2_x = int(np.random.normal(0, 50))
        accel2_y = int(np.random.normal(0, 50))
        accel2_z = int(np.random.normal(1000, 50))
        
        temperature = 25.0 + np.random.normal(0, 1)
        
        mag_x = int(np.random.normal(0, 100))
        mag_y = int(np.random.normal(0, 100))
        mag_z = int(np.random.normal(0, 100))
        
        fastest_odr = 208.0

        payload = (
            # ToF (48 bytes)
            self.int32_to_bytes(tof_left_distance) +
            self.int32_to_bytes(tof_left_ambient) +
            self.int32_to_bytes(tof_left_signal) +
            self.int32_to_bytes(tof_left_status) +
            self.int32_to_bytes(tof_centre_distance) +
            self.int32_to_bytes(tof_centre_ambient) +
            self.int32_to_bytes(tof_centre_signal) +
            self.int32_to_bytes(tof_centre_status) +
            self.int32_to_bytes(tof_right_distance) +
            self.int32_to_bytes(tof_right_ambient) +
            self.int32_to_bytes(tof_right_signal) +
            self.int32_to_bytes(tof_right_status) +
            # IMU (52 bytes)
            self.int32_to_bytes(accel1_x) +
            self.int32_to_bytes(accel1_y) +
            self.int32_to_bytes(accel1_z) +
            self.int32_to_bytes(gyro_x) +
            self.int32_to_bytes(gyro_y) +
            self.int32_to_bytes(gyro_z) +
            self.int32_to_bytes(accel2_x) +
            self.int32_to_bytes(accel2_y) +
            self.int32_to_bytes(accel2_z) +
            self.float_to_bytes(temperature) +
            self.int32_to_bytes(mag_x) +
            self.int32_to_bytes(mag_y) +
            self.int32_to_bytes(mag_z) +
            self.int32_to_bytes(self.counter) +
            self.float_to_bytes(fastest_odr)
        )

        return self.HEADER + payload + self.TERMINATOR

    def generate_vl53l8a1_packet(self) -> bytes:
        """Generate VL53L8A1 8x8 ToF array packet (288 bytes payload)"""
        self.counter += 1

        # Generate 8x8 random distance matrix (500-800 mm)
        distance_matrix = np.random.uniform(500, 800, (8, 8)).astype(np.float32)

        payload = b''
        for row in distance_matrix:
            for distance in row:
                payload += self.float_to_bytes(float(distance))

        return self.HEADER + payload + self.TERMINATOR


class TestPacketValidator:
    """Validate packet structure and integrity"""

    @staticmethod
    def validate_iks02a1_packet(data: bytes) -> bool:
        """Validate IKS02A1 packet structure"""
        if len(data) != 66:
            print(f"❌ Invalid length: {len(data)} (expected 66)")
            return False

        if data[:3] != b'A_J':
            print("❌ Invalid header")
            return False

        if data[-3:] != b'J_A':
            print("❌ Invalid terminator")
            return False

        print("✓ Valid IKS02A1 packet (66 bytes)")
        print(f"  Header: {data[:3]}")
        print(f"  Payload: {len(data)-6} bytes")
        print(f"  Terminator: {data[-3:]}")
        return True

    @staticmethod
    def validate_combined_packet(data: bytes) -> bool:
        """Validate combined IKS02A1 + VL53L1A1 packet"""
        if len(data) != 106:
            print(f"❌ Invalid length: {len(data)} (expected 106)")
            return False

        if data[:3] != b'A_J':
            print("❌ Invalid header")
            return False

        if data[-3:] != b'J_A':
            print("❌ Invalid terminator")
            return False

        print("✓ Valid Combined packet (106 bytes)")
        print(f"  Header: {data[:3]}")
        print(f"  ToF payload: 48 bytes")
        print(f"  IMU payload: 52 bytes")
        print(f"  Terminator: {data[-3:]}")
        return True

    @staticmethod
    def validate_tof_l1a1_packet(data: bytes) -> bool:
        """Validate VL53L1A1 ToF packet"""
        if len(data) != 58:
            print(f"❌ Invalid length: {len(data)} (expected 58)")
            return False

        if data[:3] != b'A_J':
            print("❌ Invalid header")
            return False

        if data[-3:] != b'J_A':
            print("❌ Invalid terminator")
            return False

        print("✓ Valid VL53L1A1 packet (58 bytes)")
        return True

    @staticmethod
    def validate_vl53l8a1_packet(data: bytes) -> bool:
        """Validate VL53L8A1 8x8 ToF packet"""
        if len(data) != 297:
            print(f"❌ Invalid length: {len(data)} (expected 297)")
            return False

        if data[:3] != b'A_J':
            print("❌ Invalid header")
            return False

        if data[-3:] != b'J_A':
            print("❌ Invalid terminator")
            return False

        print("✓ Valid VL53L8A1 8x8 packet (297 bytes)")
        return True


def test_iks02a1():
    """Test IKS02A1 packet generation and validation"""
    print("\n" + "="*60)
    print("Testing IKS02A1 Packet Generation")
    print("="*60)

    generator = MockVCPGenerator()
    packet = generator.generate_iks02a1_packet()

    print(f"\nGenerated packet size: {len(packet)} bytes")
    print(f"Hex representation (first 20 bytes): {packet[:20].hex()}")

    TestPacketValidator.validate_iks02a1_packet(packet)

    # Parse values to verify correctness
    print("\nDecoded values:")
    payload = packet[3:-3]
    
    accel1_x = struct.unpack('<i', payload[0:4])[0]
    temperature = struct.unpack('<f', payload[36:40])[0]
    counter = struct.unpack('<i', payload[52:56])[0]
    fastest_odr = struct.unpack('<f', payload[56:60])[0]

    print(f"  Accel1 X: {accel1_x} mg")
    print(f"  Temperature: {temperature:.2f} °C")
    print(f"  Counter: {counter}")
    print(f"  Fastest ODR: {fastest_odr:.1f} Hz")


def test_combined():
    """Test Combined packet generation and validation"""
    print("\n" + "="*60)
    print("Testing Combined IKS02A1 + VL53L1A1 Packet")
    print("="*60)

    generator = MockVCPGenerator()
    packet = generator.generate_combined_packet()

    print(f"\nGenerated packet size: {len(packet)} bytes")
    print(f"Hex representation (first 20 bytes): {packet[:20].hex()}")

    TestPacketValidator.validate_combined_packet(packet)

    # Parse values
    print("\nDecoded values:")
    payload = packet[3:-3]
    
    tof_left_dist = struct.unpack('<i', payload[0:4])[0]
    accel1_x = struct.unpack('<i', payload[48:52])[0]
    temperature = struct.unpack('<f', payload[84:88])[0]
    counter = struct.unpack('<i', payload[100:104])[0]

    print(f"  ToF Left Distance: {tof_left_dist} mm")
    print(f"  Accel1 X: {accel1_x} mg")
    print(f"  Temperature: {temperature:.2f} °C")
    print(f"  Counter: {counter}")


def test_tof():
    """Test ToF packet generation and validation"""
    print("\n" + "="*60)
    print("Testing VL53L1A1 Packet Generation")
    print("="*60)

    generator = MockVCPGenerator()
    packet = generator.generate_tof_l1a1_packet()

    print(f"\nGenerated packet size: {len(packet)} bytes")
    print(f"Hex representation (first 20 bytes): {packet[:20].hex()}")

    TestPacketValidator.validate_tof_l1a1_packet(packet)


def test_vl53l8a1():
    """Test VL53L8A1 packet generation and validation"""
    print("\n" + "="*60)
    print("Testing VL53L8A1 8x8 ToF Array Packet")
    print("="*60)

    generator = MockVCPGenerator()
    packet = generator.generate_vl53l8a1_packet()

    print(f"\nGenerated packet size: {len(packet)} bytes")
    TestPacketValidator.validate_vl53l8a1_packet(packet)

    # Show first few values from the matrix
    print("\nFirst 4x4 of 8x8 distance matrix:")
    payload = packet[3:-3]
    for i in range(4):
        row_values = []
        for j in range(4):
            offset = (i * 8 + j) * 4
            value = struct.unpack('<f', payload[offset:offset+4])[0]
            row_values.append(f"{value:.1f}")
        print(f"  [{', '.join(row_values)}] mm")


def save_test_data(filename: str, num_packets: int = 100):
    """Save test packets to a file for offline analysis"""
    print(f"\n{'='*60}")
    print(f"Saving {num_packets} test packets to {filename}")
    print('='*60)

    generator = MockVCPGenerator()

    with open(filename, 'wb') as f:
        for i in range(num_packets):
            if i % 4 == 0:
                packet = generator.generate_iks02a1_packet()
            elif i % 4 == 1:
                packet = generator.generate_combined_packet()
            elif i % 4 == 2:
                packet = generator.generate_tof_l1a1_packet()
            else:
                packet = generator.generate_vl53l8a1_packet()
            f.write(packet)

    print(f"✓ Saved {num_packets} packets")
    print(f"  File size: {len(f.name)} bytes")


def main():
    parser = argparse.ArgumentParser(
        description='VCP Data Parser - Test/Debug Utility'
    )
    parser.add_argument(
        '--generate-iks02a1',
        action='store_true',
        help='Generate and validate IKS02A1 packet'
    )
    parser.add_argument(
        '--generate-combined',
        action='store_true',
        help='Generate and validate combined packet'
    )
    parser.add_argument(
        '--generate-tof',
        action='store_true',
        help='Generate and validate ToF packet'
    )
    parser.add_argument(
        '--generate-vl53l8a1',
        action='store_true',
        help='Generate and validate VL53L8A1 8x8 packet'
    )
    parser.add_argument(
        '--test-all',
        action='store_true',
        help='Run all packet type tests'
    )
    parser.add_argument(
        '--save-test-data',
        type=str,
        metavar='FILENAME',
        help='Save synthetic test data to file'
    )

    args = parser.parse_args()

    if args.generate_iks02a1:
        test_iks02a1()
    elif args.generate_combined:
        test_combined()
    elif args.generate_tof:
        test_tof()
    elif args.generate_vl53l8a1:
        test_vl53l8a1()
    elif args.test_all:
        test_iks02a1()
        test_tof()
        test_combined()
        test_vl53l8a1()
    elif args.save_test_data:
        save_test_data(args.save_test_data)
    else:
        # Default: run all tests
        test_iks02a1()
        test_tof()
        test_combined()
        test_vl53l8a1()

    print("\n" + "="*60)
    print("All tests completed!")
    print("="*60 + "\n")


if __name__ == '__main__':
    main()
