# VCP Data Parser and Plotter - AP_01 Project

## Overview

This Python script provides real-time parsing and visualization of sensor data transmitted from the STM32 Nucleo board over a Virtual COM Port (VCP). It supports all implementations of the AP_01 project:

- **IKS02A1 Source Code Full**: IMU sensors only (Accel, Gyro, Magnetometer, Temperature)
- **IKS02A1 Source Code LP Accel**: Low-power accelerometer variant
- **IKS02A1 VL53L1A1 Source Code Both**: IMU + single-pixel ToF sensor
- **IKS02A1 VL53L8A1 Source Code Both**: IMU + 8x8 ToF array
- **VL53L1A1 Source Code Full**: Single-pixel ToF sensor only
- **VL53L8A1 Source Code Full**: 8x8 ToF array only

## UART Protocol Analysis

### Protocol Structure

All data is transmitted using the STM32 HAL UART functions with the following structure:

```
[Header (3 bytes)] [Payload (variable)] [Terminator (3 bytes)]
   'A' 'J' '_'                            'J' 'A' '_'
```

### Data Formats

#### 1. **IKS02A1 Only** (Payload: 36 bytes)

Raw data structure (36 bytes):
```
Offset  0- 3: int32_t  accel1_x
Offset  4- 7: int32_t  accel1_y
Offset  8-11: int32_t  accel1_z
Offset 12-15: int32_t  gyro_x
Offset 16-19: int32_t  gyro_y
Offset 20-23: int32_t  gyro_z
Offset 24-27: int32_t  accel2_x
Offset 28-31: int32_t  accel2_y
Offset 32-35: float32  temperature
```

Note: Full IKS02A1 packet also includes:
- int32_t accel2_z (4 bytes)
- float32 mag_x, mag_y, mag_z (12 bytes)
- int32_t counter (4 bytes)
- float32 fastestODR (4 bytes)

#### 2. **VL53L1A1 Only** (Payload: 36 bytes)

Raw data structure (36 bytes, 3 sensors):
```
Offset  0- 3: uint32_t tof_left_distance
Offset  4- 7: uint32_t tof_left_ambient
Offset  8-11: uint32_t tof_left_signal
Offset 12-15: uint32_t tof_left_status
Offset 16-19: uint32_t tof_centre_distance
Offset 20-23: uint32_t tof_centre_ambient
Offset 24-27: uint32_t tof_centre_signal
Offset 28-31: uint32_t tof_centre_status
Offset 32-35: uint32_t counter
```

#### 3. **Combined IKS02A1 + VL53L1A1** (Payload: 76 bytes)

Raw data structure (76 bytes):
```
Offset  0- 3: int32_t  tof_left_distance
Offset  4- 7: int32_t  tof_left_ambient
Offset  8-11: int32_t  tof_left_signal
Offset 12-15: int32_t  tof_left_status
Offset 16-19: int32_t  tof_centre_distance
Offset 20-23: int32_t  tof_centre_ambient
Offset 24-27: int32_t  tof_centre_signal
Offset 28-31: int32_t  tof_centre_status
Offset 32-35: int32_t  tof_right_distance
Offset 36-39: int32_t  tof_right_ambient
Offset 40-43: int32_t  tof_right_signal
Offset 44-47: int32_t  tof_right_status
Offset 48-51: int32_t  accel1_x
Offset 52-55: int32_t  accel1_y
Offset 56-59: int32_t  accel1_z
Offset 60-63: int32_t  gyro_x
Offset 64-67: int32_t  gyro_y
Offset 68-71: int32_t  gyro_z
Offset 72-75: int32_t  accel2_x
Offset 76-79: int32_t  accel2_y
Offset 80-83: int32_t  accel2_z
Offset 84-87: float32  temperature
Offset 88-91: int32_t  mag_x
Offset 92-95: int32_t  mag_y
Offset 96-99: int32_t  mag_z
Offset 100-103: int32_t counter
Offset 104-107: float32 fastestODR
```

#### 4. **VL53L8A1 8x8 Array** (Payload: 288 bytes)

Raw data structure (288 bytes = 8x8 float32 matrix):
```
Offset  0-3, 4-7, ..., 284-287: float32 distance_matrix[8][8]
```

### Key Observations on UART Functions

1. **HAL_UART_Transmit() Usage**:
   - All data is transmitted using `HAL_UART_Transmit(&huart2, data_ptr, size, HAL_MAX_DELAY)`
   - Each field is transmitted as 4 bytes with `HAL_MAX_DELAY` timeout
   - Header (3 bytes) and terminator (3 bytes) are transmitted separately

2. **UART Configuration**:
   - **Baud Rate**: 1,843,200 bps (very high speed for low latency)
   - **Word Length**: 9 bits
   - **Stop Bits**: 2
   - **Parity**: Even
   - **Flow Control**: None

3. **Data Integrity**:
   - Header validation: `{'J', '_', 'A'}`
   - Terminator validation: `{'A', '_', 'J'}`
   - Packet counter included in most variants for loss detection

4. **Transmission Efficiency**:
   - Fixed packet sizes allow deterministic transmission time
   - No checksums (relies on UART parity bit)
   - Straightforward byte-by-byte transmission

## Installation

### Requirements
- Python 3.7+
- numpy
- matplotlib
- pyserial

### Setup

```bash
pip install numpy matplotlib pyserial
```

## Usage

### List Available COM Ports

```bash
python vcp_data_parser.py --list-ports
```

### Start Monitoring

```bash
python vcp_data_parser.py --port COM3
```

With custom baud rate:

```bash
python vcp_data_parser.py --port COM3 --baud 1843200
```

On Linux:

```bash
python vcp_data_parser.py --port /dev/ttyUSB0 --baud 1843200
```

## Features

### Real-Time Plotting

The script automatically detects the sensor configuration and displays relevant plots:

#### For IKS02A1 (IMU):
- **Accelerometer 1**: X, Y, Z axes
- **Accelerometer 2**: X, Y, Z axes
- **Gyroscope**: X, Y, Z rotation rates
- **Temperature**: Ambient temperature
- **Magnetometer**: X, Y, Z magnetic field
- **Packet Counter**: Data integrity monitoring

#### For VL53L1A1 (Single Pixel ToF):
- **Distance**: Left, Center, Right sensors
- **Ambient Light**: Background luminosity
- **Signal Rate**: Received signal strength
- **Range Status**: Measurement validity

#### For VL53L8A1 (8x8 ToF Array):
- **Distance Heatmap**: Real-time 8x8 distance matrix visualization

#### For Combined Sensors:
- All IMU plots
- All ToF plots

### Automatic Configuration Detection

The parser automatically detects which sensors are connected based on packet size:

| Packet Size | Configuration |
|---|---|
| 9 bytes (header+terminator) | Error/Minimal data |
| 42 bytes (9 + 36 payload) | Single sensor type |
| 85 bytes (9 + 76 payload) | IKS02A1 + VL53L1A1 |
| 297 bytes (9 + 288 payload) | VL53L8A1 8x8 array |

## Data Processing

### Byte-to-Value Conversion

The script implements the same conversion functions as the embedded code:

```python
# Float conversion (little-endian)
bytes_to_float(b1, b2, b3, b4) -> float

# Integer conversion (little-endian, signed)
bytes_to_int32(b1, b2, b3, b4) -> int

# Integer conversion (little-endian, unsigned)
bytes_to_uint32(b1, b2, b3, b4) -> int
```

### Packet Validation

Each packet is validated for:
1. Valid header sequence
2. Valid terminator sequence
3. Expected payload size
4. Proper byte alignment

## Troubleshooting

### No Data Received

1. Check UART configuration in STM32CubeIDE matches script (1843200 baud)
2. Verify USB cable connection
3. Check Device Manager for correct COM port
4. Ensure serial port is not opened by another application

### Incomplete Packets

- The parser automatically buffers and searches for complete packets
- Partial data is discarded and logged
- Check USB cable quality for reliable connection

### Memory Usage

- Default buffer stores last 500 packets
- Modify `max_points=500` in `DataPlotter()` to adjust memory usage
- Approximately 100 KB per 500 packets

## File Structure

```
src/
├── Plotting/
│   ├── vcp_data_parser.py      (This script)
│   ├── VCP_Parser_Analysis.md  (This file)
│   ├── DistanceVisualizer.m    (MATLAB visualization)
│   ├── OrientationPlot.m       (MATLAB orientation)
│   └── PCMPlot.m               (MATLAB audio plotting)
```

## Contributing

To add support for additional sensor configurations:

1. Define new `@dataclass` in the script
2. Add parsing logic in `_parse_packet()` method
3. Update `setup_plots()` with new visualization axes
4. Add update logic in `update()` method

## License

This project is part of the AP_01 research initiative.

## Author

Jesse Jabez Arendse
2025
