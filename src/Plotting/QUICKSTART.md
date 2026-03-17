# Quick Start Guide - VCP Data Parser

## Installation

### Step 1: Install Python Dependencies

```bash
pip install numpy matplotlib pyserial
```

Alternatively, using a requirements file:

```bash
pip install -r requirements.txt
```

### Step 2: Locate Your COM Port

**Windows:**
```bash
python vcp_data_parser.py --list-ports
```

**Linux/Mac:**
```bash
python3 vcp_data_parser.py --list-ports
```

You should see something like:
```
Available COM ports:
  0: COM3 - STMicroelectronics STLink Virtual COM Port
```

## Usage

### Basic Usage

**Windows:**
```bash
python vcp_data_parser.py --port COM3
```

**Linux/Mac:**
```bash
python3 vcp_data_parser.py --port /dev/ttyUSB0
```

### With Custom Baud Rate

```bash
python vcp_data_parser.py --port COM3 --baud 1843200
```

### Testing Without Hardware

Generate synthetic data to test the parser:

```bash
python vcp_data_parser_test.py --test-all
```

Generate specific packet types:
```bash
python vcp_data_parser_test.py --generate-combined
python vcp_data_parser_test.py --generate-vl53l8a1
```

## What You'll See

### Real-Time Plot Windows

Once connected, you'll see an interactive plot showing:

#### For IKS02A1 (IMU):
- **Top-Left**: Accelerometer 1 (X, Y, Z)
- **Top-Middle**: Accelerometer 2 (X, Y, Z)
- **Top-Right**: Gyroscope (X, Y, Z)
- **Middle-Left**: Temperature
- **Middle-Middle**: Magnetometer (X, Y, Z)
- **Middle-Right**: ToF Distance (if combined)

#### For ToF Sensors:
- **Distance Plot**: Left, Center, Right values over time
- **Ambient Light**: Background luminosity
- **Signal Rate**: Received signal strength
- **Range Status**: Data validity indicators

#### For VL53L8A1 8x8 Array:
- **Heatmap**: Real-time 8x8 distance matrix with color scale

## UART Verification

### Check COM Port Status

**Windows (PowerShell):**
```powershell
Get-CimInstance -Class Win32_SerialPort
```

**Linux:**
```bash
ls -la /dev/ttyUSB*
```

### Test UART Communication

**Windows (using Tera Term or similar):**
1. Open COM port
2. Set baud rate to 1843200
3. Should see continuous data stream

**Linux (using minicom):**
```bash
minicom -D /dev/ttyUSB0 -b 1843200
```

## Troubleshooting

### "No data received"

1. **Check connection:**
   - Verify USB cable is properly connected
   - Check Device Manager for COM port

2. **Check baud rate:**
   - Must be 1843200 bps
   ```bash
   python vcp_data_parser.py --port COM3 --baud 1843200
   ```

3. **Check STM32 firmware:**
   - Ensure board is programmed with correct code
   - LED indicator should be on (GREEN_LED_Pin or LD2_Pin)

### "Port already in use"

- Close any other serial monitor applications
- Windows: `taskkill /F /IM putty.exe` (or relevant app)

### "Incorrect data received"

- Check packet alignment (header: 'A_J', terminator: 'J_A')
- Verify sensor FSR/ODR settings match firmware

## File Structure

```
Plotting/
├── vcp_data_parser.py              # Main plotting script
├── vcp_data_parser_test.py         # Test/debug script
├── VCP_Parser_Analysis.md          # Detailed analysis
├── UART_Protocol_Reference.md      # Protocol documentation
├── README.md                       # This file
├── DistanceVisualizer.m            # MATLAB visualization
├── OrientationPlot.m               # MATLAB orientation
└── PCMPlot.m                       # MATLAB audio plotting
```

## Advanced Usage

### Save Data to CSV

You can modify the script to save data:

```python
import csv
import time

# After connecting and collecting data
with open('sensor_data.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'accel_x', 'accel_y', 'accel_z', ...])
    for packet in monitor.plotter.packets:
        writer.writerow([
            packet.timestamp,
            packet.accel1_x,
            packet.accel1_y,
            packet.accel1_z,
            ...
        ])
```

### Process Offline Data

```python
from vcp_data_parser import VCPDataParser

# Read binary file
with open('sensor_data.bin', 'rb') as f:
    data = f.read()

# Parse packets
parser = VCPDataParser()
packets = parser.add_data(data)

# Process packets
for packet in packets:
    print(f"Counter: {packet.counter}")
    if hasattr(packet, 'accel1_x'):
        print(f"Accel X: {packet.accel1_x}")
```

### Custom Data Filtering

```python
# Filter packets by timestamp
start_time = time.time()
filtered_packets = [
    p for p in monitor.plotter.packets 
    if p.timestamp - start_time > 5  # Only packets after 5 seconds
]
```

## Performance Tips

1. **Reduce plot update rate:**
   ```python
   ani = FuncAnimation(
       fig, update, 
       interval=500,  # Update every 500ms instead of 100ms
       blit=True
   )
   ```

2. **Limit buffer size:**
   ```python
   plotter = DataPlotter(max_points=200)  # Instead of 500
   ```

3. **Reduce figure quality:**
   - Close unused subplots
   - Simplify plot styles

## Hardware Verification

### Expected Packet Rates

| Configuration | Typical Rate | Packet Size | Data Rate |
|---|---|---|---|
| IKS02A1 only | 100 Hz | 66 bytes | 6.6 kBps |
| VL53L1A1 only | 30 Hz | 58 bytes | 1.74 kBps |
| Combined | 30 Hz | 106 bytes | 3.18 kBps |
| VL53L8A1 | 30 Hz | 297 bytes | 8.91 kBps |

### Example: Calculate ODR from Counter

If you see 300 packets in 10 seconds:
- ODR = 300 / 10 = 30 Hz ✓

## Data Validation

### Check Packet Integrity

The script automatically validates:
- ✓ Header presence (`A_J`)
- ✓ Terminator presence (`J_A`)
- ✓ Payload size (matches expected configuration)
- ✓ Byte alignment

### Manual Verification

```bash
python vcp_data_parser_test.py --save-test-data test_data.bin
# Then open test_data.bin with vcp_data_parser.py
```

## Example Workflows

### Workflow 1: Quick Sensor Check

```bash
# 1. List ports
python vcp_data_parser.py --list-ports

# 2. Start monitoring
python vcp_data_parser.py --port COM3

# 3. Check plots appear
# 4. Verify data is updating (packet counter should increase)
# 5. Press Ctrl+C to exit
```

### Workflow 2: Debug UART Issues

```bash
# 1. Generate test data
python vcp_data_parser_test.py --test-all

# 2. Test packet parsing
python vcp_data_parser_test.py --generate-combined

# 3. If parser works, issue is with hardware
# 4. Check STM32 code for UART initialization
```

### Workflow 3: Data Collection & Analysis

```bash
# 1. Connect board
# 2. Run parser and let it collect data for required duration
# 3. Press Ctrl+C to stop
# 4. Save packets to CSV (using custom code above)
# 5. Analyze in MATLAB or Python
```

## Common Parameters

### Sensor Configuration Defaults

```c
// UART
BaudRate:    1843200 bps
WordLength:  9 bits
StopBits:    2
Parity:      EVEN

// IKS02A1 (IMU)
Accel1 ODR:  208 Hz
Gyro ODR:    208 Hz
Accel2 ODR:  200 Hz  (LP Accel variant)
Mag ODR:     20 Hz

// VL53L1A1 (ToF)
Timing Budget:  30 ms
ODR:            30 Hz (from timing budget)
Distance Mode:  1 (short) or 2 (long)
```

## Support & Resources

### Documentation Files

1. **vcp_data_parser.py** - Main script (inline comments)
2. **VCP_Parser_Analysis.md** - Protocol analysis
3. **UART_Protocol_Reference.md** - UART details
4. **README.md** - Full documentation

### Related STM32 Code Files

- `IKS02A1 Source Code Full/Core/Src/main.c`
- `IKS02A1 VL53L1A1 Source Code Both/Core/Src/main.c`
- `VL53L1A1 Source Code Full/Core/Src/main.c`

## Keyboard Shortcuts

While the plot is open:

- **Home**: Reset view
- **Back**: Undo zoom
- **Forward**: Redo zoom
- **Pan (drag)**: Left click and drag to pan
- **Zoom (box)**: Right click and drag to zoom to box
- **Save Figure**: File → Save (Ctrl+S in some backends)
- **Close**: Ctrl+W or close window

## Performance Metrics

### CPU Usage
- Typical: 2-5% (single core)
- Peak: 10-15% (during plot updates)

### Memory Usage
- With 500 packets: ~50 MB
- Grows at ~100 KB per 500 packets

### Latency
- Display update: 100 ms (10 Hz)
- USB transfer: ~1-5 ms per packet
- Total end-to-end: 50-150 ms

---

**Last Updated**: 2025-03-17  
**Author**: Jesse Jabez Arendse  
**Version**: 1.0
