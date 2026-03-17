# UART Protocol Quick Reference - AP_01 Project

## Summary of Implementations

### 1. IKS02A1 Source Code Full
**File**: `IKS02A1 Source Code Full/Core/Src/main.c`

```c
void sendToSimulink(){
    HAL_UART_Transmit(&huart2, (uint8_t *) &header,3,HAL_MAX_DELAY);
    
    // 11 sensors × 4 bytes = 44 bytes
    HAL_UART_Transmit(&huart2, (int32_t *) &(accel1_axis.x),4,HAL_MAX_DELAY);  // 0-3
    HAL_UART_Transmit(&huart2, (int32_t *) &accel1_axis.y,4,HAL_MAX_DELAY);    // 4-7
    HAL_UART_Transmit(&huart2, (int32_t *) &accel1_axis.z,4,HAL_MAX_DELAY);    // 8-11
    HAL_UART_Transmit(&huart2, (int32_t *) &gyro_axis.x,4,HAL_MAX_DELAY);      // 12-15
    HAL_UART_Transmit(&huart2, (int32_t *) &gyro_axis.y,4,HAL_MAX_DELAY);      // 16-19
    HAL_UART_Transmit(&huart2, (int32_t *) &gyro_axis.z,4,HAL_MAX_DELAY);      // 20-23
    HAL_UART_Transmit(&huart2, (int32_t *) &accel2_axis.x,4,HAL_MAX_DELAY);    // 24-27
    HAL_UART_Transmit(&huart2, (int32_t *) &accel2_axis.y,4,HAL_MAX_DELAY);    // 28-31
    HAL_UART_Transmit(&huart2, (int32_t *) &accel2_axis.z,4,HAL_MAX_DELAY);    // 32-35
    HAL_UART_Transmit(&huart2, (float_t *) &temperature,4,HAL_MAX_DELAY);      // 36-39
    HAL_UART_Transmit(&huart2, (int32_t *) &mag_axis.x,4,HAL_MAX_DELAY);       // 40-43
    HAL_UART_Transmit(&huart2, (int32_t *) &mag_axis.y,4,HAL_MAX_DELAY);       // 44-47
    HAL_UART_Transmit(&huart2, (int32_t *) &mag_axis.z,4,HAL_MAX_DELAY);       // 48-51
    
    HAL_UART_Transmit(&huart2, (int32_t *) &counter,4,HAL_MAX_DELAY);          // 52-55
    HAL_UART_Transmit(&huart2, (float_t *) &fastestODR,4,HAL_MAX_DELAY);       // 56-59
    
    HAL_UART_Transmit(&huart2, (uint8_t *) &terminator,3,HAL_MAX_DELAY);
}

// Total: 3 + 60 + 3 = 66 bytes per packet
// Payload: 60 bytes
```

**Payload Structure (60 bytes)**:
```
[0-3]   accel1.x (int32)
[4-7]   accel1.y (int32)
[8-11]  accel1.z (int32)
[12-15] gyro.x (int32)
[16-19] gyro.y (int32)
[20-23] gyro.z (int32)
[24-27] accel2.x (int32)
[28-31] accel2.y (int32)
[32-35] accel2.z (int32)
[36-39] temperature (float32)
[40-43] mag.x (int32)
[44-47] mag.y (int32)
[48-51] mag.z (int32)
[52-55] counter (int32)
[56-59] fastestODR (float32)
```

---

### 2. VL53L1A1 Source Code Full
**File**: `VL53L1A1 Source Code Full/Core/Src/main.c`

```c
void sendToSimulink(){
    HAL_UART_Transmit(&huart2, (uint8_t *) &header,3,HAL_MAX_DELAY);
    
    // 3 ToF sensors × 4 values × 4 bytes = 48 bytes
    HAL_UART_Transmit(&huart2, &(TOF_left_result.Distance),4,HAL_MAX_DELAY);   // 0-3
    HAL_UART_Transmit(&huart2, &(TOF_left_result.Ambient),4,HAL_MAX_DELAY);    // 4-7
    HAL_UART_Transmit(&huart2, &(TOF_left_result.Signal),4,HAL_MAX_DELAY);     // 8-11
    HAL_UART_Transmit(&huart2, &(TOF_left_result.Status),4,HAL_MAX_DELAY);     // 12-15
    
    HAL_UART_Transmit(&huart2, &(TOF_centre_result.Distance),4,HAL_MAX_DELAY); // 16-19
    HAL_UART_Transmit(&huart2, &(TOF_centre_result.Ambient),4,HAL_MAX_DELAY);  // 20-23
    HAL_UART_Transmit(&huart2, &(TOF_centre_result.Signal),4,HAL_MAX_DELAY);   // 24-27
    HAL_UART_Transmit(&huart2, &(TOF_centre_result.Status),4,HAL_MAX_DELAY);   // 28-31
    
    HAL_UART_Transmit(&huart2, &(TOF_right_result.Distance),4,HAL_MAX_DELAY);  // 32-35
    HAL_UART_Transmit(&huart2, &(TOF_right_result.Ambient),4,HAL_MAX_DELAY);   // 36-39
    HAL_UART_Transmit(&huart2, &(TOF_right_result.Signal),4,HAL_MAX_DELAY);    // 40-43
    HAL_UART_Transmit(&huart2, &(TOF_right_result.Status),4,HAL_MAX_DELAY);    // 44-47
    
    HAL_UART_Transmit(&huart2, (uint32_t *) &counter,4,HAL_MAX_DELAY);         // 48-51
    
    HAL_UART_Transmit(&huart2, (uint8_t *) &terminator,3,HAL_MAX_DELAY);
}

// Total: 3 + 52 + 3 = 58 bytes per packet
// Payload: 52 bytes
```

**Payload Structure (52 bytes)**:
```
[0-3]   tof_left.distance (uint32)
[4-7]   tof_left.ambient (uint32)
[8-11]  tof_left.signal (uint32)
[12-15] tof_left.status (uint32)
[16-19] tof_centre.distance (uint32)
[20-23] tof_centre.ambient (uint32)
[24-27] tof_centre.signal (uint32)
[28-31] tof_centre.status (uint32)
[32-35] tof_right.distance (uint32)
[36-39] tof_right.ambient (uint32)
[40-43] tof_right.signal (uint32)
[44-47] tof_right.status (uint32)
[48-51] counter (uint32)
```

---

### 3. IKS02A1 VL53L1A1 Source Code Both
**File**: `IKS02A1 VL53L1A1 Source Code Both/Core/Src/main.c`

```c
void sendToSimulink(){
    HAL_UART_Transmit(&huart2, (uint8_t *) &header,3,HAL_MAX_DELAY);

    // ToF data: 48 bytes (same as VL53L1A1 only)
    HAL_UART_Transmit(&huart2, &(TOF_left_result.Distance),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &(TOF_left_result.Ambient),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &(TOF_left_result.Signal),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &(TOF_left_result.Status),4,HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, &(TOF_centre_result.Distance),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &(TOF_centre_result.Ambient),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &(TOF_centre_result.Signal),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &(TOF_centre_result.Status),4,HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, &(TOF_right_result.Distance),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &(TOF_right_result.Ambient),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &(TOF_right_result.Signal),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &(TOF_right_result.Status),4,HAL_MAX_DELAY);

    // IMU data: 52 bytes
    HAL_UART_Transmit(&huart2, (int32_t *) &(accel1_axis.x),4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel1_axis.y,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel1_axis.z,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &gyro_axis.x,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &gyro_axis.y,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &gyro_axis.z,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel2_axis.x,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel2_axis.y,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &accel2_axis.z,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *) &temperature,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &mag_axis.x,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &mag_axis.y,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (int32_t *) &mag_axis.z,4,HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (int32_t *) &counter,4,HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (float_t *) &fastestODR,4,HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart2, (uint8_t *) &terminator,3,HAL_MAX_DELAY);
}

// Total: 3 + 100 + 3 = 106 bytes per packet
// Payload: 100 bytes
```

**Payload Structure (100 bytes)**:
```
TOF DATA (48 bytes):
[0-3]   tof_left.distance
[4-7]   tof_left.ambient
[8-11]  tof_left.signal
[12-15] tof_left.status
[16-19] tof_centre.distance
[20-23] tof_centre.ambient
[24-27] tof_centre.signal
[28-31] tof_centre.status
[32-35] tof_right.distance
[36-39] tof_right.ambient
[40-43] tof_right.signal
[44-47] tof_right.status

IMU DATA (52 bytes):
[48-51] accel1.x
[52-55] accel1.y
[56-59] accel1.z
[60-63] gyro.x
[64-67] gyro.y
[68-71] gyro.z
[72-75] accel2.x
[76-79] accel2.y
[80-83] accel2.z
[84-87] temperature
[88-91] mag.x
[92-95] mag.y
[96-99] mag.z

METADATA (8 bytes):
[100-103] counter
[104-107] fastestODR
```

---

## UART Configuration (All Variants)

```
BaudRate:     1,843,200 bps (0x1C2000)
WordLength:   9 bits (including parity)
StopBits:     2 UART_STOPBITS_2
Parity:       EVEN (UART_PARITY_EVEN)
FlowControl:  None (UART_HWCONTROL_NONE)
Timeout:      HAL_MAX_DELAY
```

---

## Key Functions Used

### sendToSimulink()
- Transmits complete sensor packet
- Called by ISR (interrupt service routine) at timer interrupt
- Uses blocking transmission (HAL_MAX_DELAY)

### initialCalibration() / receivedFromSimulink()
- Receives configuration packet from host
- Sets sensor parameters:
  - Full-scale range (FSR)
  - Output data rate (ODR)
  - ToF timing budget
  - ToF polling period

### Conversion Functions

**In main.c**:
```c
float bytesToFloat_main(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
    float result;
    uint8_t bytes[4] = {b1, b2, b3, b4};
    memcpy(&result, bytes, sizeof(float));
    return result;
}

int32_t bytesToInt32_main(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
    int32_t result;
    uint8_t bytes[4] = {b1, b2, b3, b4};
    memcpy(&result, bytes, sizeof(int32_t));
    return result;
}
```

**Byte Order**: Little-endian (LSB first)

---

## Performance Metrics

| Parameter | Value |
|-----------|-------|
| Baud Rate | 1.8432 Mbps |
| Bits per byte | 12 (1 start + 9 data + 2 stop) |
| Throughput | ~153.6 kBps |
| IKS02A1 Full (66 bytes) | ~0.43 ms per packet |
| VL53L1A1 Full (58 bytes) | ~0.38 ms per packet |
| Combined (106 bytes) | ~0.69 ms per packet |
| Max throughput @ 100 Hz | ~6.6 kBps |

---

## Error Detection

### Packet Validation
1. **Header Check**: `{'J', '_', 'A'}` at bytes 0-2
2. **Terminator Check**: `{'A', '_', 'J'}` at expected end
3. **Counter**: Validates no packets lost (if implemented)
4. **Parity**: Hardware parity bit (even parity enabled)

### Known Issues
- No software CRC (relies on parity bit)
- HAL_UART_Transmit blocks execution
- No flow control (assumes reliable link)

---

## Common Issues & Solutions

### Issue: "Cannot parse packet"
**Cause**: Serial port buffer misalignment  
**Solution**: Increase `buffer` size in parser, check baud rate

### Issue: "Incomplete packets"
**Cause**: USB cable quality or long cable  
**Solution**: Use shielded USB cable, reduce cable length

### Issue: "Temperature reading invalid"
**Cause**: Float representation mismatch  
**Solution**: Verify little-endian byte order in conversion

---

## References

- **STM32F411 UART HAL**: `stm32f4xx_hal_uart.h`
- **VL53L1X API**: `VL53L1X_api.h`
- **IKS02A1 BSP**: `iks02a1_motion_sensors.h`

---

**Last Updated**: 2025-03-17  
**Author**: Jesse Jabez Arendse
