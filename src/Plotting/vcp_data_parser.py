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
import webbrowser
import logging
import numpy as np
from flask import Flask, Response, jsonify


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

    HEADER = b'J_A'
    TERMINATOR = b'A_J'

    def __init__(self, verbose: bool = False):
        self.buffer = bytearray()
        self.verbose = verbose

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

        while len(self.buffer) >= 10:
            header_idx = self.buffer.find(self.HEADER)

            if header_idx == -1:
                # No header found; keep last 2 bytes in case header spans reads
                self.buffer = self.buffer[-2:]
                break

            if header_idx > 0:
                self.buffer = self.buffer[header_idx:]

            terminator_idx = self.buffer.find(self.TERMINATOR, len(self.HEADER))
            if terminator_idx == -1:
                break

            packet_data = bytes(self.buffer[:terminator_idx + len(self.TERMINATOR)])
            self.buffer = self.buffer[terminator_idx + len(self.TERMINATOR):]

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

        if payload_size == 52:  # VL53L1A1 only
            packet = self._parse_vl53l1a1_only(data)
        elif payload_size == 60:  # IKS02A1 only
            packet = self._parse_iks02a1_only(data)
        elif payload_size in (100, 108, 114, 228):  # Combined IKS02A1 + VL53L1A1
            packet = self._parse_combined(data)
        elif payload_size == 288:  # VL53L8A1 (64 floats for 8x8 matrix)
            packet = self._parse_vl53l8a1(data)
        else:
            if self.verbose:
                print(f"Unknown packet size: {payload_size}, data: {data.hex()}")
            return None

        if self.verbose and packet:
            print(f"Parsed packet: {type(packet).__name__} (size {payload_size})")

        return packet

    def _parse_iks02a1_only(self, data: bytes) -> Optional[IKS02A1Packet]:
        """Parse IKS02A1 only packet (60 bytes payload)."""
        payload = data[3:-3]
        if len(payload) != 60:
            return None

        try:
            accel1_x = self.bytes_to_int32(payload[0], payload[1], payload[2], payload[3])
            accel1_y = self.bytes_to_int32(payload[4], payload[5], payload[6], payload[7])
            accel1_z = self.bytes_to_int32(payload[8], payload[9], payload[10], payload[11])
            gyro_x = self.bytes_to_int32(payload[12], payload[13], payload[14], payload[15])
            gyro_y = self.bytes_to_int32(payload[16], payload[17], payload[18], payload[19])
            gyro_z = self.bytes_to_int32(payload[20], payload[21], payload[22], payload[23])
            accel2_x = self.bytes_to_int32(payload[24], payload[25], payload[26], payload[27])
            accel2_y = self.bytes_to_int32(payload[28], payload[29], payload[30], payload[31])
            accel2_z = self.bytes_to_int32(payload[32], payload[33], payload[34], payload[35])
            temperature = self.bytes_to_float(payload[36], payload[37], payload[38], payload[39])
            mag_x = self.bytes_to_int32(payload[40], payload[41], payload[42], payload[43])
            mag_y = self.bytes_to_int32(payload[44], payload[45], payload[46], payload[47])
            mag_z = self.bytes_to_int32(payload[48], payload[49], payload[50], payload[51])
            counter = self.bytes_to_int32(payload[52], payload[53], payload[54], payload[55])
            fastest_odr = self.bytes_to_float(payload[56], payload[57], payload[58], payload[59])

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
                mag_x=mag_x,
                mag_y=mag_y,
                mag_z=mag_z,
                fastest_odr=fastest_odr,
            )
        except Exception as e:
            print(f"Error parsing IKS02A1 packet: {e}")
            return None

    def _parse_vl53l1a1_only(self, data: bytes) -> Optional[VL53L1A1Packet]:
        """Parse VL53L1A1 only packet (52 bytes payload)."""
        payload = data[3:-3]
        if len(payload) != 52:
            return None

        try:
            tof_left_distance = self.bytes_to_uint32(payload[0], payload[1], payload[2], payload[3])
            tof_left_ambient = self.bytes_to_uint32(payload[4], payload[5], payload[6], payload[7])
            tof_left_signal = self.bytes_to_uint32(payload[8], payload[9], payload[10], payload[11])
            tof_left_status = self.bytes_to_uint32(payload[12], payload[13], payload[14], payload[15])
            tof_centre_distance = self.bytes_to_uint32(payload[16], payload[17], payload[18], payload[19])
            tof_centre_ambient = self.bytes_to_uint32(payload[20], payload[21], payload[22], payload[23])
            tof_centre_signal = self.bytes_to_uint32(payload[24], payload[25], payload[26], payload[27])
            tof_centre_status = self.bytes_to_uint32(payload[28], payload[29], payload[30], payload[31])
            tof_right_distance = self.bytes_to_uint32(payload[32], payload[33], payload[34], payload[35])
            tof_right_ambient = self.bytes_to_uint32(payload[36], payload[37], payload[38], payload[39])
            tof_right_signal = self.bytes_to_uint32(payload[40], payload[41], payload[42], payload[43])
            tof_right_status = self.bytes_to_uint32(payload[44], payload[45], payload[46], payload[47])
            counter = self.bytes_to_uint32(payload[48], payload[49], payload[50], payload[51])

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
                tof_right_distance=tof_right_distance,
                tof_right_ambient=tof_right_ambient,
                tof_right_signal=tof_right_signal,
                tof_right_status=tof_right_status,
            )
        except Exception as e:
            print(f"Error parsing VL53L1A1 packet: {e}")
            return None

    def _parse_combined(self, data: bytes) -> Optional[CombinedPacket]:
        """Parse combined IKS02A1 + VL53L1A1 packet (76 bytes payload)"""
        payload = data[3:-3]

        if len(payload) not in (100, 108):
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

            if len(payload) >= offset + 8:
                counter = self.bytes_to_int32(
                    payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
                )
                offset += 4
                fastest_odr = self.bytes_to_float(
                    payload[offset], payload[offset + 1], payload[offset + 2], payload[offset + 3]
                )
            else:
                counter = 0
                fastest_odr = 0.0

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


# ---------------------------------------------------------------------------
# Flask dashboard HTML (served at /)
# ---------------------------------------------------------------------------
HTML_DASHBOARD = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>AP_01 Sensor Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4/dist/chart.umd.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#e6edf3;font-family:'Segoe UI',system-ui,sans-serif;font-size:14px}
header{display:flex;align-items:center;gap:20px;padding:12px 20px;background:#161b22;border-bottom:1px solid #30363d;flex-wrap:wrap}
header h1{font-size:1.1rem;font-weight:700;color:#58a6ff;margin-right:4px}
.badge{padding:3px 10px;border-radius:12px;font-size:0.75rem;font-weight:700;background:#1f3a1f;color:#3fb950}
.badge.err{background:#3d1a1a;color:#f85149}
.badge.warn{background:#2d2200;color:#d29922}
.stat{font-size:0.8rem;color:#8b949e}
.stat b{color:#e6edf3;font-size:0.95rem}
#config-bar{padding:7px 20px;background:#0d1117;border-bottom:1px solid #21262d;font-size:0.78rem;color:#8b949e;min-height:28px}
#config-bar b{color:#d2a679}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:14px;padding:16px 20px}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px}
.card h2{font-size:0.68rem;text-transform:uppercase;letter-spacing:.08em;color:#8b949e;margin-bottom:10px}
.vals{display:flex;gap:10px;margin-bottom:10px;flex-wrap:wrap}
.vb{flex:1;min-width:60px}
.vlabel{font-size:0.62rem;color:#8b949e;text-transform:uppercase;margin-bottom:2px}
.vnum{font-size:1.25rem;font-weight:700;font-variant-numeric:tabular-nums;line-height:1.2}
.vnum.x{color:#f85149}.vnum.y{color:#3fb950}.vnum.z{color:#58a6ff}
.vnum.d{color:#e6edf3}.vnum.t{color:#d2a679}
.vstatus{font-size:0.68rem;margin-top:2px}
.vstatus.ok{color:#3fb950}.vstatus.warn{color:#d29922}.vstatus.err{color:#f85149}
.ch{height:80px;position:relative}
</style>
</head>
<body>
<header>
  <h1>AP_01 Sensor Dashboard</h1>
  <div id="status-badge" class="badge err">Waiting</div>
  <div class="stat">Rate: <b id="rate">--</b> Hz</div>
  <div class="stat">Firmware ODR: <b id="fw-odr">--</b> Hz</div>
  <div class="stat">Packet #<b id="counter">--</b></div>
  <div class="stat">Temp: <b id="temp-hdr">--</b> &deg;C</div>
</header>
<div id="config-bar">Waiting for calibration config&hellip;</div>
<div class="grid">

  <div class="card">
    <h2>Time-of-Flight Distance (mm)</h2>
    <div class="vals">
      <div class="vb"><div class="vlabel">Left</div><div class="vnum d" id="tof-l">--</div><div class="vstatus" id="tof-l-st"></div></div>
      <div class="vb"><div class="vlabel">Centre</div><div class="vnum d" id="tof-c">--</div><div class="vstatus" id="tof-c-st"></div></div>
      <div class="vb"><div class="vlabel">Right</div><div class="vnum d" id="tof-r">--</div><div class="vstatus" id="tof-r-st"></div></div>
    </div>
    <div class="ch"><canvas id="chart-tof"></canvas></div>
  </div>

  <div class="card">
    <h2>Accelerometer 1 (mg)</h2>
    <div class="vals">
      <div class="vb"><div class="vlabel">X</div><div class="vnum x" id="a1x">--</div></div>
      <div class="vb"><div class="vlabel">Y</div><div class="vnum y" id="a1y">--</div></div>
      <div class="vb"><div class="vlabel">Z</div><div class="vnum z" id="a1z">--</div></div>
    </div>
    <div class="ch"><canvas id="chart-accel1"></canvas></div>
  </div>

  <div class="card">
    <h2>Gyroscope (dps)</h2>
    <div class="vals">
      <div class="vb"><div class="vlabel">X</div><div class="vnum x" id="gx">--</div></div>
      <div class="vb"><div class="vlabel">Y</div><div class="vnum y" id="gy">--</div></div>
      <div class="vb"><div class="vlabel">Z</div><div class="vnum z" id="gz">--</div></div>
    </div>
    <div class="ch"><canvas id="chart-gyro"></canvas></div>
  </div>

  <div class="card">
    <h2>Accelerometer 2 (mg)</h2>
    <div class="vals">
      <div class="vb"><div class="vlabel">X</div><div class="vnum x" id="a2x">--</div></div>
      <div class="vb"><div class="vlabel">Y</div><div class="vnum y" id="a2y">--</div></div>
      <div class="vb"><div class="vlabel">Z</div><div class="vnum z" id="a2z">--</div></div>
    </div>
    <div class="ch"><canvas id="chart-accel2"></canvas></div>
  </div>

  <div class="card">
    <h2>Magnetometer (mGauss)</h2>
    <div class="vals">
      <div class="vb"><div class="vlabel">X</div><div class="vnum x" id="mx">--</div></div>
      <div class="vb"><div class="vlabel">Y</div><div class="vnum y" id="my">--</div></div>
      <div class="vb"><div class="vlabel">Z</div><div class="vnum z" id="mz">--</div></div>
    </div>
    <div class="ch"><canvas id="chart-mag"></canvas></div>
  </div>

  <div class="card">
    <h2>Temperature (&deg;C)</h2>
    <div class="vals">
      <div class="vb"><div class="vlabel">Board</div><div class="vnum t" id="temp">--</div></div>
    </div>
    <div class="ch"><canvas id="chart-temp"></canvas></div>
  </div>

</div>
<script>
const TOF_ST = {0:'OK',1:'Sigma Fail',2:'Signal Fail',4:'Phase Fail',7:'Wrap Fail',12:'No Target'};
function stClass(s){return s===0?'ok':(s===12?'warn':'err');}
function stText(s){return TOF_ST[s]??'Status '+s;}

const N = 100;
function mkChart(id, labels, colors){
  return new Chart(document.getElementById(id).getContext('2d'),{
    type:'line',
    data:{
      labels:Array(N).fill(''),
      datasets:labels.map((l,i)=>({label:l,data:Array(N).fill(null),
        borderColor:colors[i],borderWidth:1.5,pointRadius:0,tension:0.2,fill:false}))
    },
    options:{animation:false,responsive:true,maintainAspectRatio:false,
      plugins:{legend:{display:false}},
      scales:{x:{display:false},y:{display:true,
        grid:{color:'#21262d'},
        ticks:{color:'#8b949e',font:{size:9},maxTicksLimit:4}}}}
  });
}

function setChartSeries(chart, series){
  series.forEach((arr,di)=>{
    if(!arr||!arr.length)return;
    const s=arr.slice(-N);
    chart.data.datasets[di].data=Array(N-s.length).fill(null).concat(s);
  });
  chart.update('none');
}

const charts={
  tof:   mkChart('chart-tof',   ['L','C','R'],  ['#f85149','#3fb950','#58a6ff']),
  a1:    mkChart('chart-accel1',['X','Y','Z'],  ['#f85149','#3fb950','#58a6ff']),
  gyro:  mkChart('chart-gyro',  ['X','Y','Z'],  ['#f85149','#3fb950','#58a6ff']),
  a2:    mkChart('chart-accel2',['X','Y','Z'],  ['#f85149','#3fb950','#58a6ff']),
  mag:   mkChart('chart-mag',   ['X','Y','Z'],  ['#f85149','#3fb950','#58a6ff']),
  temp:  mkChart('chart-temp',  ['Temp'],       ['#d2a679']),
};

let cfgShown=false;

async function poll(){
  try{
    const d=await(await fetch('/data')).json();
    const badge=document.getElementById('status-badge');
    badge.textContent=d.latest?'Live':'Waiting';
    badge.className='badge'+(d.latest?'':' err');
    document.getElementById('rate').textContent=d.rate_hz.toFixed(1);

    if(d.config&&!cfgShown&&Object.keys(d.config).length){
      cfgShown=true;
      const c=d.config;
      document.getElementById('config-bar').innerHTML=
        '<b>Config sent</b> &mdash; '+
        'ToF: '+(c.tof_profile||'?')+' FSR='+c.tof_fsr+' @ '+c.tof_odr+' Hz &nbsp;|&nbsp; '+
        'Accel1: &plusmn;'+c.accel1_fsr+'g @ '+c.accel1_odr+' Hz &nbsp;|&nbsp; '+
        'Gyro: &plusmn;'+c.gyro_fsr+'dps @ '+c.gyro_odr+' Hz &nbsp;|&nbsp; '+
        'Accel2: &plusmn;'+c.accel2_fsr+'g @ '+c.accel2_odr+' Hz &nbsp;|&nbsp; '+
        'Mag: '+c.mag_odr+' Hz';
    }

    const lat=d.latest;
    if(!lat)return;
    if(lat.counter!=null)document.getElementById('counter').textContent=lat.counter;
    if(lat.fastest_odr!=null)document.getElementById('fw-odr').textContent=lat.fastest_odr.toFixed(1);
    if(lat.temperature!=null){
      document.getElementById('temp-hdr').textContent=lat.temperature.toFixed(1);
      document.getElementById('temp').textContent=lat.temperature.toFixed(2);
    }
    if(lat.tof){
      const t=lat.tof;
      [['l','left'],['c','centre'],['r','right']].forEach(([s,k])=>{
        document.getElementById('tof-'+s).textContent=t[k].dist+' mm';
        const el=document.getElementById('tof-'+s+'-st');
        el.textContent=stText(t[k].status);
        el.className='vstatus '+stClass(t[k].status);
      });
    }
    const xyz=(pfx,obj)=>{if(!obj)return;
      document.getElementById(pfx+'x').textContent=obj.x;
      document.getElementById(pfx+'y').textContent=obj.y;
      document.getElementById(pfx+'z').textContent=obj.z;};
    xyz('a1',lat.accel1);xyz('g',lat.gyro);xyz('a2',lat.accel2);xyz('m',lat.mag);

    const h=d.history;
    setChartSeries(charts.tof, [h.tof_left_distance,h.tof_centre_distance,h.tof_right_distance]);
    setChartSeries(charts.a1,  [h.accel1_x,h.accel1_y,h.accel1_z]);
    setChartSeries(charts.gyro,[h.gyro_x,h.gyro_y,h.gyro_z]);
    setChartSeries(charts.a2,  [h.accel2_x,h.accel2_y,h.accel2_z]);
    setChartSeries(charts.mag, [h.mag_x,h.mag_y,h.mag_z]);
    setChartSeries(charts.temp,[h.temperature]);
  }catch(e){
    document.getElementById('status-badge').textContent='Error';
    document.getElementById('status-badge').className='badge err';
    console.error(e);
  }
}
setInterval(poll,250);
poll();
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# Flask dashboard backend
# ---------------------------------------------------------------------------
class FlaskDashboard:
    MAX_HISTORY = 100

    def __init__(self):
        self.lock = threading.Lock()
        self.latest: Optional[DataPacket] = None
        self.history: deque = deque(maxlen=self.MAX_HISTORY)
        self.packet_times: deque = deque(maxlen=200)
        self.config: dict = {}
        # Suppress per-request werkzeug logs
        logging.getLogger('werkzeug').setLevel(logging.ERROR)
        self.app = Flask(__name__)
        self._setup_routes()

    def set_config(self, cfg: dict):
        with self.lock:
            self.config = cfg

    def add_packet(self, packet: DataPacket):
        with self.lock:
            self.latest = packet
            self.history.append(packet)
            self.packet_times.append(time.time())

    def _get_rate(self) -> float:
        with self.lock:
            times = list(self.packet_times)
        if len(times) < 2:
            return 0.0
        elapsed = times[-1] - times[0]
        return 0.0 if elapsed <= 0 else (len(times) - 1) / elapsed

    def _packet_to_dict(self, p: DataPacket) -> dict:
        if isinstance(p, CombinedPacket):
            return {
                'type': 'combined', 'timestamp': p.timestamp, 'counter': p.counter,
                'tof': {
                    'left':   {'dist': p.tof_left_distance,   'ambient': p.tof_left_ambient,   'signal': p.tof_left_signal,   'status': p.tof_left_status},
                    'centre': {'dist': p.tof_centre_distance, 'ambient': p.tof_centre_ambient, 'signal': p.tof_centre_signal, 'status': p.tof_centre_status},
                    'right':  {'dist': p.tof_right_distance,  'ambient': p.tof_right_ambient,  'signal': p.tof_right_signal,  'status': p.tof_right_status},
                },
                'accel1': {'x': p.accel1_x, 'y': p.accel1_y, 'z': p.accel1_z},
                'gyro':   {'x': p.gyro_x,   'y': p.gyro_y,   'z': p.gyro_z},
                'accel2': {'x': p.accel2_x, 'y': p.accel2_y, 'z': p.accel2_z},
                'mag':    {'x': p.mag_x,    'y': p.mag_y,    'z': p.mag_z},
                'temperature': p.temperature,
                'fastest_odr': p.fastest_odr,
            }
        if isinstance(p, IKS02A1Packet):
            return {
                'type': 'imu', 'timestamp': p.timestamp, 'counter': p.counter,
                'accel1': {'x': p.accel1_x, 'y': p.accel1_y, 'z': p.accel1_z},
                'gyro':   {'x': p.gyro_x,   'y': p.gyro_y,   'z': p.gyro_z},
                'accel2': {'x': p.accel2_x, 'y': p.accel2_y, 'z': p.accel2_z},
                'mag':    {'x': p.mag_x,    'y': p.mag_y,    'z': p.mag_z},
                'temperature': p.temperature, 'fastest_odr': p.fastest_odr,
            }
        if isinstance(p, VL53L1A1Packet):
            return {
                'type': 'tof1', 'timestamp': p.timestamp, 'counter': p.counter,
                'tof': {
                    'left':   {'dist': p.tof_left_distance,   'ambient': p.tof_left_ambient,   'signal': p.tof_left_signal,   'status': p.tof_left_status},
                    'centre': {'dist': p.tof_centre_distance, 'ambient': p.tof_centre_ambient, 'signal': p.tof_centre_signal, 'status': p.tof_centre_status},
                    'right':  {'dist': p.tof_right_distance,  'ambient': p.tof_right_ambient,  'signal': p.tof_right_signal,  'status': p.tof_right_status},
                },
            }
        if isinstance(p, VL53L8A1Packet):
            return {'type': 'tof8', 'timestamp': p.timestamp, 'counter': p.counter,
                    'tof_matrix': p.tof_distance_matrix.tolist()}
        return {'type': 'unknown'}

    def _setup_routes(self):
        app = self.app
        db = self

        @app.route('/')
        def index():
            return Response(HTML_DASHBOARD, mimetype='text/html')

        @app.route('/data')
        def data():
            with db.lock:
                latest = db.latest
                history = list(db.history)
                config = dict(db.config)
            rate = db._get_rate()

            def imu(attr):
                return [getattr(p, attr) for p in history if isinstance(p, (CombinedPacket, IKS02A1Packet))]

            def tof(attr):
                return [getattr(p, attr) for p in history if isinstance(p, (CombinedPacket, VL53L1A1Packet))]

            return jsonify({
                'rate_hz': round(rate, 1),
                'config': config,
                'latest': db._packet_to_dict(latest) if latest else None,
                'history': {
                    'tof_left_distance':   tof('tof_left_distance'),
                    'tof_centre_distance': tof('tof_centre_distance'),
                    'tof_right_distance':  tof('tof_right_distance'),
                    'accel1_x': imu('accel1_x'), 'accel1_y': imu('accel1_y'), 'accel1_z': imu('accel1_z'),
                    'gyro_x':   imu('gyro_x'),   'gyro_y':   imu('gyro_y'),   'gyro_z':   imu('gyro_z'),
                    'accel2_x': imu('accel2_x'), 'accel2_y': imu('accel2_y'), 'accel2_z': imu('accel2_z'),
                    'mag_x':    imu('mag_x'),    'mag_y':    imu('mag_y'),    'mag_z':    imu('mag_z'),
                    'temperature': imu('temperature'),
                },
            })

    def start(self, host: str = '127.0.0.1', port: int = 10000):
        url = f'http://{host}:{port}'
        print(f"Dashboard: {url}")
        threading.Timer(1.2, lambda: webbrowser.open(url)).start()
        self.app.run(host=host, port=port, debug=False, use_reloader=False, threaded=True)


# ---------------------------------------------------------------------------
# VCP monitor (serial reader + dashboard orchestrator)
# ---------------------------------------------------------------------------
class VCPMonitor:
    """Main class for monitoring VCP data"""

    def __init__(self, port: str, baudrate: int = 1843200, preset: str = "iks02a1-vl53l1a1-both",
                 verbose: bool = False, web_port: int = 10000):
        self.port = port
        self.baudrate = baudrate
        self.preset = preset
        self.verbose = verbose
        self.web_port = web_port
        self.ser = None
        self.parser = VCPDataParser(self.verbose)
        self.dashboard = FlaskDashboard()
        self.running = False
        self.read_thread = None

    def send_initial_calibration(self):
        """Send startup calibration/config frame required by current firmware preset."""
        if not self.ser or self.preset != "iks02a1-vl53l1a1-both":
            return

        # Config values — must match main.c receivedFromSimulink() byte layout:
        # [tof_fsr(i32), tof_odr(i32), accel1_fsr(i32), accel1_odr(f32),
        #  gyro_fsr(i32), gyro_odr(f32), accel2_fsr(i32), accel2_odr(f32), mag_odr(f32)]
        tof_fsr    = 2       # ranging profile: 1=short range, 2=long range
        tof_odr    = 10      # Hz
        accel1_fsr = 4       # ±g
        accel1_odr = 104.0   # Hz
        gyro_fsr   = 1000    # ±dps
        gyro_odr   = 104.0   # Hz
        accel2_fsr = 4       # ±g
        accel2_odr = 104.0   # Hz
        mag_odr    = 100.0   # Hz

        payload = struct.pack(
            '<iiifififf',
            tof_fsr, tof_odr,
            accel1_fsr, accel1_odr,
            gyro_fsr, gyro_odr,
            accel2_fsr, accel2_odr,
            mag_odr,
        )
        packet = b'J_A' + payload + b'A_J'
        self.ser.write(packet)
        self.ser.flush()
        if self.verbose:
            print(f"TX: {packet.hex()}")

        tof_profile = "Short Range" if tof_fsr == 1 else "Long Range"
        print("Calibration packet sent — configuration:")
        print(f"  ToF:    Profile={tof_profile} (FSR={tof_fsr})   ODR={tof_odr} Hz")
        print(f"  Accel1: FSR=\u00b1{accel1_fsr}g   ODR={accel1_odr:.1f} Hz")
        print(f"  Gyro:   FSR=\u00b1{gyro_fsr} dps   ODR={gyro_odr:.1f} Hz")
        print(f"  Accel2: FSR=\u00b1{accel2_fsr}g   ODR={accel2_odr:.1f} Hz")
        print(f"  Mag:    ODR={mag_odr:.1f} Hz")

        self.dashboard.set_config({
            'tof_fsr': tof_fsr, 'tof_odr': tof_odr, 'tof_profile': tof_profile,
            'accel1_fsr': accel1_fsr, 'accel1_odr': accel1_odr,
            'gyro_fsr': gyro_fsr, 'gyro_odr': gyro_odr,
            'accel2_fsr': accel2_fsr, 'accel2_odr': accel2_odr,
            'mag_odr': mag_odr,
        })

    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            self.send_initial_calibration()
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
                    if self.verbose:
                        print(f"RX: {data.hex()}")
                    packets = self.parser.add_data(data)
                    for packet in packets:
                        self.dashboard.add_packet(packet)
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

        self.dashboard.start(port=self.web_port)
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
        description='VCP Data Parser and Dashboard for AP_01 Sensor Board'
    )
    parser.add_argument('--port', type=str, help='Serial port (e.g., COM3, /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=1843200, help='Baud rate (default: 1843200)')
    parser.add_argument('--list-ports', action='store_true', help='List available COM ports and exit')
    parser.add_argument('--preset', type=str, default='iks02a1-vl53l1a1-both',
                        choices=['iks02a1-vl53l1a1-both', 'auto'],
                        help='Firmware preset (default: iks02a1-vl53l1a1-both)')
    parser.add_argument('--web-port', type=int, default=10000,
                        help='Flask dashboard port (default: 10000)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')

    args = parser.parse_args()

    if args.list_ports:
        list_ports()
        return

    if not args.port:
        print("Error: --port is required. Use --list-ports to see available ports.")
        sys.exit(1)

    monitor = VCPMonitor(args.port, args.baud, args.preset, args.verbose, args.web_port)
    try:
        monitor.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        monitor.stop()


if __name__ == '__main__':
    main()

