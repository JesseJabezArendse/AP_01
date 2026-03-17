# VCP Data Parser - Complete Documentation Index

## 📑 Quick Navigation

### 🚀 Getting Started (Start Here!)
- **[QUICKSTART.md](QUICKSTART.md)** - Installation & first use (5 min read)
- **[README.md](README.md)** - Complete overview (15 min read)

### 🔧 Technical Documentation
- **[IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md)** - What was built (20 min read)
- **[VCP_Parser_Analysis.md](VCP_Parser_Analysis.md)** - Protocol deep dive (25 min read)
- **[UART_Protocol_Reference.md](UART_Protocol_Reference.md)** - Quick reference (10 min read)

### 💻 Scripts
- **[vcp_data_parser.py](vcp_data_parser.py)** - Main plotting script (1000+ lines)
- **[vcp_data_parser_test.py](vcp_data_parser_test.py)** - Test/debug script (600+ lines)
- **[requirements.txt](requirements.txt)** - Python dependencies

---

## 📋 Document Overview

### QUICKSTART.md
**Best for**: Getting up and running immediately

Contents:
- Installation steps
- Basic usage examples
- Troubleshooting common issues
- Hardware verification
- Advanced workflows

**Time to read**: 5-10 minutes

---

### README.md
**Best for**: Understanding the complete system

Contents:
- Overview and features
- File descriptions
- UART analysis summary
- Implementation comparison
- Usage examples
- Architecture overview

**Time to read**: 10-15 minutes

---

### IMPLEMENTATION_SUMMARY.md
**Best for**: Understanding what was created and why

Contents:
- All files created
- UART functions analysis
- Packet type detection
- Visualization features
- Data structures
- Performance metrics
- Supported configurations

**Time to read**: 15-20 minutes

---

### VCP_Parser_Analysis.md
**Best for**: Deep understanding of the protocol

Contents:
- Complete UART protocol analysis
- Data format specifications for each variant
- Byte-by-byte packet structures
- Key observations from source code
- Data integrity mechanisms
- File structure
- Contributing guide

**Time to read**: 20-25 minutes

---

### UART_Protocol_Reference.md
**Best for**: Quick reference during development

Contents:
- Summary of all implementations
- UART configuration parameters
- Detailed payload structures with byte offsets
- Performance metrics
- Error detection mechanisms
- Common issues & solutions
- References

**Time to read**: 10-15 minutes

---

## 🎯 Suggested Reading Paths

### Path 1: "I just want to see my data"
1. QUICKSTART.md (5 min)
2. Run: `python vcp_data_parser.py --list-ports`
3. Run: `python vcp_data_parser.py --port COM3`

### Path 2: "I want to understand everything"
1. README.md (15 min)
2. IMPLEMENTATION_SUMMARY.md (20 min)
3. VCP_Parser_Analysis.md (25 min)
4. Source code comments in vcp_data_parser.py

### Path 3: "I'm debugging a communication issue"
1. UART_Protocol_Reference.md (10 min)
2. vcp_data_parser_test.py --test-all
3. Check STM32 source code in `Core/Src/main.c`

### Path 4: "I want to extend the code"
1. README.md - Architecture section (5 min)
2. vcp_data_parser.py - Class structure (10 min)
3. VCP_Parser_Analysis.md - Contributing section (5 min)

---

## 📊 File Summary Table

| File | Type | Purpose | Size | Read Time |
|------|------|---------|------|-----------|
| QUICKSTART.md | Doc | Getting started | 8 KB | 5 min |
| README.md | Doc | Complete overview | 15 KB | 15 min |
| IMPLEMENTATION_SUMMARY.md | Doc | What was built | 12 KB | 20 min |
| VCP_Parser_Analysis.md | Doc | Protocol analysis | 16 KB | 25 min |
| UART_Protocol_Reference.md | Doc | Quick reference | 12 KB | 10 min |
| vcp_data_parser.py | Script | Main program | 45 KB | 30 min |
| vcp_data_parser_test.py | Script | Testing utility | 25 KB | 20 min |
| requirements.txt | Config | Dependencies | <1 KB | 1 min |

**Total documentation**: ~73 KB (read all in ~90 minutes)  
**Total code**: ~70 KB (1600+ lines)

---

## 🔍 Key Topics & Where to Find Them

### UART Configuration
- QUICKSTART.md → "Troubleshooting" section
- UART_Protocol_Reference.md → "UART Configuration" section
- VCP_Parser_Analysis.md → "UART Protocol Analysis" section

### Packet Structures
- UART_Protocol_Reference.md → Complete byte-by-byte diagrams
- IMPLEMENTATION_SUMMARY.md → Data structures section
- README.md → Packet format summary

### Usage Examples
- QUICKSTART.md → "Usage" and "Workflows" sections
- README.md → "Usage Examples" section
- vcp_data_parser.py → Help text: `python vcp_data_parser.py --help`

### Troubleshooting
- QUICKSTART.md → "Troubleshooting" section
- README.md → "Troubleshooting" section
- UART_Protocol_Reference.md → "Common Issues" section

### Testing Without Hardware
- QUICKSTART.md → "Testing Without Hardware" section
- vcp_data_parser_test.py → All test examples
- README.md → "Testing & Validation" section

### Performance Metrics
- UART_Protocol_Reference.md → "Performance Metrics" table
- IMPLEMENTATION_SUMMARY.md → "Performance" section
- README.md → "Performance Metrics" section

### Architecture & Design
- README.md → "Architecture" section
- IMPLEMENTATION_SUMMARY.md → "Design" section
- vcp_data_parser.py → Class docstrings

---

## 📞 Quick Reference Commands

```bash
# Installation
pip install -r requirements.txt

# List COM ports
python vcp_data_parser.py --list-ports

# Start monitoring (Windows)
python vcp_data_parser.py --port COM3

# Start monitoring (Linux/Mac)
python3 vcp_data_parser.py --port /dev/ttyUSB0

# Test without hardware
python vcp_data_parser_test.py --test-all

# Generate specific packet for testing
python vcp_data_parser_test.py --generate-combined

# View script help
python vcp_data_parser.py --help
```

---

## 🎓 Learning Objectives by Document

### After reading QUICKSTART.md, you can:
- ✓ Install the parser
- ✓ Find your COM port
- ✓ Start monitoring sensor data
- ✓ Troubleshoot connection issues

### After reading README.md, you can:
- ✓ Understand the complete system architecture
- ✓ Know what packet types are supported
- ✓ Explain how data flows from board to computer
- ✓ Modify script parameters for custom use

### After reading IMPLEMENTATION_SUMMARY.md, you can:
- ✓ Explain what was created and why
- ✓ Understand packet auto-detection
- ✓ Describe visualization features
- ✓ Estimate performance requirements

### After reading VCP_Parser_Analysis.md, you can:
- ✓ Understand the UART protocol completely
- ✓ Parse raw binary data manually
- ✓ Implement the protocol in another language
- ✓ Extend the code with new features

### After reading UART_Protocol_Reference.md, you can:
- ✓ Debug communication issues
- ✓ Verify packet structure
- ✓ Estimate data rates
- ✓ Understand error detection

---

## 🔗 Related Source Code

All source code files are in the workspace:

```
IKS02A1 Source Code Full/Core/Src/
  - main.c (sendToSimulink, initialCalibration)

IKS02A1 VL53L1A1 Source Code Both/Core/Src/
  - main.c (sendToSimulink)
  - IKS02A1_Simulink.c (sensor functions)
  - VL53L1A1_Simulink.c (ToF functions)

VL53L1A1 Source Code Full/Core/Src/
  - main.c (sendToSimulink)

IKS02A1 VL53L8A1 Source Code Both/Core/Src/
  - main.c (sendToSimulink)
```

---

## ✅ Checklist for First Use

- [ ] Read QUICKSTART.md (5 min)
- [ ] Install requirements: `pip install -r requirements.txt` (2 min)
- [ ] List COM ports: `python vcp_data_parser.py --list-ports` (1 min)
- [ ] Connect board via USB
- [ ] Run parser: `python vcp_data_parser.py --port COM3` (1 min)
- [ ] Verify data appears in plots
- [ ] Close parser (Ctrl+C)

**Total time to working system: ~10 minutes**

---

## 🚦 Status Indicators

### Documentation Status
- ✅ README.md - Complete
- ✅ QUICKSTART.md - Complete
- ✅ VCP_Parser_Analysis.md - Complete
- ✅ UART_Protocol_Reference.md - Complete
- ✅ IMPLEMENTATION_SUMMARY.md - Complete

### Code Status
- ✅ vcp_data_parser.py - Production ready
- ✅ vcp_data_parser_test.py - Production ready

### Supported Configurations
- ✅ IKS02A1 only
- ✅ VL53L1A1 only
- ✅ IKS02A1 + VL53L1A1 combined
- ✅ VL53L8A1 8×8 array
- ✅ Auto-detection of all types

---

## 📮 Document Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-03-17 | Initial release |

---

## 🎯 Next Steps

1. **For immediate use**: Read QUICKSTART.md and start monitoring
2. **For deep understanding**: Read all documentation in order
3. **For debugging**: Use vcp_data_parser_test.py and UART_Protocol_Reference.md
4. **For extending**: Study vcp_data_parser.py architecture and VCP_Parser_Analysis.md

---

## 📚 Additional Resources

### Python Documentation
- [struct module](https://docs.python.org/3/library/struct.html) - Byte conversion
- [matplotlib](https://matplotlib.org/) - Real-time plotting
- [pyserial](https://pyserial.readthedocs.io/) - Serial communication

### STM32 Documentation
- STM32F411 Reference Manual
- STM32 HAL UART API
- VL53L1X API Documentation

### Project Files
- All source code in workspace subdirectories
- Configuration files in various `Core/Inc` folders

---

**Last Updated**: 2025-03-17  
**Total Documentation**: ~73 KB  
**Total Code**: ~70 KB  
**Status**: ✅ Ready for Use
