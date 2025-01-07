# UCL Sensor Board Repository

This repository contains all moddels and source code and guides for the UCL Sensor Board Teaching Tool.

Microcontroller: STM32 Nucleo F411RE

Supported expansion boards: IKS02A1, VL53L1, VL53L8

---

### [Device Drivers/](https://github.com/JesseJabezArendse/AP_01/tree/main/driver%20installation)
This directory contains the installers for the drivers needed to use the STLink, onboard the Nucleo Board. Ensure this is installed first.

### [Source Code/](https://github.com/JesseJabezArendse/AP_01/tree/main/src)
This directory contains C Code for each sensor board that does the data aquisition and porting to Simulink. Not necessary unless you're curious and want to make custom changes.

### [Binaries/](https://github.com/JesseJabezArendse/AP_01/tree/main/binaries)
This directory contains the precompiled binaries of the source code folders, which is needed by Simulink.

---

## How to use the Simulink Models

First, clone this repository on your own machine, in your own working directory using the following command line:

$ git clone https://github.com/JesseJabezArendse/AP_01.git

or download the repo as a .zip



### For Windows:
   It's plug and play :)

### For Linux 
   Hopefully plug and play (limited USB device access with a Virtual Machine, apologies)
   
   If not you'll be prompted to choose the COM Port for the Nucleo Board - one of the /dev/ttyACM's (you probably want to check which one appears when you plug it in before selecting it)
   
### For Mac: (plug and play funtionality pending)
   - Copy the .bin file of the model you are wanting to run into the F411 mass storage drive
   - Create a variable called STLINK_COMPORT that is the string of the comport of the Nucleo Board
   - Run the model
