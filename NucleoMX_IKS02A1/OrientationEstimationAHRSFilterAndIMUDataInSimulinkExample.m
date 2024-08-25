%% Estimate Orientation Using AHRS Filter and IMU Data in Simulink
% This example shows how to stream IMU data from sensors connected to Arduino(R) board and estimate orientation using AHRS filter and IMU sensor.

% Copyright 2022 The MathWorks, Inc.

%% Required MathWorks Products
%
% * MATLAB(R)
%
% * Simulink(R)
%
% * Simulink support Package for Arduino Hardware
%
% * Either Navigation Toolbox(TM) or Sensor Fusion and Tracking Toolbox(TM) 
%
%% Hardware Required
% *1.* Any of the Arduino board given below: 
%
% * Arduino Leonardo
%
% * Arduino Mega 2560
%
% * Arduino Mega ADK
%
% * Arduino Micro
%
% * Arduino Nano 3.0
%
% * Arduino Uno
%
% * Arduino Due
%
% * Arduino MKR1000
%
% * Arduino MKR WIFI 1010
%
% * Arduino MKR ZERO
%
% * Arduino Nano 33 IoT
%
% * Arduino Nano 33 BLE Sense
% 
% *2.* IMU sensor with accelerometer, gyroscope, and magnetometer. In this example, 
% <https://www.st.com/en/ecosystems/x-nucleo-iks01a2.html X-NUCLEO-IKS01A2> sensor expansion board is used. 
% The LSM6DSL sensor on the expansion board is used to get acceleration and angular rate values. 
% The LSM303AGR sensor on the expansion board is used to get magnetic field value. The sensor data can be read using I2C protocol.
% 
%% Hardware Connection 
% X-NUCLEO-IKS01A2 shield comes with Arduino UNO connectors, which makes it easy to interface with UNO board. 
% For other boards, connect the SDA, SCL, 3.3V, and GND pin of the Arduino board to the respective pins on the sensor shield.
%
%% Hardware Configuration in the model
% The example uses two models, <matlab:open_system('AnalyseIMUData') AnalyseIMUData.slx> 
% and <matlab:open_system('EstimateOrientationUsingAHRSandIMU') EstimateOrientationUsingAHRSandIMU.slx>. 
% Both the models are preconfigured to work with Arduino UNO. If you are using a different Arduino board, 
% change the hardware board by doing the following steps: 
%
% *1.* Click *Hardware Settings* in the *Hardware* tab of the Simulink toolbar.
%
% *2.* In the Configurations Parameters dialog box, select *Hardware Implementation*.
%
% *3.* From the *Hardware board* list, select the type of Arduino board that you are using.
%
% *4.* Click *Apply*. Click *OK* to close the dialog box.
% 
%% Task 1 - Read and Calibrate Sensor Values
% This section describes how to read and calibrate the sensor values for the orientation estimation algorithm. 
% To read and analyze values, use the model <matlab:open_system('AnalyseIMUData') AnalyseIMUData.slx>.
%%
open_system('AnalyseIMUData.slx');
%%
% *Reading acceleration and angular rate from LSM6DSL Sensor*
%
% Simulink Support Package for Arduino Hardware provides <docid:arduino_ref#mw_0cb96b39-313b-4258-8c09-ecda34fa58bc LSM6DSL IMU Sensor> 
% block to read acceleration and angular rate along the X, Y and Z axis 
% from LSM6DSL sensor connected to Arduino. 
% The block outputs acceleration in m/s2 and angular rate in rad/s. The sensor can be further configured by selecting the options given on the block mask. 
%  
% *Reading sensor values form LSM303AGR*
%
% To read magnetic field values using LSM303AGR sensor, the example uses 
% <docid:arduino_ref#bvp4cb8-26> and <docid:arduino_ref#bvp4cb8-21> blocks in the Support Package.
%
% The I2C Address of the LSM303AGR sensor is 0x1E. This address is specified as _Slave address_ in I2C Read/I2C Write block 
% that is used to configure and read the magnetic field value from the sensor.
%
% Depending on the Output Data Rate (ODR) required, a value needs to be written to CFG_REG_A_M register (0x60) of the sensor. 
% To see the available ODRs,  refer the <https://www.st.com/resource/en/datasheet/lsm303agr.pdf LSM303AGR datasheet>.  
% This is a one-time operation required to initialize the sensor, and it is done using the 
% <docid:simulink_ref#bva2afv-1> block in the model. 
%
% The magnetic field is read from the output registers (0x68 - 0x6D) of the sensor using the I2C Read block and it is converted to 
% microtesla as required by the example.
%
% <<../magnetometer-i2c-read-write.png>>
%  
% *Perform Magnetometer calibration*
%
% To get accurate measurements from the sensor, the sensor need to be calibrated. In this section, we consider magnetometer calibration 
% for compensating hard iron distortions. Hard iron effects are stationary interfering magnetic noise sources. 
% Often, these come from other metallic objects on the circuit board with the magnetometer. 
% These distortions can be corrected by subtracting the correction value from the magnetometer readings for each axis. 
% 
% To find the correction values, do the following:
%
% *1.* Open the model _AnalyseIMUData_. The model uses the _To workspace_ block (|out.MagneticField| in the model) 
% to log magnetometer data.
%
% The model is already configured to run in *Connected IO* mode. The simulation using Connected IO allows you to run your 
% algorithm in Simulink 
% with peripheral data from the hardware. For more details, refer to
% <docid:arduino_ug#mw_3c4745a2-f09e-4c87-83ca-6577d9b1288c>.
%
% *2.* Run Connected IO by clicking the Run button corresponding to *Run
% with IO* under the *Hardware* tab.
%
% <<../run-with-io-tab.png>>
%  
% *3.* While the model is running, rotate the sensor from 0 to 360 degree along each axis. 
% 
% *4.* Click the Stop button to stop the Connected IO Simulation
% 
% *6.* The Magnetic field values are logged in the MATLAB base workspace as *out.MagneticField* variable. 
% Use the <docid:fusion_ref#function_magcal> function on the logged values in 
% MATLAB command window to obtain the correction coefficients.
% 
%  [softIronFactor, hardIronOffset] = magcal(out.MagneticField);
% 
% *Note*: The correction values change with the surroundings.
% 
%% Task 2. Fuse Sensor Data with AHRS Filter
% This section describes how to fuse the sensor data to estimate the orientation. Use the model 
% _EstimateOrientationUsingAHRSAndIMU_ for this section.
%%
open_system("EstimateOrientationUsingAHRSandIMU.slx");
%%
% *Sensor Blocks*
%
% The first part of the model is for reading sensor values, which is described in the previous section.  If you make changes to 
% sensor blocks in the 
% previous task, make the corresponding changes in the blocks in this model as well.
%
% *PreProcessor Block*
%
% The Preprocessor block in the model accepts acceleration, angular rate, and magnetic field from the sensor and magnetic field correction values. 
% The block outputs the calibrated and axis-aligned sensor values. 
% 
% Modify the values in the Constant block _Magnetometer correction values_, which is 
% the input to the Preprocessor block, with the correction values (|hardIronOffset|) obtained from the step 6 in 
% *Perform Magnetometer Calibration* section.
%
% The axes of the accelerometer, gyroscope, and magnetometer in the sensor may not be aligned with each other. Specify the index and sign 
% of x-, y-, and z-axis of each sensor on the PreProcessor block mask, so that the sensor is aligned with the North-East-Down (NED) coordinate 
% system when it is at rest. In this example, the magnetometer Y-axes is changed while the accelerometer and gyroscope axes remain fixed. 
% 
% *Filter Block*
%
% To estimate orientation with IMU sensor data, an <docid:nav_ref#block_ahrsfilter> block is used. 
% The AHRS block fuses accelerometer, magnetometer, and gyroscope sensor data 
% to estimate device orientation. The AHRS block has tunable parameters. Tuning the parameters based on the specified sensors being used 
% can improve performance. For more details, refer to |Tuning Filter
% Parameters| section in 
% <docid:nav_ug#mw_e5835a34-8335-47ca-ba54-eb3daf01b1c5>. 
% 
% *Visualization Block*
%
% To visualize the orientation in Simulink, this example provides a helper block, _HelperPosePlot_. The block plots the pose specified 
% by the quaternion or rotation matrix.
%  
% <<../image-helperposeplot.png>>
% 
%% Validate the Model Design Using Connected IO
% You can simulate the model in Connected IO to validate the model design
% before generating the code and deploying the model on Arduino board.
% This communication between the model and Arduino does not require any code generation or model deployment, 
% thus accelerating the simulation process. 
% For more information on Connected IO, see <docid:arduino_ug#mw_3c4745a2-f09e-4c87-83ca-6577d9b1288c>. The model is already configured 
% to run in Connected IO mode.
%
% This application requires the sensor data to be acquired in real time. To get real time data with Connected IO, you need to enable pacing. 
% To acquire real time data from hardware, do the following: 
%
% *1.* On the Simulink toolbar, click the *Simulation* tab and set the Simulation mode to *Normal*.
%
% *2.* To run this model in the Connected IO mode, click the *Hardware* 
% tab, go to the *Mode* section, and select *Connected IO*.
%
% *3.* On the *Hardware* tab, open the dropdown *Run with IO* in the *Run on
% Computer* section, and select *Simulation Pacing*.
% 
% *4.* Select *Enable pacing to slow down simulation*.
%
% <<../enable-pacing.png>>
%  
% *5.*  Click the Run icon corresponding to *Run with IO* to start the Connected IO Simulation.
%
% Move the sensor and check if the motion in the figure is matching the motion of the sensor.
%
% *6.* To stop running the model, click the Stop icon corresponding to *Run with IO*.
% 
%% Run the Model in External Mode
% After you successfully simulate the model in Connected IO, simulate the model in External mode. Unlike Connected IO, the model is 
% deployed as a C code on the hardware. The code obtains real-time data from the hardware. In external mode, the data acquisition and 
% parameter tuning are done while the application is running on the
% hardware.
%
% *Note:* Ensure that the Arduino board you are using has sufficient memory to run the application on hardware. 
% The boards like Arduino Uno, which have low memory, cannot support this application. 
%
% *Note:* The block _HelperPosePlot_ is not supported for external mode workflow. To view the orientation in external mode, 
% use other dashboards in Simulink like Scope, Display blocks, and so on. 
%
% *1.* To run External Mode, click the *Hardware* tab, go to the Mode section, select *Run on board (External mode)* and 
% then click *Monitor & Tune*.
% 
% <<../monitor-and-tune-ahrs.png>> 
% 
% The lower left corner of the model window displays status while Simulink prepares, downloads, and runs the model on the hardware.
%
% Move the sensor and verify the orientation values.
%
% *2.* To stop running the model, click Stop corresponding to *Monitor and Tune*.
%
