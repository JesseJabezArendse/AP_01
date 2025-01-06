///////////////////////////////////////////////////////////
// Project : STM32 Nucleo + IKS02A1
// Abbreviation : AP_01
// Microcontroller: STM32 Nucleo F411RE
// Made For : Dr. Amir Patel
// @author: Jesse Jabez Arendse
// @modified: 24/08/2024
///////////////////////////////////////////////////////////

#define PCM_BUFFER_SIZE 480 //bytes
#define PDM_BUFFER_SIZE (16*480) //bits




void initIKS02A1(void);
void getIKS02A1(void);
void receivedFromSimulink(uint8_t* bigBuffer);
void sendToSimulink(void);
void blueButtonPressed(void);
void setupInterrupt(void);
void calibrate_IKS02A1(void);
void sendEachToSimulink(int16_t val);
void startAudioPacket();
void sendAudioChunk();
void endAudioPacket();