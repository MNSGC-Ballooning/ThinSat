/*
Project: Thin Sat
Authors: Joe Poeppel & Andrew Van Gerpen
Team: MNSGC - Ballooning
*/

// -------Libraries and Definitions-------

#include "simpletools.h"
#include "lsm9ds1.h"
#include "fdserial.h" 

#define LTF 	11
#define LED1 	20
#define LED2 	21
#define PL_BUSY 24
#define PL_RX 25
#define PL_TX 26
#define IMU_EN 	27
    
// -------Global Variables and Objects-------

float __imuX, __imuY, __imuZ, __compI;
const int BYTES_PER_PACKET = 35;
const char ACK[] = {0xAA, 0x05, 0x00};
const int BAUD = 38400;
const int __Mvref = 3300;

uint16_t packetNum = 0x0001;
uint8_t sensorPacket[35];
const uint8_t SENS_PACKET_ID = 0xAA;

serial *payloadSerial;    

// -------Function Declarations------- 

void unknownSensor(); //Replace this function with whatever sensor code we want to add

void blink_leds();
void readGyro();
void readAccel();
void readMag();
void readLightSensors();
void readTemp();
void initSerial();
void sendHeader();
void sendDataPacket();
int isACK();
int isBusy();
int read_mcp320x(int __McsPin, int __MclkPin, int __MdoPin, int __MdiPin, int __Mbits, int __Mdata, int __MVr, int __Mres);             
void writePacket(uint8_t* packet);
void twoByteFormat(uint16_t val, uint8_t* dataArray, int highByteLoc, int lowByteLoc);
void printData();

// -------Main Program-------

int main()                                    
{
  print("Initalizing sensors...");
  high(LED1);
  high(IMU_EN);
  initSerial();
  while(imu_init(8, 9, 6, 5) == 0){
    pause(100);
  }      
  for(int i=0; i<=BYTES_PER_PACKET; i++){
    sensorPacket[i] = 0x5A; 
  }    
  pause(1000);
  low(LED1);
  print("done\n\n");


 
  while(1)
  {
    blink_leds();
    readGyro();
    readAccel();
    readMag();
    readLightSensors();
    readTemp();
    unknownSensor();
    sendDataPacket();
    printData();
  }  
}


// -------Functions-------

void initSerial(){
  payloadSerial = fdserial_open(PL_RX, PL_TX, 0b0000, BAUD);
}

void sendHeader(){
  fdserial_rxFlush(payloadSerial);
  fdserial_txFlush(payloadSerial);
  
  fdserial_txChar(payloadSerial, 0x50);
  fdserial_txChar(payloadSerial, 0x50);
  fdserial_txChar(payloadSerial, 0x50);
}

void blink_leds() {
  high(LED1);
  pause(200);
  low(LED1);
  pause(200);
}

int read_mcp320x(int __McsPin, int __MclkPin, int __MdoPin, int __MdiPin, int __Mbits, int __Mdata, int __MVr, int __Mres) {
  high(__McsPin);  low(__MclkPin);  low(__McsPin);
  shift_out(__MdiPin, __MclkPin, MSBFIRST, __Mbits, __Mdata);
  int __Mvolts = shift_in(__MdoPin, __MclkPin, MSBPOST, __Mres);
  high(__McsPin);  high(__MclkPin);
  return ((__Mvolts * __MVr) / pow(2,__Mres));
}

void readAccel() {
  high(IMU_EN);
  pause(100);
  imu_readAccelCalculated(&__imuX, &__imuY, &__imuZ);
  int16_t imuX = (int) (100.0 * __imuX);
  int16_t imuY = (int) (100.0 * __imuY);
  int16_t imuZ = (int) (100.0 * __imuZ);

  twoByteFormat(imuX, sensorPacket, 23, 24);
  twoByteFormat(imuY, sensorPacket, 25, 26);
  twoByteFormat(imuZ, sensorPacket, 27, 28);
}

void readGyro(){
  high(IMU_EN);
  pause(100);  
  imu_readGyroCalculated(&__imuX, &__imuY, &__imuZ);
  int16_t imuX = (int) (100.0 * __imuX);
  int16_t imuY = (int) (100.0 * __imuY);
  int16_t imuZ = (int) (100.0 * __imuZ);
    
  twoByteFormat(imuX, sensorPacket, 17, 18);
  twoByteFormat(imuY, sensorPacket, 19, 20);
  twoByteFormat(imuZ, sensorPacket, 21, 22);
}  
  
void readMag(){
  high(IMU_EN);
  pause(100);  
  imu_readMagCalculated(&__imuX, &__imuY, &__imuZ);
  int16_t imuX = (int) (100.0 * __imuX);
  int16_t imuY = (int) (100.0 * __imuY);
  int16_t imuZ = (int) (100.0 * __imuZ);
  
  twoByteFormat(imuX, sensorPacket, 29, 30);
  twoByteFormat(imuY, sensorPacket, 31, 32);
  twoByteFormat(imuZ, sensorPacket, 33, 34);
}
  
  void readLightSensors() {
  uint16_t uvaVal = read_mcp320x(7, 8, 9, 10, 6, 0b111100, __Mvref, 12);
  uint16_t uvcVal = read_mcp320x(7, 8, 9, 10, 6, 0b111110, __Mvref, 12);
  uint16_t phototranVal = read_mcp320x(7, 8, 9, 10, 6, 0b110100, __Mvref, 12);
  uint16_t ltfVal = count(LTF, 10);
  
  twoByteFormat(uvaVal, sensorPacket, 3, 4);
  twoByteFormat(uvcVal, sensorPacket, 5, 6);
  twoByteFormat(phototranVal, sensorPacket, 7, 8);
}

void readTemp() {
  int16_t temp0, temp1, temp2, temp3;
  
  temp1 = read_mcp320x(7, 8, 9, 10, 6, 0b110110, __Mvref, 12);
  temp2 = read_mcp320x(7, 8, 9, 10, 6, 0b111000, __Mvref, 12);
  temp3 = read_mcp320x(7, 8, 9, 10, 6, 0b111010, __Mvref, 12);

  twoByteFormat(temp1, sensorPacket, 9, 10);
  twoByteFormat(temp2, sensorPacket, 11, 12);
  twoByteFormat(temp3, sensorPacket, 13, 14);
}

void unknownSensor() {
/*
Particle Counter Function would be here, but don't have this sensor 
so we will have two extra bytes available within sensorPacket to 
use with another sensor. The bytes will be in elements 15 and 16 of
sensorPacket. For now, setting these elements to 0.
*/

sensorPacket[15] = 0;
sensorPacket[16] = 0;

}

int isBusy(){
  return input(PL_BUSY);
}

int isACK(){
  int numACKBytes = 3;
  char response[numACKBytes];
  for(int i=0; i<numACKBytes; ++i){
    int safteyEscape = 0;
    while(fdserial_rxCount(payloadSerial) == 0){
      if(safteyEscape++ > 1500){
        high(LED1);
        high(LED2);
        pause(500);
        low(LED1);
        low(LED2);
        return 0;
      }
      pause(10);
    }
    response[i] = fdserial_rxChar(payloadSerial);
  }
  int valid = 1;
  for(int i = 0; i<numACKBytes; ++i){
    if(ACK[i] != response[i]){
      valid = 0;
    }
  }
  return valid;
}

void sendDataPacket(){
    high(LED2);
   if(!isBusy()){
     high(LED2);
     sendHeader();
       sensorPacket[0] = SENS_PACKET_ID;
       twoByteFormat(packetNum, sensorPacket, 1, 2);
       writePacket(sensorPacket);  
     }                                         
     
     /* UNCOMMENT WHEN HOOKED UP TO EM --- COMMENTED FOR TESTING PURPOSES
     if(isACK()){    
       print("ack\n\n");
       packetNum++;
     }
     else{
       print("NAK\n");
     }
     */
     packetNum++; //COMMENT WHEN HOOKED UP TO EM --- USED FOR TESTING PURPOSES
             
     low(LED1);
     low(LED2);     
}

void twoByteFormat(uint16_t val, uint8_t* dataArray, int highByteLoc, int lowByteLoc){
   dataArray[highByteLoc] = (uint8_t)(val >> 8);
   dataArray[lowByteLoc] = (uint8_t)(val & 0x00FF);
}  

void writePacket(uint8_t* packet){
  for(int i=0; i < BYTES_PER_PACKET; i++){
    fdserial_txChar(payloadSerial, packet[i]);
  }
}

void printData() {
  
  for(int i = 0; i<BYTES_PER_PACKET; i++) {
    print("%x,",sensorPacket[i]); //Prints data to monitor as hex
  }
  
  print("%s", "\n");
  
  for(int i = 0; i<BYTES_PER_PACKET; i++) {    
  print("%d,",sensorPacket[i]); //Prints data to monitor as decimal
  }  
    
  print("%s", "\n");
  print("%s", "\n");
}    