
#include <Wire.h>

const int accelerometer_address = 0x53; // is this 0x53?
const int magnetometer_address = 0x1E; // is this 0x3C?
const int gyro_address = 0x68; //(if logic is low)
//const int gyro_address = 0x69; //(if logic is high)

float accelVals[3], magVals[3], gyroVals[3];

const float GYRO_SENSITIVITY = 14.375;

const byte GYRO_REG_X_H=0x1D;
const byte GYRO_REG_X_L=0x1E;
const byte GYRO_REG_Y_H=0x1F;
const byte GYRO_REG_Y_L=0x20;
const byte GYRO_REG_Z_H=0x21;
const byte GYRO_REG_Z_L=0x22;

float gyroOffsets[3];

float magx_scale, magy_scale, magz_scale, magx_max, magy_max, magz_max;

const byte MAG_X_H=0x03;
const byte MAG_X_L=0x04;
const byte MAG_Y_H=0x07;
const byte MAG_Y_L=0x08;
const byte MAG_Z_H=0x05;
const byte MAG_Z_L=0x06;

#define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3 used with the Wire library
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

// HMC58X3 register map. For details see HMC58X3 datasheet
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_R_XM 3
#define HMC58X3_R_XL 4

#define INTCFG_ITG_RDY_EN         0x04  // 00000100
#define INTCFG_RAW_RDY_EN         0x01  // 00000001
#define PWRMGM_CLK_SEL            0x07  // 00000111
#define BW256_SR8           0
#define RANGE2000           3   // default
#define NOSRDIVIDER         0 
#define PLL_XGYRO_REF       1

#define INT_STATUS         0x1A  // R	Interrupt: Status
#define GYRO_XOUT          0x1D  // R	SENSOR: Gyro X 2bytes  

#define GYRO_YOUT          0x1F  // R	SENSOR: Gyro Y 2bytes

#define GYRO_ZOUT          0x21  // R	SENSOR: Gyro Z 2bytes

#define DLPF_FS            0x16
#define INT_CFG            0x17
#define DLPFFS_FS_SEL             0x18  // 00011000
#define DLPFFS_DLPF_CFG           0x07  // 00000111
#define PWR_MGM            0x3E  // RW	Power Management

const byte ACCEL_REG=0x32;

void setup() 
{

  Serial.begin(57600);
  Serial.println(" serial ok ");
  Wire.begin();


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
  // deactivate internal pull-ups for twi
  // as per note from atmega8 manual pg167
  cbi(PORTC, 4);
  cbi(PORTC, 5);
#else
  // deactivate internal pull-ups for twi
  // as per note from atmega128 manual pg204
  cbi(PORTD, 0);
  cbi(PORTD, 1);
#endif

  //Turning on the ADXL345
  writemem(accelerometer_address, 0x2D, 0);      
  writemem(accelerometer_address, 0x2D, 16);
  writemem(accelerometer_address, 0x2D, 8);

  //magnetometer startup
  initMag();
  calibrateMag(1);

  //turning on the gyro  
  initITG();


}


void initITG()
{

  uint8_t buff;
  
  // fs range
  readmem(gyro_address, DLPF_FS, 1, &buff);   
  writemem(gyro_address, DLPF_FS, ((buff & ~DLPFFS_FS_SEL) | (3 << 3)) ); 
  // filter
  readmem(gyro_address, DLPF_FS, 1, &buff);
  writemem(gyro_address, DLPF_FS, ((buff & ~DLPFFS_DLPF_CFG) | BW256_SR8)); 
  //clock
  readmem(gyro_address, PWR_MGM, 1, &buff);
  writemem(gyro_address, PWR_MGM, ((buff & ~PWRMGM_CLK_SEL) | PLL_XGYRO_REF)); 
  //itg ready
  readmem(gyro_address, INT_CFG, 1, &buff);
  writemem(gyro_address, INT_CFG, ((buff & ~INTCFG_ITG_RDY_EN) | 1 << 2)); 
  //raw ready
  readmem(gyro_address, INT_CFG, 1, &buff);
  writemem(gyro_address, INT_CFG, ((buff & ~INTCFG_RAW_RDY_EN) | 1)); 
  //delay
  delay(70); // time to delay
  
  zeroCalibrate(2500, 2);
  
}

void initMag()
{
  writemem(magnetometer_address, HMC58X3_R_CONFA, 0x70);
  writemem(magnetometer_address, HMC58X3_R_CONFB, 0xA0);
  writemem(magnetometer_address, HMC58X3_R_MODE, 0x00);
  
}

void calibrateMag(unsigned char gain) {
  magx_scale=1; // get actual values
  magy_scale=1;
  magz_scale=1;
  writemem(magnetometer_address, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias
  
  // set the gain
  writemem(magnetometer_address, HMC58X3_R_CONFB, gain << 5);
  
  // now iniit
  float x, y, z, mx=0, my=0, mz=0, t=10;
  
  byte buff[6];
  
  for (int i=0; i<(int)t; i++) { 
    writemem(magnetometer_address, HMC58X3_R_MODE, 1); // calibration mode
    delay(100);

    readFromMagnet(&buff[0]);
    
    if (magVals[0] > mx) mx = x;
    if (magVals[2] > my) my = y;
    if (magVals[1] > mz) mz = z;
  }
  
  float max=0;
  if (mx>max) max=mx;
  if (my>max) max=my;
  if (mz>max) max=mz;
  
  magx_max = mx;
  magy_max = my;
  magz_max = mz;
  magx_scale = max/mx; // calc scales
  magy_scale = max/my;
  magz_scale = max/mz;
  writemem(magnetometer_address, HMC58X3_R_CONFA, 0x010); // set RegA/DOR back to default
  delay(10);
  // now set mode
  writemem(magnetometer_address, HMC58X3_R_MODE, 0);
  delay(100);
}

void writemem(uint8_t dev_address, uint8_t _addr, uint8_t _val) {

  Wire.beginTransmission(dev_address);   // start transmission to device 
  Wire.send(_addr); // send register address
  Wire.send(_val); // send value to writemem
  Wire.endTransmission(); // end transmission

}

void zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) {

  byte xyz[6]; 
  float tmpOffsets[] = {0,0,0};

  for (int i = 0;i < totSamples;i++){
    delay(sampleDelayMS);
    readFromGyro(xyz);
    tmpOffsets[0] += gyroVals[0];
    tmpOffsets[1] += gyroVals[1];
    tmpOffsets[2] += gyroVals[2];  
  }
  
  gyroOffsets[0] = -tmpOffsets[0] / totSamples;
  gyroOffsets[1] = -tmpOffsets[1] / totSamples;
  gyroOffsets[2] = -tmpOffsets[2] / totSamples;

}

void readmem(uint8_t dev_address, uint8_t _addr, uint8_t _nbytes, uint8_t _buff[]) {

  Wire.beginTransmission(dev_address); // start transmission to device 
  Wire.send(_addr); // sends register address to read from
  Wire.endTransmission(); // end transmission

  Wire.beginTransmission(dev_address); // start transmission to device 
  Wire.requestFrom(dev_address, _nbytes);// send data n-bytes read
  uint8_t i = 0; 
  while (Wire.available()) {
    _buff[i] = Wire.receive(); // receive DATA
    i++;
  }
  Wire.endTransmission(); // end transmission
}


void readFromAccel(byte *buff)
{
  //readFrom(accelerometer_address, ACCEL, TO_READ, *buff); //read the acceleration data from the ADXL345

  Wire.beginTransmission(accelerometer_address); //start transmission to device 
  Wire.send(ACCEL_REG);        //sends address to read from
  Wire.endTransmission(); //end transmission


    Wire.beginTransmission(accelerometer_address); //start transmission to device (initiate again)
  Wire.requestFrom(accelerometer_address, 6);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.receive(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission

    //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  //thus we are converting both bytes in to one int
  accelVals[0] = (((int)buff[1]) << 8) | buff[0];   
  accelVals[1] = (((int)buff[3])<< 8) | buff[2];
  accelVals[2] = (((int)buff[5]) << 8) | buff[4];
}

void readFromMagnet(byte* buff)
{
  /*Wire.beginTransmission(magnetometer_address);
  Wire.send(HMC58X3_R_XM); // will start from DATA X MSB and fetch all the others
  Wire.endTransmission();
  
  Wire.beginTransmission(magnetometer_address);
  Wire.requestFrom(magnetometer_address, 6);
  if(6 == Wire.available()) {
    // read out the 3 values, 2 bytes each.
    magVals[0] = (Wire.receive() << 8) | Wire.receive();
    magVals[1] = (Wire.receive() << 8) | Wire.receive();
    magVals[2] = (Wire.receive() << 8) | Wire.receive();
  }
  Wire.endTransmission();*/
  
  readmem(magnetometer_address, HMC58X3_R_XM, 6, buff);
  
  Serial.print(buff[0], DEC);
  Serial.print(buff[1], DEC);
  Serial.print(buff[2], DEC);
  Serial.print(buff[3], DEC);
  Serial.print(buff[4], DEC);
  Serial.print(buff[5], DEC);
  Serial.println("");
  
  magVals[0] = ((buff[0] << 8) | buff[1]);
  magVals[2] = ((buff[2] << 8) | buff[3]); 
  magVals[1] = ((buff[4] << 8) | buff[5]);

  
  /*magVals[0] = (float) (buff[0] | (buff[1]<<8)); 
  magVals[1] = (float) (buff[2] | (buff[3]<<8)); 
  magVals[2] = (float) (buff[4] | (buff[5]<<8)); */

}

void readFromGyro(byte* buff)
{
  
  readmem(gyro_address, GYRO_XOUT, 6, buff);

  gyroVals[0] = ((buff[0] << 8) | buff[1]);
  gyroVals[1] = ((buff[2] << 8) | buff[3]); 
  gyroVals[2] = ((buff[4] << 8) | buff[5]);
}

void debugWrite()
{
  Serial.print('g');
  Serial.print(gyroVals[0]);
  Serial.print(" "); 
  Serial.print(gyroVals[1]); 
  Serial.print(" ");
  Serial.print(gyroVals[2]);
  Serial.println("");
  Serial.print('m'); 
  Serial.print(magVals[0]);
  Serial.print(" ");
  Serial.print(magVals[1]); 
  Serial.print(" ");
  Serial.print(magVals[2]);
  Serial.println("");
  Serial.print('a'); 
  Serial.print(accelVals[0]); 
  Serial.print(" ");
  Serial.print(accelVals[1]); 
  Serial.print(" ");
  Serial.print(accelVals[2]);
  Serial.println("");

}

void loop()
{

  byte buffer[6];

  readFromAccel(&buffer[0]);
  readFromGyro(&buffer[0]);
  readFromMagnet(&buffer[0]);
  
  delay(10);

  //debugWrite();

}



