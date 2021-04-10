#include "CRC.h"
#include "TMC7300_Register.h"
#include <HardwareSerial.h>
#include <Arduino.h>
#include <Wire.h> //Wire lib for I2C communication
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Adafruit_NeoPixel.h> // for the lopy RGB led control source : https://github.com/adafruit/Adafruit_NeoPixel

#define CRC8_GEN 0x07


//Hardware pins
#define EN 14
#define ad0 25
#define ad1 26
#define RX 15
#define TX 4
#define battery 36
#define encoderL1 37
#define encoderR1 38
#define encoderL2 39
#define encoderR2 35

//Hardware serial for UART communication on LoPy
HardwareSerial mySerial(1);

//MPU and Kalman object
Kalman kalmanY;
uint8_t i2cData[14]; // Buffer for I2C data
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroYangle; // Calculated angle using the gyro only
double compAngleY; // Calculated angle using a complementary filter
double kalAngleY; // Calculated angle using a Kalman filter

double pitch;

uint32_t timer;
double dt,dtPrec;

double angleerror=0;
double gyroXerror,gyroYerror,gyroZerror;





//object for automatic control
volatile double vconsigne=0;
double ev[3]={0,0,0};
double tetar[3]={0,0,0};
double etetaSum=0;

const double Kp=4843.8;
const double Kd=31050;
const double Ki=23000;
double cmd=0;

//other timers
double chrono;
uint32_t timer1;
uint32_t timer2;

// timer for calculate phase with encoder
double timerLprec;
double timerRprec;
double posL=0;
double posR=0;
double posprecL=0;
double posprecR=0;
const double R=0.021;
double phaseL=0;
double phaseR=0;
double TencodL=0;
double TencodR=0;
short int sensL,sensR;

//timer for automatic control
hw_timer_t * timerA = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool flag=false;
void IRAM_ATTR onTime();


void setup(){
  delay(100);
  //hardware pin
  pinMode(EN,OUTPUT);
  pinMode(ad0,OUTPUT);
  pinMode(ad1,OUTPUT);
  pinMode(encoderL1,INPUT);
  pinMode(encoderR1,INPUT);
  pinMode(encoderL2,INPUT);
  pinMode(encoderR2,INPUT);
  pinMode(battery,INPUT);

  //setting driver adress
  digitalWrite(ad0, HIGH);
  digitalWrite(ad1, LOW);

  //set encoder position functions
  attachInterrupt(digitalPinToInterrupt(encoderL1), encoderChangeL1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderR1), encoderChangeR1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderL2), encoderChangeL2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderR2), encoderChangeR2, RISING);

  //timer for automatic control flag
  timerA = timerBegin(0, 80, true); //prescaler : 80e6/80 = 1e6 (ESP32 : 80MHz)
  timerAttachInterrupt(timerA, &onTime, true);
  timerAlarmWrite(timerA, 10000, true); //for 100Hz : 100=1e6/1e4     
  timerAlarmEnable(timerA);
  

  //UART communication
  Serial1.begin(115200, SERIAL_8N1, RX, TX);
  //Serial communication with computer
  Serial.begin(115200);

  
  //Driver setup
  delay(500);
  uartWriteDatagram(0x00, TMC7300_SLAVECONF, 0x00000001); //SLAVEADDR to 1
  tmc7300_initMotorDrivers();
  delay(500);

  //MPU setup line 174
  Wire.begin();
  setupMPU();

  //timers init
  timer = micros();
  chrono=0;
  timer1=millis();
  timer2=millis();
  dtPrec=micros()/1e6;
  timerLprec=micros()/1e6;
  timerRprec=micros()/1e6;
  
}

void loop(){

  //Serial.print(phaseL); Serial.print("\t\t"); Serial.println(phaseR);
  //delay(200);
  if(flag){ 
    //Serial.println(millis());
    
    /* ecrire le code d'asservissement ici*/
    
    flag=false;
  }
  
  
}

void IRAM_ATTR onTime() {
   flag=true;
}

//encoder functions
void encoderChangeL1(){

  TencodL = (micros()/1e6)-timerLprec;
  timerLprec = micros()/1e6;

  posL += sensL * (1/(3*150.58));
  //Serial.println("encoder L1");
}

void encoderChangeR1(){
  TencodR = (micros()/1e6)-timerRprec;
  timerRprec = micros()/1e6;
  posR += sensR * (1/(3*150.58));
  //Serial.println("encoder R1");
}

void encoderChangeL2(){
  if (TencodL != 0)
    phaseL = 360*((micros()/1e6)-timerLprec)/TencodL;
  if (phaseL>180)
    sensL = -1;
  else
    sensL = 1;
    
  //Serial.println("encoder L2");
}

void encoderChangeR2(){
  if (TencodR != 0)
    phaseR = 360*((micros()/1e6)-timerRprec)/TencodR;
  if (phaseR<180)
    sensR = -1;
  else
    sensR = 1;
  //Serial.println("encoder R2");
}



//MPU setup funtion
void setupMPU(){
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif
  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroXerror = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroYerror = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZerror = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  //pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  angleerror= atan2(-accX, accZ) * RAD_TO_DEG;
  
  pitch = atan2(-accX, accZ) * RAD_TO_DEG-angleerror;

  kalmanY.setAngle(pitch);
  gyroYangle = pitch;
  compAngleY = pitch; 
}

void readValues(){
    /* Update all the values for MPU*/
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
}

void getAngles(){
  pitch = atan2(-accX, accZ) * RAD_TO_DEG-angleerror;
  double gyroYrate = gyroY / 131.0; // Convert to deg/s  
  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  gyroYangle += gyroYrate * dt;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  timer = micros();
}



//TMC init
void tmc7300_initMotorDrivers()
{
  digitalWrite(EN, LOW);
  uartWriteDatagram(0x01,0x00,0x01);
  uartWriteDatagram(0x01, 0x10,0x00001F01 ); // 32/32 current limit
  uartWriteDatagram(0x01, 0x6C,0x13008001 ); // CHOPCONF
  uartWriteDatagram(0x01, 0x70,0xC40D1024 ); // PWMCONF
  uartWriteDatagram(0x01,TMC7300_PWM_AB , 0x00000000);
  digitalWrite(EN, HIGH);
}

//motors control functions
void runMotors(float pwm_a,float pwm_b){
  int32_t pa=(int)(pwm_a+0.5);
  int32_t pb=(int)(pwm_b+0.5);
  long int data=((pb<<16)&0xffff0000)|(pa&0x0000ffff);
  uartWriteDatagram(0x01, TMC7300_PWM_AB, data);
}

void uartWriteDatagram(uint8_t SLAVEADDR, uint8_t registerAddress, 
      unsigned long datagram) {
  //TMC5072 takes 64 bit data: 8 sync + reserved, 8 chip address, 
  //8 register address, 32 data, 8 CRC

  uint8_t CRC = 0;
  int temp;
  unsigned char buf[8];
  CRC = NextCRC(CRC, 0x05, CRC8_GEN);
  CRC = NextCRC(CRC, SLAVEADDR, CRC8_GEN);
  CRC = NextCRC(CRC, registerAddress|0x80, CRC8_GEN);  
  CRC = NextCRC(CRC, (datagram >> 24) & 0xff, CRC8_GEN);
  CRC = NextCRC(CRC, (datagram >> 16) & 0xff, CRC8_GEN);
  CRC = NextCRC(CRC, (datagram >> 8) & 0xff, CRC8_GEN);
  CRC = NextCRC(CRC, datagram & 0xff, CRC8_GEN);
  
  buf[0] = 0x05;
  buf[1] = SLAVEADDR;
  buf[2] = registerAddress|0x80;
  buf[3] = (datagram >> 24) & 0xff;
  buf[4] = (datagram >> 16) & 0xff;
  buf[5] = (datagram >> 8) & 0xff;
  buf[6] = datagram & 0xff; 
  buf[7] = CRC;
  
  temp = Serial1.write(buf, 8); //write datagram
  //Serial1.flush();  //wait until all datas are written
  Serial1.readBytes(buf, 8); //clear input buffer
}

unsigned long uartRead(uint8_t SALVEADDR, uint8_t registerAddress) {

  uint8_t CRC = 0, temp;
  unsigned char buf[8];
  unsigned long dataBytes;

  CRC = NextCRC(CRC, 0x05, CRC8_GEN);
  CRC = NextCRC(CRC, SALVEADDR, CRC8_GEN);
  CRC = NextCRC(CRC, registerAddress, CRC8_GEN);
  buf[0] = 0x05;
  buf[1] = SALVEADDR;
  buf[2] = registerAddress;
  buf[3] = CRC;
  Serial1.write(buf, 4); //write datagram
  //Serial1.flush();  //wait until all datas are written
  Serial1.readBytes(buf, 4); //clear input buffer
  
  Serial1.readBytes(buf, 8);
  temp = buf[2];
  dataBytes = buf[3]; //bit 32...24
  dataBytes <<= 8;
  dataBytes |= buf[4]; //bit 23...16
  dataBytes <<= 8;
  dataBytes |= buf[5]; //bit 15...8
  dataBytes <<= 8;
  dataBytes |= buf[6]; //bit 7...0
  
  CRC = 0;
  for(int i=0;i<7;i++)
  {
    CRC = NextCRC(CRC, buf[i], CRC8_GEN);
  }

  //show received bytes
  Serial.print("Received: 0x");
  for(int i=0;i<8;i++)
  {
    char tmp[16];
    sprintf(tmp, "%.2X", buf[i]);
    Serial.print(tmp);
  }
  Serial.print("\n");
  Serial.print("CRC: "); Serial.print(CRC,HEX);Serial.print(" <-> BUFFER: "); 
  Serial.println(buf[7],HEX);

  return dataBytes;
}
