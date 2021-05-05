#include "CRC.h"
#include "TMC7300_Register.h"
#include <HardwareSerial.h>
#include <Arduino.h>
#include <Wire.h> //Wire lib for I2C communication
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Adafruit_NeoPixel.h> // for the lopy RGB led control source : https://github.com/adafruit/Adafruit_NeoPixel

#define CRC8_GEN 0x07


//Hardware pins
#define EN 14 // driver enable pin
#define ad0 25 // driver adress pin
#define ad1 26 // driver adress pin
#define RX 15 // driver RX pin
#define TX 4 // driver TX pin
#define battery 36 // battery reading pin
#define encoderL1 38 // encoder on left motor (yellow wire)
#define encoderR1 37 // encoder on right motor (yellow wire)
#define encoderL2 35 // encoder on left motor (white wire)
#define encoderR2 39 // encoder on right motor (white wire)
#define out 33 //pin test periode echantillonage

//freq echantill
float fe =100;  //freq echantill

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
double tetam[3]={0,0,0};
float cmd[3] = {0,0,0};
double etetaSum=0;
double ui[2] = {0,0};
double ud[2] = {0,0};

/*const double Kp=-20.2617;
const double Kd=-0.5395;
const double Ki=473.9586;*/
const double Kp = 5;
const double Kd = 3;
const double Ki = 20;

const double tau_d = abs(2000*Kd);
const double tau =0.01;
const double a1 = 4*Kd+2*tau_d*Ki*tau+Ki*tau+2*Kp*tau_d+Kp;
const double a2 = -8*Kd+Ki*tau+2*tau_d*Ki*tau-2*Kp*tau_d+Kp+2*Kp*tau_d+1;
const double a3 = -4*tau_d+4*Kd;
const double beta = (tau-2*tau_d)/(2*tau_d + tau);
const double alpha = 2*Kd/(2*tau_d+tau);

const double b1 = 4*tau_d+2;
const double b2 = -8*tau_d;
const double b3 = 4*tau_d-2;

//other timers
double chrono;
uint32_t timer1;
uint32_t timer2;

// timer for calculate phase with encoder
double posL=0; //position angulaire roue gauche en radian
double posR=0; //position angulaire roue droite en radian
double posprecL=0;
double posprecR=0;
const double R=0.021;
double vm = 0;
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
  pinMode(out,OUTPUT);
  
  //setting driver adress
  digitalWrite(ad0, HIGH);
  digitalWrite(ad1, LOW);
  digitalWrite(out, LOW);

  //set encoder position functions
  attachInterrupt(digitalPinToInterrupt(encoderL1), encoderChangeL1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderR1), encoderChangeR1, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoderL2), encoderChangeL2, RISING);
  //attachInterrupt(digitalPinToInterrupt(encoderR2), encoderChangeR2, RISING);

  //timer for automatic control flag
  timerA = timerBegin(0, 80, true); //prescaler : 80e6/80 = 1e6 (ESP32 : 80MHz)
  timerAttachInterrupt(timerA, &onTime, true);
  timerAlarmWrite(timerA, 1e6/fe , true);
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

  
}

void loop(){
  
  //runMotors(100,100); //runMotor(rightM pwn, leftM pwm)
  //Serial.print(phaseL); Serial.print("\t\t"); Serial.println(phaseR);
  //Serial.print(posL); Serial.print("\t\t"); Serial.println(posR);
  for (int i = -255; i<=255; i+=5){
    runMotors(i,i);
    float tprec = millis();
    double pposL = posL;
    double pposR = posR;
    delay(1000);
    Serial.print(i); Serial.print("\t"); Serial.println((-pposL+posL)*1000/((millis() - tprec)/1000)); /*Serial.print("\t"); Serial.println((-pposR+posR)*1000/((millis() - tprec)/1000));*/
  }
  delay(60000);
  
  if(0){ 
    digitalWrite(out, HIGH);
    
    readValues();
    getAngles();
    Serial.print("angle : "); Serial.println(kalAngleY);  
    
    double up;
    
    tetam[0] = kalAngleY;

    up =  tetam[0]*Kp;
    ui[0] = ui[1]-tetam[0]+Ki*tau/2-ui[1]*Ki*tau/2;
    ud[0] = beta*ud[1]-alpha*tetam[0]+alpha*tetam[1];

    cmd[0] = up+ui[0]+ud[0];
    //cmd[0] = (-b1*tetam[0]-b2*tetam[1]-b3*tetam[2]-a2*cmd[1]-a3*cmd[2])/a1;

    tetam[1] = tetam[0];
    ui[1] = ui[0];
    ud[1] = ud[0];
    
    
    


    
    float cmd2;
    if abs(cmd[0])>155
      cmd2 = 0.922*cmd[0]-238.19;
    else{
      if (cmd[0]>0)
        cmd2 = 0.9567*cmd[0]-991.55;
      else
        cmd2 = 0.9943*cmd[0]+497.69;
    }
      
    //saturation
    Serial.print("avant satu"); Serial.print("\t"); Serial.print(cmd[0]); Serial.print("\t\t"); Serial.println(cmd2);
    //satutation 
    if (cmd[0]>255)
      cmd[0] = cmd2 = 255;
    if (cmd[0]<-255)
      cmd[0] = cmd2 = -255;



    //Serial.print("apres satu"); Serial.print("\t"); Serial.print(cmd[0]); Serial.print("\t\t"); Serial.println(cmd2);
    runMotors(cmd2,cmd[0]);


    digitalWrite(out, LOW);
      
    flag=false;
  }
  
  
}

void IRAM_ATTR onTime() {
   vm = R*(-posprecR+posR-posprecL+posL)*fe/2;
   posprecR = posR;
   posprecL = posL;
   flag=true;
}

//encoder functions
void encoderChangeL1(){
  if (digitalRead(encoderL2))
    posL -= 2*PI/(3*150.58); //position angulaire roue gauche en radian
  else
    posL += 2*PI/(3*150.58);
}

void encoderChangeR1(){
  if (digitalRead(encoderR2))
    posR += 2*PI/(3*150.58); //position angulaire roue droite en radian
  else
    posR -= 2*PI/(3*150.58);
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

  /*uint8_t CRC = 0;
  if (Serial1.available()){
    unsigned char buf[8];
    buf[0] = 0x05;
    buf[1] = TMC7300_PWM_AB;
    buf[2] = TMC7300_PWM_AB|0x80;
    buf[3] = (data >> 24) & 0xff;
    buf[4] = (data >> 16) & 0xff;
    buf[5] = (data >> 8) & 0xff;
    buf[6] = data & 0xff; 
    buf[7] = CRC;
    
    Serial1.write(buf, 8); //write datagram
  }*/
}

void uartWriteDatagram(uint8_t SLAVEADDR, uint8_t registerAddress, 
      unsigned long datagram) {
  //TMC5072 takes 64 bit data: 8 sync + reserved, 8 chip address, 
  //8 register address, 32 data, 8 CRC

  uint8_t CRC = 0;
  int temp;
  
  CRC = NextCRC(CRC, 0x05, CRC8_GEN);
  CRC = NextCRC(CRC, SLAVEADDR, CRC8_GEN);
  CRC = NextCRC(CRC, registerAddress|0x80, CRC8_GEN);  
  CRC = NextCRC(CRC, (datagram >> 24) & 0xff, CRC8_GEN);
  CRC = NextCRC(CRC, (datagram >> 16) & 0xff, CRC8_GEN);
  CRC = NextCRC(CRC, (datagram >> 8) & 0xff, CRC8_GEN);
  CRC = NextCRC(CRC, datagram & 0xff, CRC8_GEN);
  
  
  if (Serial1.available()){
    unsigned char buf[8];
    buf[0] = 0x05;
    buf[1] = SLAVEADDR;
    buf[2] = registerAddress|0x80;
    buf[3] = (datagram >> 24) & 0xff;
    buf[4] = (datagram >> 16) & 0xff;
    buf[5] = (datagram >> 8) & 0xff;
    buf[6] = datagram & 0xff; 
    buf[7] = CRC;
    
    Serial1.write(buf, 8); //write datagram
    //Serial1.flush();  //wait until all datas are written
    //Serial1.readBytes(buf, 8); //clear input buffer
  }
  else{
    Serial.println("UART not available");
  }
}
