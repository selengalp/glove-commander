/*
         ~~PİN BAĞLANTILARI~~
         Arduino --->  GY-88
          3.3v          3.3v
          GND           GND
          A5            SCL
          A4            SDA
          
++++++-----++-------++++++----++------+++++++++----------+++++---------------
~~~~~~~~MPU6050 REGİSTER BİLGİLERİ~~ (!!!ALL REGISTER VALUES ARE IN HEXADECIMAL!!!)
      
   ***MPU6050 ADRESS: 0x68
   
   ***CONFIG   :1A
   DLPF SETTINGS ARE DONE HERE
   
   ***GYRO CONFIG   : 0x1B
            FS_SEL ==> [4:3] bits
            FS_SEL selects the full scale range of the
            gyroscope outputs according to the following table.
           _____________________________
           |~~FS_SEL Full Scale Range~~ |
           |            LSB Sensitivity |
           |  0 ± 250 °/s  131 LSB/°/s  |
           |  1 ± 500 °/s   65.5 LSB/°/s|
           |  2 ± 1000 °/s  32.8 LSB/°/s|
           |  3 ± 2000 °/s  16.4 LSB/°/s|
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   ***ACCEL CONFIG  : 0x1C
           AFS_SEL ==> [4:3] bits
           AFS_SEL selects the full scale range of the
           accelerometer outputs according to the following table.
          _________________________________
           |~~AFS_SEL Full Scale Range~~|
           |            LSB Sensitivity |
           |   0 ±   2 g  16384 LSB/g   |
           |   1 ±   4 g   8192 LSB/g   |
           |   2 ±   8 g   4096 LSB/g   |
           |   3 ±   16 g  2048 LSB/g   |
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
           
   ***ACCEL XOUT H  : 3B
      ...             ...    
      ACCEL ZOUT L  : 40     
      
      
   ***TEMP OUT H     : 41
      TEMP OUT L     : 42
          Temperature in degrees 
        C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
          Please note that the math in the above equation is in decimal.
      
      
   ***GYRO XOUT H    : 43
      ...             ...
      GYRO ZOUT L    : 48
      
            Note: Register Names ending in _H and _L contain the high 
            and low bytes, respectively, of an internal register value.
            In the detailed register tables that follow, register names
            are in capital letters, while register values are in capital
            letters and italicized. For example, the ACCEL_XOUT_H register
            (Register 59) contains the 8 most significant bits,
            ACCEL_XOUT[15:8], of the 16-bit X-Axis accelerometer
            measurement, ACCEL_XOUT.
            
            H OLUNCA [15:8] BİTLER
            L OLUNCA [ 7:0]  BİTLER
      
   ***USER CTRL      : 6A
   REGISTER MAP PAGE 39,
   This register allows the user to enable and disable the FIFO buffer,
   I2C Master Mode, and primary I2C interface. The FIFO buffer, I2C Master,
   sensor signal paths and sensor registers can also be reset using this register.
   
   
   ***PWR MGMT 1     : 6B
       CLKSEL ==> [2:0] bits
       CLKSEL |  Clock Source
        0     |  Internal 8MHz oscillator
        1     |  PLL with X axis gyroscope reference
        2     |  PLL with Y axis gyroscope reference
        3     |  PLL with Z axis gyroscope reference
        4     |  PLL with external 32.768kHz reference
        5     |  PLL with external 19.2MHz reference
        6     |  Reserved
        7     |  Stops the clock and keeps the timing generator in reset
    Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. 
    However, it is highly recommended that the device be configured to use one of the gyroscopes 
    (or an external clock source) as the clock reference for improved stability. 
    The clock source can be selected according to the following table.      
    
  */
  
#include <Wire.h>
#include "Kalman.h"

#define MPU6050        0x68
#define MPU_DLPF       0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define TEMP_OUT_H     0x41
#define GYRO_XOUT_H    0x43
#define MPU_USER_CTRL  0x6A
#define MPU_PWR_MGMT1  0x6B  

#define DEADZONE 5

// Zamanlama degiskenleri
int STD_LOOP_TIME  = 25;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

byte buffer[20];

float ax, ay, az, t, gx, gy, gz;
float acc_angle_x, acc_angle_y;
float gyro_angle_x, gyro_angle_y;

unsigned long timer;

Kalman kalman_x;
Kalman kalman_y;

float kalman_angle_x;
float kalman_angle_y;

  void setup()
  {
    Serial.begin(9600);
    Wire.begin(9600);
    
    writeTo(MPU6050,GYRO_CONFIG, 0x08);
    writeTo(MPU6050,ACCEL_CONFIG, 0x08);    
    writeTo(MPU6050,MPU_PWR_MGMT1, 0x00);    
    writeTo(MPU6050,MPU_DLPF, 0x03); 

    kalman_x.setAngle(0);
    kalman_y.setAngle(0);
    
    delay(100);
    
    timer = micros();
  }
  
void loop()
{
  
  readAllData();
  
  acc_angle_x = (atan2(ax,az)+PI)*RAD_TO_DEG;
  acc_angle_y = (atan2(ay,az)+PI)*RAD_TO_DEG;
  
  kalman_angle_x = kalman_x.getAngle(acc_angle_x,gx,(float)(micros() - timer));
  kalman_angle_y = kalman_y.getAngle(acc_angle_y,gy,(float)(micros() - timer));

  timer = micros();

  if (kalman_angle_x < 180 + DEADZONE && kalman_angle_x > 180 - DEADZONE) {
    kalman_angle_x = 180;
  }
  
  if (kalman_angle_y < 180 + DEADZONE && kalman_angle_y > 180 - DEADZONE) {
    kalman_angle_y = 180;
  }
  
  if (kalman_angle_x > 225) {
    kalman_angle_x = 225;
  } else if (kalman_angle_x < 135) {
    kalman_angle_x = 135;
  }
  if (kalman_angle_y > 225) {
    kalman_angle_y = 225;
  } else if (kalman_angle_y < 135) {
    kalman_angle_y = 135;
  }

  Serial.print((int)kalman_angle_x);
  Serial.print("-");
  Serial.print((int)kalman_angle_y);
  Serial.println("-");
  
  // Donguyu zamanlamak icin
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
}
  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~FUNCTIONS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  void writeTo(int device, byte address, byte val) 
  {
    Wire.beginTransmission(device); // start transmission to device 
    Wire.write(address);             // send register address
    Wire.write(val);                 // send value to write
    Wire.endTransmission();         // end transmission
  }
  
  
//_____________---------------------__________________---------------------
//-------------____________________------------------_______________________

  void readFrom(int device, byte address, int count)
  {
    Wire.beginTransmission(device); // start transmission to device 
    Wire.write(address);             // send register address
    Wire.endTransmission();         // end transmission
    
    // read "count" times data bytes; WE CAN READ MAX 20 BYTES
    //BECAUSE buffer[] ARRAY'S LIMIT IS 20
    Wire.beginTransmission(device);
    Wire.requestFrom(device,count); // Read 12 bytes
    int i = 0;
    while(Wire.available())
    {
        buffer[i] = Wire.read();
        i++;
    }
    Wire.endTransmission();
  }
  
//__________________--------------------___________________----------------------
//-------------------__________________----------------------_______________________


  //READING ALL DATA FROM MPU6050
  void readAllData()
  {
    readFrom(MPU6050, ACCEL_XOUT_H, 14);
    
    ax = (float)(buffer[0] << 8 | buffer[1]) / 8192;
    ay = (float)(buffer[2] << 8 | buffer[3]) / 8192;
    az = (float)(buffer[4] << 8 | buffer[5]) / 8192;
    t =  (float)(buffer[6] << 8 | buffer[7])/340 + 36.53;
    gx = (float)(buffer[8] << 8 | buffer[9]) / 65.5;
    gy = (float)(buffer[10] << 8 | buffer[11]) / 65.5;
    gz = (float)(buffer[12] << 8 | buffer[13]) / 65.5;
        
  }
//__________________--------------------___________________----------------------
//-------------------__________________----------------------_______________________
