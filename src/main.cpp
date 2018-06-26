#include <Arduino.h>
#include <Wire.h>
#include "BMP085.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "ADXL345.h"


void scan(){
    Serial.println(" Scanning I2C Addresses");
    uint8_t cnt=0;
    for(uint8_t i=0;i<128;i++){
    Wire.beginTransmission(i);
    uint8_t ec=Wire.endTransmission(true);
    if(ec==0){
        if(i<16)Serial.print('0');
        Serial.print(i,HEX);
        cnt++;
    }
    else Serial.print("..");
    Serial.print(' ');
    if ((i&0x0f)==0x0f)Serial.println();
    }
    Serial.print("Scan Completed, ");
    Serial.print(cnt);
    Serial.println(" I2C Devices found.");
}

ADXL345 accel;
HMC5883L compass;
ITG3200 gyro;
BMP085 bmp(BMP085_ADDRESS);
float t, p, a;
int16_t x,y,z;
int16_t ax,ay,az;
const float maxg = 16000.0;

void setup()
{
    Serial.begin(115200);
    Wire.begin(22,21);
    scan();
    bmp.initialize();
    gyro.initialize();
    compass.initialize();
    accel.initialize();

    if (!bmp.testConnection())
        Serial.println("BMP error!");
    if (!gyro.testConnection())
        Serial.println("Gyro error!");
    if (!compass.testConnection())
        Serial.println("Compass error!");
    if (!accel.testConnection())
        Serial.println("Accelerometer error!");

    compass.setGain(HMC5883L_GAIN_1370);
    compass.setMode(HMC5883L_MODE_CONTINUOUS);
    compass.setDataRate(HMC5883L_RATE_15);
    compass.setSampleAveraging(HMC5883L_AVERAGING_4);

    accel.setRange(ADXL345_RANGE_2G);
}

void bmp_test()
{
    bmp.setControl(BMP085_MODE_TEMPERATURE);
    t = bmp.getTemperatureC();
    bmp.setControl(BMP085_MODE_PRESSURE_3);
    p = bmp.getPressure();// * 0.0075006375541921;
    a = bmp.getAltitude(p);
    Serial.printf("B: Temp=%04.2f Alt=%04.2f Press=%04.2f \n\r", t, a, p);
}

void gyro_test()
{
    float t = gyro.getTemperature();
    gyro.getRotation(&x,&y,&z);
    Serial.printf("G: T=%4.0f %4.0f:%4.0f:%4.0f\n\r",t, 100.0 * ((float)x/maxg), 100.0 * ((float)y/maxg), 100.0 * ((float)z/maxg));
}



// No tilt compensation
float noTiltCompensate(int x, int y, int z)
{
  float heading = atan2(y, x);
  return heading;
}
 
// Tilt compensation
float tiltCompensate(int x, int y, int z, int ax, int ay, int az)
{
  // Pitch & Roll 

  float roll;
  float pitch;

  roll = asin(ay*0.004);
  pitch = asin(-ax*0.004);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt compensation
  float Xh = x * cosPitch + z * sinPitch;
  float Yh = x * sinRoll * sinPitch + y * cosRoll - z * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}
// Correct angle
float correctAngle(float heading)
{
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }
  return heading;
}
void compass_test()
{
    accel.getAcceleration(&ax,&ay,&az);
    compass.getHeading(&x,&y,&z);
    float heading1 = noTiltCompensate(x,y,z);
    float heading2 = tiltCompensate(x,y,z, ax,ay,az);

    if (heading2 == -1000)
    {
        heading2 = heading1;
    }

    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
    heading1 += declinationAngle;
    heading2 += declinationAngle;

    // Correct for heading < 0deg and heading > 360deg
    heading1 = correctAngle(heading1);
    heading2 = correctAngle(heading2);

    // Convert to degrees
    heading1 = heading1 * 180/M_PI; 
    heading2 = heading2 * 180/M_PI; 

    Serial.printf("C: %3.2f %3.2f\n\r",heading1, heading2);
}

float fXg,fYg,fZg, pitch, roll;
void accel_test(float alpha = 0.5)
{
    accel.getAcceleration(&ax, &ay, &az);
 
    //Low Pass Filter
    fXg = ax * alpha + (fXg * (1.0 - alpha));
    fYg = ay * alpha + (fYg * (1.0 - alpha));
    fZg = az * alpha + (fZg * (1.0 - alpha));
 
    //Roll & Pitch Equations
    roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
    pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;

    Serial.printf("A: %4.1f %4.1f\n\r",pitch, roll);
}


void loop()
{
    bmp_test();
    gyro_test();
    compass_test();
    accel_test();
    delay(100);
}