/************************************************************************
*
* Test of Pmod NAV (Based on Jim Lindblom's program)
*
*************************************************************************
* Description: Pmod_NAV
* All data (accelerometer, gyroscope, magnetometer) are displayed
* In the serial monitor
*
* Material
* 1. Arduino Uno
* 2. Pmod NAV (dowload library
* https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library)
* Licence Beerware
*
* Wiring
* Module<----------> Arduino
* J1 broche 6 3.3V
* J1 broche 5 GND
* J1 broche 4 A5
* J1 broche 2 A4
************************************************************************/
// Call of libraries
#include <Wire.h>
#include <SparkFunLSM9DS1.h>

// DÃ©claration des adresses du module
#define LSM9DS1_M 0x1E
#define LSM9DS1_AG 0x6B

LSM9DS1 imu; // Creation of the object imu

// Configuration du module
#define PRINT_CALCULATED
#define PRINT_SPEED 250
static unsigned long lastPrint = 0;

// The earth's magnetic field varies according to its location.
// Add or subtract a constant to get the right value
// of the magnetic field using the following site
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -0.33 // dÃ©clinaison (en degrÃ©s) pour Paris.

float pitchDefault,rollDefault;
float pitchPrev, rollPrev, accelPrev = 1, acceleration;
bool slowDown = false, permission = true;

void setup(void)
{
//   rollDefault = atan2(imu.ay, imu.az);
//   pitchDefault = atan2(imu.ax, sqrt(imu.ay * imu.ay + imu.az * imu.az));
//   rollDefault *= 180.0 / PI;
//   pitchDefault *= 180.0 / PI;
 Serial.begin(115200); // initialization of serial communication
 imu.settings.device.commInterface = IMU_MODE_I2C; // initialization of the module
 imu.settings.device.mAddress = LSM9DS1_M;
 imu.settings.device.agAddress = LSM9DS1_AG;
 if (!imu.begin())
 {
  Serial.println("Probleme de communication avec le LSM9DS1.");
  while (1);
 }
  pinMode(3, OUTPUT);
  pinMode(4 , OUTPUT);

}

void loop()
{
  
 if ( imu.gyroAvailable() )
 {
  imu.readGyro(); // acquisition des donnÃ©es du gyroscope
 }
 if ( imu.accelAvailable() )
 {
  imu.readAccel(); //Acquisition of accelerometer data
 }
 if ( imu.magAvailable() )
 {
  imu.readMag(); // Acquisition of the magnetometer
 }
 
 if ((lastPrint + PRINT_SPEED) < millis())
 {

  printAccel();
 
  calculateOrientation(imu.ax, imu.ay, imu.az,-imu.my, -imu.mx, imu.mz);
  
  Serial.println();
  lastPrint = millis();
 }
}

//void printGyro()
//{
// Serial.print("G: ");
//#ifdef PRINT_CALCULATED
// Serial.print(imu.calcGyro(imu.gx), 2);
// Serial.print(", ");
// Serial.print(imu.calcGyro(imu.gy), 2);
// Serial.print(", ");
// Serial.print(imu.calcGyro(imu.gz), 2);
// Serial.println(" deg/s");
//#elif defined PRINT_RAW
// Serial.print(imu.gx);
// Serial.print(", ");
// Serial.print(imu.gy);
// Serial.print(", ");
// Serial.println(imu.gz);
//#endif
//}
//
void printAccel()
{
 Serial.print("A: ");
#ifdef PRINT_CALCULATED
 Serial.print(imu.calcAccel(imu.ax), 2);
 Serial.print(", ");
 Serial.print(imu.calcAccel(imu.ay), 2);
 Serial.print(", ");
 float accel1 = imu.calcAccel(imu.az);
 Serial.print(accel1, 2);
 Serial.println(" g");
//#elif defined PRINT_RAW
// Serial.print(imu.ax);
// Serial.print(", ");
// Serial.print(imu.ay);
// Serial.print(", ");
// Serial.println(imu.az);
// if((accel1 - accelPrev) < 0.2)
// {
//   slowDown = true;
// }
 accelPrev = accel1;

#endif

}
//void printMag()
//{
// Serial.print("M: ");
//#ifdef PRINT_CALCULATED
// Serial.print(imu.calcMag(imu.mx), 2);
// Serial.print(", ");
// Serial.print(imu.calcMag(imu.my), 2);
// Serial.print(", ");
// Serial.print(imu.calcMag(imu.mz), 2);
// Serial.println(" gauss");
//#elif defined PRINT_RAW
// Serial.print(imu.mx);
// Serial.print(", ");
// Serial.print(imu.my);
// Serial.print(", ");
// Serial.println(imu.mz);
//#endif
//}

/////////// USER DEFINED FUNCTION /////////////
void calculateOrientation(float ax, float ay, float az, float mx, float my, float mz)
{
 float roll = atan2(ay, az);
 float pitch = atan2(-ax, sqrt(ay * ay + az * az));
 float heading;
 if (my == 0)
  heading = (mx < 0) ? PI : 0;
 else
  heading = atan2(mx, my);
  heading -= DECLINATION * PI / 180;
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
  Serial.print("Prev Pitch, Roll: ");
  Serial.print(pitchPrev, 2);
  Serial.print(", ");
  Serial.println(rollPrev, 2);
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  //Serial.print("Heading: "); Serial.println(heading, 2);
  float pitchDiff = pitchPrev - pitch;
  float rollDiff = rollPrev - roll;
  Serial.print("Diff Pitch, Roll: ");
  Serial.print(pitchDiff, 2);
  Serial.print(", ");
  Serial.println(rollDiff, 2);
  if (pitchDiff  > 30)
  {
    Serial.println("Slowing down");
    digitalWrite(6, HIGH);
    delay(1000);
    digitalWrite(6, LOW);
    slowDown = false;
  }
  else if(rollDiff > 50 )
  {
    
    Serial.println("Turn right");
    digitalWrite(3, HIGH);
    delay(6000);
    digitalWrite(3, LOW);
   
  }
  else if (rollDiff < -50 )
  {
    
    Serial.println("Turn left");
    digitalWrite(4, HIGH);
    delay(6000);
    digitalWrite(4, LOW);
 
  }

  pitchPrev = pitch;
  rollPrev = roll;
  
}


