/************************************************************************/
/*  PICxel_2_strand_demo.pde - PIC32 Neopixel Library Demo              */
/*                                    */
/*  A simple to use library for addressable LEDs like the WS2812 for    */
/*  the PIC32 line of microcontrollers.                                 */
/*                                                                      */
/*  tested supported boards:                                            */
/*    - Digilent UNO32                                                  */
/*    - Digilent UC32                                                   */
/*                                                                      */
/*  This library is protected under the GNU GPL v3.0 license            */
/*  http://www.gnu.org/licenses/                                        */
/************************************************************************/
// Call of libraries
#include <Wire.h>
#include <SparkFunLSM9DS1.h>

// Déclaration des adresses du module
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
#define DECLINATION -0.33 // déclinaison (en degrés) pour Paris.

float pitchDefault,rollDefault;
float pitchPrev, rollPrev;
boolean blinkRight, blinkLeft, slowDownBlink;

#include <PICxel.h>

#define number_of_LEDs_strip1 15
#define LED_pin_strip1 0   

#define number_of_LEDs_strip2 15
#define LED_pin_strip2 1   

#define millisecond_delay_strip 50

PICxel strip1(number_of_LEDs_strip1, LED_pin_strip1, HSV);
PICxel strip2(number_of_LEDs_strip2, LED_pin_strip2, HSV);

int hue = 0;
int sat = 255;
int val = 100;
int prevTime = 0;
int runner_num = 0;
int count_up_flag = 0;
int runner_delay_count = 0;
int runner_hue = 0;
//int TurnLeft = 0;
int pin_2 = 2;
int pin_3 = 3;

void setup(){
  strip1.begin();
  //strip.setBrightness(50); //brightness is not needed in HSV
  strip1.setBrightness(50);
  strip2.begin();
  strip2.setBrightness(50);
  pinMode(pin_3, INPUT);
  pinMode(pin_2, OUTPUT);

  //Left and Right LED Strips
  //TurnLeft();

  blinkRight = false;
  blinkLeft = false;
  slowDownBlink = false;
}

void loop()
{
  digitalWrite(pin_2, HIGH);
  if(digitalRead (pin_3) == HIGH)
  {
    TurnLeft(); 
    delay(10);
    //Stop();
  }
  else
  {
    TurnRight();
    delay(10);
    //Stop();
  }

  Serial.begin(115200); // initialization of serial communication
   if (!imu.begin())
  {
   Serial.println("Probleme de communication avec le LSM9DS1.");
    while (1);
  } 
  imu.settings.device.commInterface = IMU_MODE_I2C; // initialization of the module
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
 
  if ( imu.gyroAvailable() )
 {
  imu.readGyro(); // acquisition des données du gyroscope
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

  calculateOrientation(imu.ax, imu.ay, imu.az,-imu.my, -imu.mx, imu.mz);
  Serial.println();
  lastPrint = millis();
 }
}

////////////////////////User Defined Functions///////////////////////////////////////
void TurnLeft()
{

   if(millis()-prevTime > 10){
    prevTime = millis();
    hue++;
    val++;
    sat++;
    if(hue > 1535)
      hue = 0;
    if(val > 255)
      val = 0;
    if(sat > 255)
      sat = 0;
    
    for(int i= 0; i<number_of_LEDs_strip1; i++){
    if(i < number_of_LEDs_strip1)
      strip1.HSVsetLEDColor(i, 200, 255, 50);
      //strip1.HSVsetLEDColor(i,0,255,0);
    //else if(i < 2*(number_of_LEDs_strip1/2))
      //strip1.HSVsetLEDColor(i, 0, 255,val);
    //else
      //strip1.HSVsetLEDColor(i, 0, sat,  100);
    }
    if(runner_delay_count == 5){
      if(count_up_flag == 0){
        strip2.HSVsetLEDColor(runner_num, 200, 255, 100);
        strip2.clear(runner_num-1);
        runner_num++;
        if(runner_num == number_of_LEDs_strip2)
          count_up_flag = 1;
      }
      else{
        strip2.HSVsetLEDColor(runner_num, 0, 0, 0);
        strip2.clear(runner_num+3);
        runner_num--;
        if(runner_num == 0)
          count_up_flag = 0;
      }
      if(runner_hue == 1400)
        runner_hue = 0;
      runner_delay_count = 0;
    }
    runner_delay_count++; 
  
strip1.refreshLEDs();
strip2.refreshLEDs();
delay(1);
  }
}
void TurnRight()
{

   if(millis()-prevTime > 10){
    prevTime = millis();
    hue++;
    val++;
    sat++;
    if(hue > 1535)
      hue = 0;
    if(val > 255)
      val = 0;
    if(sat > 255)
      sat = 0;
    
    for(int i= 0; i<number_of_LEDs_strip1; i++){
    if(i < number_of_LEDs_strip1)
      strip1.HSVsetLEDColor(i, 200, 255, 50);
      //strip1.HSVsetLEDColor(i,0,255,0);
    //else if(i < 2*(number_of_LEDs_strip1/2))
      //strip1.HSVsetLEDColor(i, 0, 255,val);
    //else
      //strip1.HSVsetLEDColor(i, 0, sat,  100);
    }
    if(runner_delay_count == 5){
      if(count_up_flag == 0){
        strip2.HSVsetLEDColor(runner_num, 0, 255, 100);
        strip2.clear(runner_num-1);
        runner_num++;
        if(runner_num == number_of_LEDs_strip2)
          count_up_flag = 1;
      }
      else{
        strip2.HSVsetLEDColor(runner_num, 0, 0, 0);
        strip2.clear(runner_num+3);
        runner_num--;
        if(runner_num == 0)
          count_up_flag = 0;
      }
      if(runner_hue == 1400)
        runner_hue = 0;
      runner_delay_count = 0;
    }
    runner_delay_count++; 
  
strip1.refreshLEDs();
strip2.refreshLEDs();
delay(1);
  }
}
//Declare Function to "stop" strip 1
void Stop()
{
  if(millis()-prevTime > 10){
    prevTime = millis();
    hue++;
    val++;
    sat++;
    if(hue > 1535)
      hue = 0;
    if(val > 255)
      val = 0;
    if(sat > 255)
      sat = 0;
    
    for(int i= 0; i<number_of_LEDs_strip1; i++){
    if(i < number_of_LEDs_strip1)
      strip1.HSVsetLEDColor(i, 200, 255, 50);
      //strip1.HSVsetLEDColor(i,0,255,0);
    //else if(i < 2*(number_of_LEDs_strip1/2))
      //strip1.HSVsetLEDColor(i, 0, 255,val);
    //else
      //strip1.HSVsetLEDColor(i, 0, sat,  100);
    }
    if(runner_delay_count == 5){
      if(count_up_flag == 0){
        strip2.HSVsetLEDColor(runner_num, 0, 0, 0);
        strip2.clear(runner_num-1);
        runner_num++;
        if(runner_num == number_of_LEDs_strip2)
          count_up_flag = 1;
      }
      else{
        strip2.HSVsetLEDColor(runner_num, 0, 0, 0);
        strip2.clear(runner_num+3);
        runner_num--;
        if(runner_num == 0)
          count_up_flag = 0;
      }
      if(runner_hue == 1400)
        runner_hue = 0;
      runner_delay_count = 0;
    }
    runner_delay_count++; 
  
strip1.refreshLEDs();
strip2.refreshLEDs();
delay(1);
  }
}

void calculateOrientation(float ax, float ay, float az, float mx, float my, float mz)
{
 float roll = atan2(ay, az);
 float pitch = atan2(-ax, sqrt(ay * ay + az * az));
 float heading;
// if (my == 0)
//  heading = (mx < 0) ? PI : 0;
// else
//  heading = atan2(mx, my);
//  heading -= DECLINATION * PI / 180;
//  if (heading > PI) heading -= (2 * PI);
//  else if (heading < -PI) heading += (2 * PI);
//  else if (heading < 0) heading += 2 * PI;
//  heading *= 180.0 / PI;
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
  if(rollDiff > 30)
  {
    Serial.println("Turn right");
    blinkRight = true;
  }
  else if (rollDiff < -30)
  {
    Serial.println("Turn left");
    blinkLeft = true;
  }
  if(pitchDiff > 30)
  {
    Serial.println("Slowing down");
    slowDownBlink = false;
  }
  pitchPrev = pitch;
  rollPrev = roll;
  
}
