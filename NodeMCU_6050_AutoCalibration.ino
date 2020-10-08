// Arduino sketch that returns calibration offsets for MPU6050 //   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors.
// The effect of temperature has not been taken into account so I can't promise that it will work if you
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.


// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "SSD1306Wire.h"
#include "OLEDDisplayUi.h"
#include "imageInXBM.h"

//velocity
float vx = 0;

//movement
float moveX = 0;

//Low-Pass Filter
//{ y[n+1] - (1-a)y[n] = a*x[n] }
//it becomes stable LowPass filter, where { 0 < factor < 1 }
//it becomes stable HighBoost filter, where { 1 < factor < 2 }
//Note that { factor = 1 } produces output equal to input shifted by 1 samples: y[n] = x[n-1]
float SmoothDataX, SmoothDataY, SmoothDataZ = 0.0;
float LowPassFactor = 0.1;

//High-Pass Filter
float highpassX = 0.0;

//sampleing Rate
int period = 10; //sampling rate 1/10(ms) = 100samples per second
unsigned long time_now = 0;

//draw
uint8_t arrowDirection = 0;
uint8_t int_DemoMode = 0;
int angleX, angleY, angleZ;
uint8_t ball_X, ball_Y;
uint8_t plotX = 127;
uint8_t plotYForAX = 0;
uint8_t plotYForAY = 0;
bool boolDidInvertDisplay = false;

//button State
bool btnD6State = false;
bool btnD4State = false;


///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 9;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro(0x68); // <-- use for AD0 high

int SCL_PIN = D1;
int SDA_PIN = D2;
const int I2C_DISPLAY_ADDRESS = 0x3c;
SSD1306Wire     display(I2C_DISPLAY_ADDRESS, SDA_PIN, SCL_PIN);
OLEDDisplayUi   ui( &display );

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {

  pinMode(D3, INPUT_PULLUP);
  digitalWrite(D3, HIGH);
  pinMode(D5, INPUT_PULLUP);
  digitalWrite(D5, HIGH);
  pinMode(D7, INPUT_PULLUP);
  digitalWrite(D7, HIGH);


  Wire.begin();
  Serial.begin(115200);

  accelgyro.initialize();

  display.init();
  display.clear();
  //display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setContrast(255);
  display.drawString(64, 24, "Testing...");
  display.display();

  // start message
  Serial.println("\nMPU6050 Calibration Sketch");
  delay(1000);
  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(1000);
  // check connection
  if (!accelgyro.testConnection()) {
    display.clear();
    display.drawString(64, 10, "6050 ERROR");
    display.drawString(64, 30, "Reboot in 3 Sec.");
    display.display();
    delay(3000);
    ESP.restart();
  }

  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop() {
  if (state == 0) {
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(50);
  }

  if (state == 1) {
    Serial.println("\nCalculating offsets...");
    display.clear();
    display.drawString(64, 24, "Calibrating...");
    display.display();
    calibration();
    state++;
    delay(50);
  }

  if (state == 2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax);
    Serial.print("\t");
    Serial.print(mean_ay);
    Serial.print("\t");
    Serial.print(mean_az);
    Serial.print("\t");
    Serial.print(mean_gx);
    Serial.print("\t");
    Serial.print(mean_gy);
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset);
    Serial.print("\t");
    Serial.print(ay_offset);
    Serial.print("\t");
    Serial.print(az_offset);
    Serial.print("\t");
    Serial.print(gx_offset);
    Serial.print("\t");
    Serial.print(gy_offset);
    Serial.print("\t");
    Serial.println(gz_offset);
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
    state = 3;
    display.setFont(ArialMT_Plain_10);
  }

  if (digitalRead(D3) == LOW)  {
    delay(200);
    accelgyro.initialize();
    while (!accelgyro.testConnection()) {
      display.clear();
      display.drawString(64, 10, "6050 ERROR");
      display.drawString(64, 30, "Reboot in 3 Sec.");
      display.display();
      delay(3000);
      ESP.restart();
    }
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);
    state = 0;
  }

  if (digitalRead(D7) == LOW)  {
    int_DemoMode++;
    int_DemoMode %= 3;
    delay(200);
  }


  if ( (state == 3) && (millis() > time_now + period) ) {
    time_now = millis();
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //Serial.print(ax);
    //Serial.print(" ");
    //Serial.print(ay);
    //Serial.print(" ");
    //Serial.print(az);
    //Serial.println(" ");

    // Low-Pass Algorithm
    // SmoothDataX = SmoothDataX - (LowPassFactor * (SmoothDataX - ax));
    // SmoothDataY = SmoothDataY - (LowPassFactor * (SmoothDataY - ay));
    // SmoothDataZ = SmoothDataZ - (LowPassFactor * (SmoothDataZ - az));


    // { y += factor * (x - y) } or { y = (factor * y) + (x - (factor * x) }
    // Where x is input, y is output.
    SmoothDataX += LowPassFactor * (ax - SmoothDataX);
    SmoothDataY += LowPassFactor * (ay - SmoothDataY);
    SmoothDataZ += LowPassFactor * (az - SmoothDataZ);

    //round it to the nearest whole numberround it to the nearest whole number
    SmoothDataX = (int)(SmoothDataX + 0.5);
    SmoothDataY = (int)(SmoothDataY + 0.5);
    SmoothDataZ = (int)(SmoothDataZ + 0.5);


    //Serial.print(SmoothDataX);
    //Serial.print(" ");
    //Serial.print(SmoothDataY);
    //Serial.println(" ");
    //Serial.println(SmoothDataZ);

    /*
      //High-Pass Filter
        highpassX = ax - SmoothDataX;
        highpassX = (highpassX * 980) / 16384.0;
        Serial.print(highpassX);
        Serial.print(" ");
        vx = vx + highpassX * 0.01;
        moveX = moveX + vx * 0.01;
        Serial.print(vx);
        Serial.print(" ");
        Serial.print(moveX);
        Serial.println(" ");
    */

    //LowPassFilter Switch
    if (digitalRead(D5) == LOW) {
      angleX = map(SmoothDataX, -16384, 16384, -90, 90);
      angleY = map(SmoothDataY, -16384, 16384, -90, 90);
      angleZ = map(SmoothDataZ, -16384, 16384, -90, 90);
    } else {
      angleX = map(ax, -16384, 16384, -90, 90);
      angleY = map(ay, -16384, 16384, -90, 90);
      angleZ = map(az, -16384, 16384, -90, 90);
    }

    Serial.print(angleX);
    Serial.print(" ");
    Serial.print(angleY);
    Serial.println(" ");
    //Serial.println(SmoothDataZ);

    /*
      //display_direction_has_been_turned_90_degrees_CounterClockWise

      direction                  (Y = -90°)
      ↖ ↑ ↗  1 2 3              |
      ←    →  8   4   (X = 90°)--+--(X = -90°)
      ↙ ↓ ↘  7 6 5              |
                                (Y = +90°)
    */
    if (( abs(angleX) < 15 ) && (abs(angleY) < 15)) arrowDirection = 0; //0
    if ((( 60 > angleX ) && ( angleX >= 15 ) && ( -15 >= angleY) && ( angleY > -60)) || ((angleX >= 60) && ( angleY <= -60 ))) {
      arrowDirection = 3;
    } //1
    if (( abs(angleX) < 15 ) && (angleY < -15)) arrowDirection = 4; //2
    if ((( -15 >= angleX ) && ( angleX > -60 ) && ( -15 >= angleY) && ( angleY > -60)) || ((angleX <= -60) && ( angleY <= -60 ))) {
      arrowDirection = 5;
    } //3
    if (( abs(angleY) < 15 ) && (angleX <= -15)) arrowDirection = 6; //4
    if ((( -15 >= angleX ) && ( angleX > -60 ) && ( 60 > angleY) && ( angleY >= 15)) || ((angleX <= -60) && ( angleY >= 60 ))) {
      arrowDirection = 7;
    } //5
    if (( abs(angleX) < 15 ) && (angleY > 15)) arrowDirection = 8;  //6
    if ((( 60 > angleX ) && ( angleX >= 15 ) && ( 60 > angleY) && ( angleY >= 15)) || ((angleX >= 60) && ( angleY >= 60 ))) {
      arrowDirection = 1;
    } //7
    if (( abs(angleY) < 15 ) && (angleX >= 15)) arrowDirection = 2;  //8

    if (int_DemoMode == 0) {
      display.clear();
      switch (arrowDirection) {
        case 0:
          display.drawString(64, 26, "+");
          break;
        case 1:
          display.drawXbm(48, 16, D1_width, D1_height, D1_bits);
          break;
        case 2:
          display.drawXbm(48, 16, D2_width, D2_height, D2_bits);
          break;
        case 3:
          display.drawXbm(48, 16, D3_width, D3_height, D3_bits);
          break;
        case 4:
          display.drawXbm(48, 16, D4_width, D4_height, D4_bits);
          break;
        case 5:
          display.drawXbm(48, 16, D5_width, D5_height, D5_bits);
          break;
        case 6:
          display.drawXbm(48, 16, D6_width, D6_height, D6_bits);
          break;
        case 7:
          display.drawXbm(48, 16, D7_width, D7_height, D7_bits);
          break;
        case 8:
          display.drawXbm(48, 16, D8_width, D8_height, D8_bits);
          break;
      }
      display.drawString(64, 54, " X = " + String(angleX) + "° " +
                         " Y = " + String(angleY) + "° " +
                         " Z = " + String(angleZ) + "°" );
    } else if (int_DemoMode == 1) {
      //drawing has been turned 90 degrees CounterClockWise!
      ball_X = map(angleY, -90, 90, 0, 120);
      ball_Y = map(angleX, -90, 90, 0, 54);
      display.clear();
      display.drawString(64, 26, "+");
      display.drawString(ball_X + 3, ball_Y - 1, "O" );
      display.drawString(123 - ball_X, 53 - ball_Y, "O" );
    } else {
      plotYForAX = map(angleX, -90, 90, 0, 63);
      plotYForAY = map(angleY, -90, 90, 0, 63);
      if (plotX < 7 ) {
        display.clear();
        plotX = 127;
      }
      display.drawString(4, 10, "X");
      display.drawString(4, 42, "Y");
      display.setPixel(plotX, 61 - plotYForAX);
      display.setPixel(plotX, 65 - plotYForAY);
      plotX--;
    }

    display.display();

  }
}


///////////////////////////////////   FUNCTIONS   ////////////////////////////////////

void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}
