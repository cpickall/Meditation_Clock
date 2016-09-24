//Caitlin Pickall | Physical Computing 1 | Spring 2015 | Adiel Fernandez
//Midterm Project - Meditation Clock


#include <CapacitiveSensor.h>
#include <NewPing.h>
#include<Wire.h>
#include <Servo.h>
#include <Multiplex7Seg.h>


//variables for touch sensor
CapacitiveSensor cs_4_2 = CapacitiveSensor(A0,A1);        // A1 is write pin, A0 is sensor pin
int capThresh = 15;    //capacitive sensor threshold

//variables for sonar distance sensor
#define TRIGGER_PIN  A2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance
//Ultrasonic reader
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//variables for accelerometer
const int MPU=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//variables for segment display
byte digitPins[] = {9, 10}; // LSB to MSB
byte segmentPins[] = {
  2, 3, 4, 5, 6, 7, 8}; // seg a to g


//mode 0: off, 1: time-set, 2: waiting for rotation, 3: counting down
int mode = 0; 

//variables for clock time set
//time to set clock to
int medTime = 15;
//variable to hold minutes as they change from the sensor value
int minutes = 5; 
//variables for smoothing functions
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
long timeLeft;
int minLeft;



//variables for time countdown
long startMil = 0;
long curMil = 0;

//variables for motor
Servo motor;
int motorPin = 12;


void setup(){
  //I2C communication with accelerometer 
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(115200); 
  //  Serial.println("serial started");
  Multiplex7Seg::set(1, 2, digitPins, segmentPins); // initialize 


  //set initial values in sonar sensor averaging array to 0
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;  
  //set pins for segment display
  pinMode(2, OUTPUT);   
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  motor.attach(motorPin);
  motor.write(150);

}


void loop(){

//  Serial.print("Mode: ");
//  Serial.println(mode);

  //accelerometer input
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true); // request a total of 14 registers
  //read gyro values from accelerometer chip
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  //Serial.print(" | GyX = ");
  //Serial.print(GyX);
  //Serial.print(" | GyY = ");
  //Serial.print(GyY);
  //Serial.print(" | GyZ = ");
  //Serial.println(GyZ);

  //Serial.print("accel: ");
  //Serial.println(GyZ);

  delay(50);



  //capacitive touch sensor input  
  long capSense =  cs_4_2.capacitiveSensor(30);
//  Serial.println(capSense);

  //SET TIME if cap sensor is touched
  if (capSense > capThresh){  //if not already in time-set mode and touch the cap sensor, enter time-set mode
    //if clock is counting down, show remaining time when touched

    //otherwise (if clock is off or waiting to start) activate time-set mode

    mode = 1;
    //  Serial.println("Chiming once and blinking the number display line line line line");
    //}
    //
    distSense();  
    //convert cm to minutes between 5 and 45 in 5 minute increments 
    minutes = ((average/3)*5)+5;
    minutes = constrain(minutes, 5, 45);
    //for troubleshooting
//    if (minutes == 5){
//      minutes = 1;
//    }

    Multiplex7Seg::loadValue(minutes);

    //check output on serial monitor
    //  Serial.print(average);
    //  Serial.print(" , ");
//    Serial.print(minutes);
//    Serial.println(" minutes");
//    delay(1);
    //set medTime to minutes selected
    medTime = minutes;
  } 
  else{
    //if cap sense is not touched
    //  Serial.print("Time set for ");
    //  Serial.print(medTime);
    //  Serial.println(" minutes. Waiting for rotation to initiate clock.");
    if (mode == 1){
      mode = 2;
//      Serial.print("Timer set for ");
//      Serial.print(medTime);
//      Serial.println(" minutes. Waiting for rotation.");
      Multiplex7Seg::loadValue(medTime);
    }

  }

  //START CLOCK if rotation is detected

  if(abs(GyZ) > 1000){
    //if the clock is set but not running
    if(mode == 2){
      startMil = millis() ;
//      Serial.print("STARTING CLOCK at millis ");
//      Serial.println(startMil);
      //turn number display off

      mode = 3;
    }
  }

  //COUNTDOWN IF clock was started
  if(mode == 3){
    if(curMil-startMil > medTime*60000){
      //reset to off when time is up
      mode = 0;
      Multiplex7Seg::loadValue(0);
//      Serial.println("Meditation over. Sounding the chime.");
//      motor.write(120);
//      delay(15);
//      motor.write(60);
//      delay(50);
//      motor.write(150);
//      delay(50);
int pos;      
for(pos = 150; pos >= 50; pos -= 10) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    motor.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
for(pos = 50; pos <= 150; pos+=10)     // goes from 180 degrees to 0 degrees 
  {                                
    motor.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
    } 
    else {
      curMil = millis();
      timeLeft = medTime*60000 - (curMil - startMil);
      minLeft = (timeLeft/60000) + 1;
      Multiplex7Seg::loadValue(minLeft);
//      Serial.print("Counting down. ");
//      Serial.print(minLeft);
//      Serial.print(" minutes and ");
//      Serial.print(timeLeft);
//      Serial.println(" millis remaining.");
    }
  }

}

int distSense(){
  //check time-setting sensor 
  unsigned int uS = sonar.ping_cm(); // Send sonar ping, return distance in cm
  //smooth the signal
  // subtract the last reading:
  total= total - readings[readIndex];         
  // read from the sensor:  
  readings[readIndex] = uS;
  // add the reading to the total:
  total= total + readings[readIndex];       
  // advance to the next position in the array:  
  readIndex = readIndex + 1;                    
  // if we're at the end of the array...
  if (readIndex >= numReadings)              
    // ...wrap around to the beginning: 
    readIndex = 0;                           
  // calculate the average:
  average = total / numReadings;   
  return(average);
}








