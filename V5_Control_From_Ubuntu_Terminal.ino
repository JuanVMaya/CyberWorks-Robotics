/*This program can subscribe to a topic and take a twist message input that is interpreted as a 
 * set of coordinates to achieve movement and turn on an LED
 */
#include <SPI.h>
#include <AccelStepper.h>

#include <ros.h>
#include <geometry_msgs/Twist.h> 

int chipSelectPin1= 8;
int chipSelectPin2= 9;
int chipSelectPin3= 10; //pins value may need to be changed. 
//end calibration values


/* Define stepper motor connections and steps per revolution:*/
#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 400 //For the NEMA 23 Motor we are using. Change to 400 when half stepping. 200 when fullstepping

AccelStepper stepper(1, stepPin, dirPin); // Set to 1 when using a driver
#define MAX_STEER_SPEED 600 //1700 initially. (reduced to 600).
#define MIN_STEER_SPEED 200
#define STEER_ACCEL 200 //1500 initially. (reduced to 200).

#define BUILTIN_LED 13
ros::NodeHandle  nh;
// Relay Pinout and Declaration
/* Manmode and Automode need to be both HIGH in order to
 *  have the robot move automatically, Deadmanual needs to be HIGH as
 *  well. To allow manual driving mode, set all of them LOW.
 */
#define AUTOMODE 44
#define MANUALMODE 40
#define FORWARD 34
#define REVERSE 30
#define STEERRELAY 36
#define BUMPER 3
#define DEADMANUAL 32

void setProportionalControl(const geometry_msgs::Twist &cmd_vel)
{
  //Setting motors relays ON to allow autonomous movement.
  digitalWrite(BUILTIN_LED,HIGH);
  Serial.println(cmd_vel.angular.z);
  delay(1000);
  digitalWrite(BUILTIN_LED,LOW);
}
//ROS Publisher/ Subscriber Set up

ros::Subscriber<geometry_msgs::Twist> cmd_vel("/cmd_vel", &setProportionalControl);
//*****************************************************
void setup() 
//*****************************************************
{
  Serial.begin(9600);
// Set up for Encoder Readings
  pinMode(chipSelectPin1, OUTPUT);
  pinMode(chipSelectPin2, OUTPUT);
  pinMode(chipSelectPin3, OUTPUT);
  
  digitalWrite(chipSelectPin1, HIGH);
  digitalWrite(chipSelectPin2, HIGH);
  digitalWrite(chipSelectPin3, HIGH);
 
 LS7366_Init();
//Set up for Motor microstepping
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
// AccelStep Library
  stepper.setMaxSpeed(1700);
  stepper.setAcceleration(500); 
  stepper.setEnablePin(24);
// ROS 
  pinMode(BUILTIN_LED, OUTPUT); 
  nh.initNode();
  nh.subscribe(cmd_vel);
  delay(100);
}

//*****************************************************
void loop() 
//*****************************************************
{
//Encoder
  long encoder1Value;
  long encoder2Value;
  long encoder3Value;
    
//  encoder1Value = getEncoderValue(1);  
//  Serial.print("Encoder X= ");
//  Serial.print(encoder1Value);
    
    encoder2Value = getEncoderValue(2);  
    Serial.print(" Encoder Steering= ");
    Serial.print(encoder2Value);
    
//  encoder3Value = getEncoderValue(3);  
//  Serial.print(" Encoder Z= ");
//  Serial.print(encoder3Value);

    Serial.print("\r\n");
    delay(100); 
    
//Microstepping with Twist Terminal Input
  nh.spinOnce(); 
  //Code that allows rotation depending on command_speed set by doSteering Function in Firmware Arduino
//  if(command_speed != 0)
//  {
//    // Disables acceleration
//    if (!ACCEL)
//      accel_coeff = MAX_STEER_SPEED * 2;
//
//    if (command_speed > current_speed + accel_coeff)
//      current_speed += accel_coeff;
//    else if (command_speed < current_speed - accel_coeff)
//      current_speed -= accel_coeff;
//    else
//      current_speed = command_speed;
//
//    stepper.setSpeed(command_speed);
//    stepper.runSpeed();
//  }
  delay(100);

  Serial.println("Out of the function");
    
 
}//end loop
void SteeringTest(){
  int step_speed = 500;
  int step_length = 2000;
  stepper.setSpeed(step_speed); //Set steps per second
  for (int i = 0; i < step_length*3; i++)
  {
    stepper.runSpeed();
    delayMicroseconds(20);
  }
}
void MoveMotor(int angle, int setDir){ //Set angle to move , Direction (1-CW) (0-CCW)
  float steps=0;
  Serial.println("Moving Motor");
// Set the spinning direction clockwise:
  if(setDir==1){
     digitalWrite(dirPin, HIGH);
  }
  // Set the spinning direction counter-clockwise:
  else if(setDir==-1){
     digitalWrite(dirPin, LOW);
  }
  
  steps=angle/360.0;  // Angle Range * stepsPerRevolution / Angle1Revolution(360)
  steps=steps * stepsPerRevolution;
  for (int i = 0; i < steps; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delay(10);
    digitalWrite(stepPin, LOW);
    delay(10);
  }
}

long getEncoderValue(int encoder)
{
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;
    
    selectEncoder(encoder);
    
     SPI.transfer(0x60); // Request count
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte
    deselectEncoder(encoder);
   
    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
    
    return result;
}//end func

//*************************************************
void selectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,LOW);
        break;
     case 2:
       digitalWrite(chipSelectPin2,LOW);
       break;
     case 3:
       digitalWrite(chipSelectPin3,LOW);
       break;    
  }//end switch
  
}//end func

//*************************************************
void deselectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
        digitalWrite(chipSelectPin1,HIGH);
        break;
     case 2:
       digitalWrite(chipSelectPin2,HIGH);
       break;
     case 3:
       digitalWrite(chipSelectPin3,HIGH);
       break;    
  }//end switch
  
}//end func



// LS7366 Initialization and configuration
//*************************************************
void LS7366_Init(void)
//*************************************************
{
   
    
    // SPI initialization
    SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);
   
   digitalWrite(chipSelectPin1,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin1,HIGH); 
   
   
   digitalWrite(chipSelectPin2,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin2,HIGH); 
   
   
   digitalWrite(chipSelectPin3,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin3,HIGH); 
   Serial.print("Encoder Shield Initialized Initialized");
}//end func
