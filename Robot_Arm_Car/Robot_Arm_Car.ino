#include <Ps3Controller.h>
#include <ESP32Servo.h>

#define SERVO_FORWARD_STEP_ANGLE 1
#define SERVO_BACKWARD_STEP_ANGLE -1

struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;  
};
std::vector<ServoPins> servoPins = 
{
  { Servo(), 27 , "Base", 90},
  { Servo(), 26 , "Shoulder", 90},
  { Servo(), 25 , "Elbow", 90},
  { Servo(), 33 , "Gripper", 90},
};

bool gripperSwitch = false;

//Right motor
int enableRightMotor=22; 
int rightMotorPin1=16;
int rightMotorPin2=17;
//Left motor
int enableLeftMotor=23;
int leftMotorPin1=18;
int leftMotorPin2=19;

#define MAX_MOTOR_SPEED 200  //Its value can range from 0-255. 255 is maximum speed.

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;

void writeServoValues(int servoIndex, int servoMoveStepSize, bool servoStepSizeIsActualServoPosition = false)
{
  int servoPosition;
  if (servoStepSizeIsActualServoPosition)
  {
    servoPosition = servoMoveStepSize; 
  }
  else
  {
    servoPosition = servoPins[servoIndex].servo.read();
    servoPosition = servoPosition + servoMoveStepSize;
  }
  if (servoPosition > 180 || servoPosition < 0)
  {
    return;
  }

  servoPins[servoIndex].servo.write(servoPosition);   
}


void notify()
{
  int rx =(Ps3.data.analog.stick.rx);  //Base       =>  Right stick - x axis
  int ry =(Ps3.data.analog.stick.ry);  //Shoulder   =>  Right stick  - y axis
  int ly =(Ps3.data.analog.stick.ly);  //Elbow      =>  Left stick  - y axis 
  int lx =(Ps3.data.analog.stick.lx);  //Gripper    =>  Left stick - x axis

  if (rx > 50)
  {
    writeServoValues(0, SERVO_BACKWARD_STEP_ANGLE);  
  }
  else if (rx < -50)
  {
    writeServoValues(0, SERVO_FORWARD_STEP_ANGLE);  
  }

  if (ry > 50)
  {
    writeServoValues(1, SERVO_BACKWARD_STEP_ANGLE);  
  }
  else if (ry < -50)
  {
    writeServoValues(1, SERVO_FORWARD_STEP_ANGLE);  
  }

  if (ly > 50)
  {
    writeServoValues(2, SERVO_FORWARD_STEP_ANGLE);  
  }
  else if (ly < -50)
  {
    writeServoValues(2, SERVO_BACKWARD_STEP_ANGLE);  
  }

  if (lx > 50)
  {
    writeServoValues(3, SERVO_BACKWARD_STEP_ANGLE);  
  }
  else if (lx < -50)
  {
    writeServoValues(3, SERVO_FORWARD_STEP_ANGLE);  
  }

  if (Ps3.event.button_down.r2)
  {
    gripperSwitch = !gripperSwitch;  //Toggle gripper close / open
    gripperSwitch ? writeServoValues(3, 170, true) :  writeServoValues(3, 100, true) ;
  }

  if (Ps3.data.button.up)             //Move car Forward
  {
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (Ps3.data.button.down)      //Move car Backward
  {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (Ps3.data.button.right)     //Move car Right
  {
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (Ps3.data.button.left)      //Move car Left
  {
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else                                //Stop the car
  {
    rotateMotor(0, 0);
  } 
    
  delay(10);
  
}

void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }
  
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }  
}

void setUpPinModes()
{
  for (int i = 0; i < servoPins.size(); i++)
  {
    servoPins[i].servo.attach(servoPins[i].servoPin);
    servoPins[i].servo.write(servoPins[i].initialPosition);    
  }

  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  //Set up PWM for motor speed
  ledcSetup(PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, PWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, PWMSpeedChannel);  
  ledcWrite(PWMSpeedChannel, MAX_MOTOR_SPEED);
  
  rotateMotor(0, 0);  
}


void setup()
{
  setUpPinModes();
  Serial.begin(115200);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisConnect);
  Ps3.begin();
  Serial.println("Ready.");
}

void loop()
{
}
