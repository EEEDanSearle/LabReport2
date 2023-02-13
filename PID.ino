
#include <Wire.h>
#include <MPU6050_tockn.h> //MPU library
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define MPU_6050_ADDR 0x68 //MPU address
#define SpeedOfSound 0.0343 //Speed of sound in cm/microsecond

MPU6050 mpu6050(Wire);

const int trigPin = 15; //Define Trigger pin
const int echoPin = 2; //Define Echo pin

const int IR1 = 35; //Leftmost IR Sensor
const int IR2 = 32;
const int IR3 = 33;
const int IR4 = 25;
const int IR5 = 26;
const int IR6 = 27; //Rightmost IR sensor

//float lastError,previousErrors;
int Minimum[6] = {4095,4095,4095,4095,4095,4095}; 
int Maximum[6] = {0,0,0,0,0,0};


void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  //mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);
  pinMode(trigPin, OUTPUT);  //Define Trigger as output
  pinMode(echoPin,INPUT);  //Define Echo as input
  
  int sensorValue;  //Currently read value
  int IR[6] = {IR1,IR2,IR3,IR4,IR5,IR6};  //Array storing the pin of each sensor
  int i;  //loop variable

  while (millis() < 5000)     //Timer for 5 seconds
  {
     
    for (i=0;i<=5;i++)                   //i represents each sensor as seen in the IR array
    {                                    
    sensorValue = analogRead(IR[i]);     //Reads value from the sensor

    // record the maximum sensor value
    if (sensorValue > Maximum[i]) {      //If the value is greater than the highest currently recorded
      Maximum[i] = sensorValue;          //Set as the new maximum for that sensor
    }

    // record the minimum sensor value
    if (sensorValue < Minimum[i]) {     //If that sensor is lower than lowest currenty recorded
      Minimum[i] = sensorValue;         //Set as new minimum for that sensor
    }

    delay(50);
    }
  }
   
  
}

//Change values in loop to manouevre
int x = 0;  //Left Motor
int y = 0;  //Right Motor
int z = 110;  //Servo Angle



long CalculateDistance(uint16_t clicks)  //Define Function Calculate Distance
{
  float diameter = 6,distance;   //Define variable diameter and distance
  distance = (clicks*((M_PI*diameter)/24));   //Calculates distance by piD and dividing by clicks per revolution of the wheel
  return(distance);  // Returns calculated value
}

uint16_t rotaryCount(int EncNum)
{
  Wire.requestFrom(I2C_SLAVE_ADDR,4); //Requests Data from Arduino encoders
  uint16_t Enc1, Enc2;  //Encoder Speed
  uint8_t Enc1_169 = Wire.read();  //Receving encoder bits 16-9
  uint8_t Enc1_81 = Wire.read();  //Receiving encoder bits 8-1
  uint8_t Enc2_169 = Wire.read();
  uint8_t Enc2_81 = Wire.read();
  Enc1 = (Enc1_169 <<8) | Enc1_81;  //Combines two 8 bit numbers into a 16 bit
  Enc2 = (Enc2_169 <<8) | Enc2_81;

  switch(EncNum)
  {
    case 1:
    return(Enc1);
    case 2:
    return(Enc2);
  }
}

float UltrasonicDistance()
{
  float distance,duration; //Define function variables

  digitalWrite(trigPin,LOW); //Reset pin
  delay(1);  //delay of 1ms
  digitalWrite(trigPin,HIGH); //Sets to high for 10microseconds
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  duration = pulseIn(echoPin, HIGH); //Reads Echo pin for High Signal, returns in microseconds

  distance = duration*(SpeedOfSound/2); //Calculates Distance in cm
  return(distance); //Return calculated value to main
}

void ChangeDirection(int leftMotor, int rightMotor, int SteeringAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  
  //send leftMotorSpeed
  Wire.write((byte)((leftMotor & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  //Send rightMotorSpeed
  Wire.write((byte)((rightMotor & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  //send steering angle to arduino
  Wire.write((byte)((SteeringAngle & 0x0000FF00) >> 8));    
  Wire.write((byte)(SteeringAngle & 0x000000FF));
 
  Wire.endTransmission();   // stop transmitting
}

int findAngle()
{
  mpu6050.update();//get data from mpu
  int ZAngle = mpu6050.getAngleZ();  //Find Angle Z
  return(ZAngle);
}

void TurnClockwise(int Angle)
{

 int StartAngle = findAngle();
 Serial.println("StartAngle: ");
 Serial.print(StartAngle);
 int AngleTurned = (findAngle() - StartAngle);

 while (AngleTurned <= Angle-10)
 {
   ChangeDirection(200,200,170); //turn Right (clockwise)
   AngleTurned = (findAngle() - StartAngle);
    Serial.println("AngleTurned: ");
    Serial.print(AngleTurned);
    delay(50);
 }
 ChangeDirection(0,0,145); //Stop car
}

void TurnAntiClockwise(int Angle)
{
  int StartAngle = findAngle();
 Serial.println("StartAngle: ");
 Serial.print(StartAngle);
 int AngleTurned = (StartAngle - findAngle());

 while (AngleTurned <= Angle-10)
 {
   ChangeDirection(200,200,110); //turn left (anticlockwise)
   AngleTurned = (StartAngle - findAngle());
   // Serial.println("AngleTurned: ");
   // Serial.print(AngleTurned);
    delay(50);
 }
 ChangeDirection(0,0,145); //Stop car
}

void ReversePark(float distance)
{
  float SensorDistance = UltrasonicDistance();
  int StartAngle = findAngle();
  while (distance+16.5 <= SensorDistance)
  {
    int actualAngle = findAngle();
    Serial.print("actualangle: ");
    Serial.println(actualAngle);
    if (actualAngle != StartAngle)

    {
      int Turn = StartAngle - actualAngle;
      ChangeDirection(-255,-255,145-Turn);
    }
    else ChangeDirection(-255,-255,145);
    SensorDistance = UltrasonicDistance();
    Serial.print("Distance: ");
    Serial.println(SensorDistance);
    delay(50);
  }
  ChangeDirection(0,0,145); //stop car
}

void TravelForwards(float Time, int StartAngle)
{
  time_t start,end;
  float elapsed;
  time(&start);

  while (elapsed <= Time)
  {
    int actualAngle = findAngle();
    Serial.print("actualangle: ");
    Serial.println(actualAngle);
    if (actualAngle != StartAngle)

    {
      int Turn = StartAngle - actualAngle;
      ChangeDirection(255,255,145+Turn);
    }
    else ChangeDirection(255,255,145);
    
    time(&end);
    elapsed = difftime(end,start);
    delay(50);
   
  }
  ChangeDirection(0,0,145); //stop car
}

void TravelBackwards(float Time, int StartAngle)
{
  time_t start,end;
  float elapsed;
  time(&start);

  while (elapsed <= Time)
  {
    int actualAngle = findAngle();
    Serial.print("actualangle: ");
    Serial.println(actualAngle);
    if (actualAngle != StartAngle)

    {
      int Turn = StartAngle - actualAngle;
      ChangeDirection(-255,-255,145-Turn);
    }
    else ChangeDirection(-255,-255,145);
    
    time(&end);
    elapsed = difftime(end,start);
    delay(50);
   
  }
  ChangeDirection(0,0,145); //stop car
}

float ErrorAVG()
{
  int val1,val2,val3,val4,val5,val6;  //Define names of IR sensors
  float xavg,num,denom;  //Define Variables for calculating average

  val1 = map(analogRead(IR1),Minimum[0],Maximum[0],750,4095);   //Take Readings from each sensor
  val2 = map(analogRead(IR2),Minimum[1],Maximum[1],750,4095);   // Map each reading from 750 to 4095
  val3 = map(analogRead(IR3),Minimum[2],Maximum[2],750,4095);
  val4 = map(analogRead(IR4),Minimum[3],Maximum[3],750,4095);
  val5 = map(analogRead(IR5),Minimum[4],Maximum[4],750,4095);
  val6 = map(analogRead(IR6),Minimum[5],Maximum[5],750,4095);

  num = (-3.75*(val1-val6))+(-2.25*(val2-val5))+ (-0.75*(val3-val4));  //Calculate Numerator
  denom = val1+val2+val3+val4+val5+val6;                               //Calculate Denominator
  xavg = num/denom;           //Calculate the average 

  return(xavg);    //Return Average

}

float PIDControl(float currentError,float lastError,float previousErrors)
{
  float u;
  float Kp = 1.8; //proportional
  float Ki = 0.05; //integral
  float Kd = 0.1; //differential

  u = (Kp*currentError)+(Ki*previousErrors)+(Kd*(currentError-lastError));
  
  return(u);

}

void loop()
{
  char buffer[40];
  int val1, val2, val3, val4, val5, val6;

  float steerAngle,Error,PIDAngle;
  int leftSpeed = 100,rightSpeed = 100;
  int motorScale;
  static float previousError;
  static float sumOfErrors;



  val1 = analogRead(IR1);
  val2 = analogRead(IR2);
  val3 = analogRead(IR3);
  val4 = analogRead(IR4);
  val5 = analogRead(IR5);
  val6 = analogRead(IR6);

 //Serial.println("1\t2\t3\t4\t5\t6\t");
  sprintf(buffer,"%d\t %d\t %d\t %d\t %d\t %d\t",val1,val2,val3,val4,val5,val6);
  //Serial.println(buffer); 

  Error = ErrorAVG();
  //Serial.println(Error);
  
  PIDAngle = PIDControl(Error,previousError,sumOfErrors);
  
  steerAngle = 145-(45*PIDAngle);
  if (steerAngle>=180) steerAngle = 180;
  if (steerAngle <=110) steerAngle = 110;
  

  motorScale = 20*PIDAngle;
  if (motorScale >= 100) motorScale = 100;

  //Serial.println(PIDAngle);
 /* Serial.print("\nError: ");
  Serial.print(Error);
  Serial.print("\nLast Error: ");
  Serial.println(previousError);
  Serial.print("Sum: ");
  Serial.print(sumOfErrors);*/


  leftSpeed = leftSpeed - motorScale;
  rightSpeed = rightSpeed + motorScale;

  if (val1 == val2 && val2 == val3 && val3 == val4 && val4 == val5 && val5 == val6)
  {
    ChangeDirection(0,0,145);  // Bot Stops
    delay(200);                //Small Delay to give time to come to a stop
    ChangeDirection(-100,-100,145);  //Starts reversing
    delay(400);               //contines reversing for x amount of time
    ChangeDirection(0,0,145);  //Stops again
  }

  ChangeDirection(leftSpeed,rightSpeed,steerAngle);
  
  previousError = Error;
  sumOfErrors = sumOfErrors + Error;
  delay(50);
}
