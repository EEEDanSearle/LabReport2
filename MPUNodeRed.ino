//****************************************//
//* ESP32 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* ESP32 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Add your required sensor/component libraries here
// --
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);
#define LED 2
#define I2C_SLAVE_ADDR 0x04
// --

// Replace the next variables with your SSID/Password combination
const char* ssid = "networkb15";                      //CHANGE ME
const char* password = "rtfmrtfm0";              //CHANGE ME     

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";                      
const char* mqtt_server = "192.168.2.1";          //CHANGE ME

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
int x,y,z;
int StartCount;



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

long CalculateDistance(int clicks)  //Define Function Calculate Distance
{
  float diameter = 6,distance;   //Define variable diameter and distance
  distance = (clicks*((M_PI*diameter)/24));   //Calculates distance by piD and dividing by clicks per revolution of the wheel
  return(distance);  // Returns calculated value
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  Wire.begin();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  
  

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println(); 

  // Add your subscribed topics here i.e. statements to control GPIOs with MQTT
  // --

  if (String(topic)=="esp32/leftMotor")
  { 
   x = messageTemp.toInt();
  }

  if (String(topic)=="esp32/rightMotor")
  { 
   y = messageTemp.toInt();
  }

  if (String(topic)=="esp32/servo")
  { 
   z = messageTemp.toInt();
  }
  
  // --
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      
      // Add your subscribe topics here
      // --
      client.subscribe("esp32/servo");
      client.subscribe("esp32/leftMotor");
      client.subscribe("esp32/rightMotor");
      // --
         
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 500) {
    lastMsg = now;
    
    // Add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    // --
    


    ChangeDirection(x,y,z);

    mpu6050.update();                  //Receive updated information from MPU
    float angle = mpu6050.getAngleZ();  //Take the "Z axis" angle and store within angle
    Serial.print("Angle = ");  //Print to Serial for Error checking
    Serial.println(angle);
    
    char tempString[8];               //Create a temporary string
    dtostrf(angle,1,2,tempString);    //Turn the MPU angle from a float to a string
    client.publish("esp32/angle",tempString);  //Publish the string to the "angle" topic
    
    
    char tempCount1[10];
    int CountA = rotaryCount(1);
    int CountB = rotaryCount(2);
    int Count1 = (CountA+CountB)/2;

    sprintf(tempCount1,"%d",Count1);
    client.publish("esp32/RotaryEncoder",tempCount1);

    Count1 = CalculateDistance(Count1);
    sprintf(tempCount1,"%d",Count1);
    client.publish("esp32/Distance",tempCount1);
   


    // --
  }
}
