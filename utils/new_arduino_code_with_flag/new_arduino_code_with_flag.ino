//  10/03/2023 Jiefeng sun, Billy Yang, Estaban, and Erick
//  Add the sensing and break in the code, it will break the inflation or jamming when "0"
// Esteban will complete the `timeout` modification - DONE

#include <Adafruit_INA260.h>
Adafruit_INA260 ina260 = Adafruit_INA260();

String command_type = "";

int timeout = 0;

bool sensor_flag = 0;

const int PUMP = 10;
const int XV1 = 5;
const int XV2 = 4;
const int XV3 = 3;
const int XV4 = 2;

// Buoyancy Control Pinout Assignments
const int BPUMP = 9;
const int BXV1 = 7;
const int BXV2 = 6;

bool hasRun = false;

unsigned long startTime, currentTime, elapsedTime;

void setup() {
  Serial.begin(115200);
    // Wait until serial port is opened
    // Estaban add if sensor_flag == 0 don't check sensor
//   if (sensor_flag != 0)
//   {
  while (!Serial) { delay(10); }
  Serial.println("Adafruit INA260 Test");
  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
//    while (1);
  }
  //}

  //Serial.println("Found INA260 chip");
  pinMode(PUMP, OUTPUT);
  pinMode(BPUMP, OUTPUT);
  pinMode(XV1, OUTPUT);
  pinMode(XV2, OUTPUT);
  pinMode(XV3, OUTPUT);
  pinMode(XV4, OUTPUT);
  pinMode(BXV1, OUTPUT);
  pinMode(BXV2, OUTPUT);
}
void loop()
{
  // might have to remove this delay
  // it's supposed to give arduino time to prepare for a message from python
  delay(100);
  if (Serial.available() >0){
    String msg = Serial.readStringUntil('\n');
    delay(100);
    // Parse the command and timeout from the received message
    int commaIndex1 = msg.indexOf(',');
    int commaIndex2 = msg.indexOf(',', commaIndex1 + 1);

    if (commaIndex1 != -1 && commaIndex2 != -1) {
      command_type = msg.substring(0, commaIndex1);
      timeout = msg.substring(commaIndex1 + 1, commaIndex2).toInt();
      sensor_flag = msg.substring(commaIndex2 + 1, commaIndex2 + 2).toInt();
      Serial.println(command_type);
      Serial.println(timeout);
      Serial.println(sensor_flag);
    }
  }

    if (command_type == "b"){
     blink_test();
      }
    //   Serial.println("Stopped Blinking");
    else if (command_type == "j"){
      jam(timeout);
    }
    else if (command_type == "u"){
      unjam(timeout);
    }
    else if (command_type == "i"){
      inflation(timeout);
    }
    else if (command_type == "d"){
      deflation(timeout);
    }
    else if (command_type == "f"){
      floatUp(timeout);
    }
    else if (command_type == "s"){
      sink(timeout);
    }
    reset_command();
    power_sense_once();
    normal();
}

bool checkForBreak() {
  while (Serial.available() > 0) {
    char input = Serial.read();
    if (input == '0') {
      return true;
    }
  }
  return false;
}
void blink_test(){
      unsigned long start_time = millis();
      while (!checkForBreak()){
        Serial.println("Hello from Arduino! Turn LED ON");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        Serial.println("Turn LED OFF");
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
      }
}
void reset_command(){
      command_type = "";
      timeout = 0;
}
void power_sense_once()
{
  // if fla == 0 skip the following:
  if (sensor_flag == 1)
  {
    float voltage_mV = ina260.readBusVoltage();
    float power_mW = ina260.readPower();
    // Concatenate voltage and power into a single string with a comma as a delimiter
    String sensorData = String(voltage_mV) + "," + String(power_mW);
    // Print the concatenated sensor data
    Serial.println(sensorData);
    delay(100); // Adjust the delay as needed. Now at 10Hz
  }
}
// Normal State of System
void normal()
{
  digitalWrite(XV1, LOW);
  digitalWrite(XV2, LOW);
  digitalWrite(XV3, LOW);
  digitalWrite(XV4, LOW);
  digitalWrite(BXV1, LOW);
  digitalWrite(BXV2, LOW);
  analogWrite(PUMP, 0);
  analogWrite(BPUMP, 0);
}
// Inflate the pouches
void inflation(int inflation_time )
{
  digitalWrite(XV2, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(PUMP, 255);
  bool stopInflation = false;
  while (elapsedTime < inflation_time  * 1000 && !checkForBreak())
  {
    power_sense_once();
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }
  analogWrite(PUMP, 0);
  digitalWrite(XV2, LOW);
}
// Jam the layers
void jam(int jam_time)
{
  digitalWrite(XV1, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(PUMP, 255);
  while(elapsedTime < jam_time * 1000 && !checkForBreak())
  {
    power_sense_once();
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }
  analogWrite(PUMP, 0);
  digitalWrite(XV1, LOW);
}
// Deflate the pouches
void deflation(int deflation_time)
{
  digitalWrite(XV1, HIGH);
  digitalWrite(XV3, HIGH);
  digitalWrite(XV4, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(PUMP, 255);
  while(elapsedTime < deflation_time * 1000 && !checkForBreak())
  {
    power_sense_once();
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }
  analogWrite(PUMP, 0);
  digitalWrite(XV1, LOW);
  digitalWrite(XV3, LOW);
  digitalWrite(XV4, LOW);
}
// Unjam the layers
void unjam(int unjam_time)
{
  digitalWrite(XV2, HIGH);
  digitalWrite(XV3, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  while(elapsedTime < unjam_time* 1000 && !checkForBreak())
  {
    analogWrite(PUMP, 255);
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }
  analogWrite(PUMP, 0);
  digitalWrite(XV2, LOW);
  digitalWrite(XV3, LOW);
}
// Sink the robot
void sink(int sink_time)
{
  digitalWrite(BXV2, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(BPUMP, 130);
  while(elapsedTime < sink_time * 1000 && !checkForBreak())
  {
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }
  analogWrite(PUMP, 0);
  digitalWrite(XV2, LOW);
}
// Float the robot
void floatUp(int floatup_time)
{
  digitalWrite(BXV1, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(BPUMP, 130);
  while(elapsedTime < floatup_time * 1000 && !checkForBreak())
  {
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }
  analogWrite(BPUMP, 0);
  digitalWrite(BXV1, LOW);
}
