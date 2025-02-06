//  10/03/2023 Jiefeng sun, Billy Yang, Estaban, and Erick
//  Add the sensing and break in the code, it will break the inflation or jamming when "0"

// #include <Adafruit_INA260.h>
// Adafruit_INA260 ina260 = Adafruit_INA260();

// Esteban will complete the `timeout` modification - DONE

String command_type = "";
int timeout = 0;
const int PUMP = 10;
const int XV1 = 5;
const int XV2 = 4;
const int XV3 = 3;
const int XV4 = 2;

// Buoyancy Control Pinout Assignments
const int BPUMP = 9;
const int BXV1 = 7;
const int BXV2 = 6;

// Sensors
// A6 = 15 PSI Pressure Sensor for Positive Pressure
// A7 = 30 PSI Pressure Sensor for Negative Pressure
const int posSense = A6;
const int negSense = A7;

bool hasRun = false;

unsigned long startTime, currentTime, elapsedTime;

void setup() {
  Serial.begin(115200);
    // Wait until serial port is opened
  // while (!Serial) { delay(10); }
  // Serial.println("Adafruit INA260 Test");
  // if (!ina260.begin()) {
  //   Serial.println("Couldn't find INA260 chip");
  //   while (1);
  // }

  Serial.println("Found INA260 chip");
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
    // see if you can pass the sensor flag as the first command: "0,i,10" means sensor_flag = 0
    int commaIndex = msg.indexOf(',');
    if (commaIndex != -1) {
      command_type = msg.substring(0, commaIndex);
      timeout = msg.substring(commaIndex + 1).toInt();
      Serial.println(command_type);
      Serial.println(timeout);
    }
  }
    if (command_type == "b"){
     blink_test();
      }
    //   Serial.println("Stopped Blinking");
    else if (command_type == "j"){
      jam();
    }
    else if (command_type == "u"){
      unjam();
    }
    else if (command_type == "i"){
      inflation();
    }
    else if (command_type == "d"){
      deflation();
    }
    else if (command_type == "f"){
      floatUp();
    }
    else if (command_type == "s"){
      sink();
    }
    reset_command();
     // power_sense_once();
     normal();
}

//bool checkForBreak() {
//  if (Serial.available() > 0) {
//    String input = Serial.readStringUntil('\n');
//    if (input == "0") {  // Check for "0" input
//      return true;
//    }
//  }
//  return false;
//}

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
    // float voltage_mV = ina260.readBusVoltage();
    // float power_mW = ina260.readPower();
    // Concatenate voltage and power into a single string with a comma as a delimiter
    // String sensorData = String(voltage_mV) + "," + String(power_mW);
    // Print the concatenated sensor data
    // Serial.println(sensorData);
    delay(100); // Adjust the delay as needed. Now at 10Hz
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
void inflation()
{
  digitalWrite(XV2, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(PUMP, 255);
  Serial.print("Inflate");
  bool stopInflation = false;
  long inflation_time = timeout;

  while (elapsedTime < timeout * 1000 && !checkForBreak())  
  {
    power_sense_once();
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }

  analogWrite(PUMP, 0);
  digitalWrite(XV2, LOW);
}

// Jam the layers
void jam()
{
  digitalWrite(XV1, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(PUMP, 255);

  while(elapsedTime < timeout * 1000 && !checkForBreak())
  {
    power_sense_once();
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }

  analogWrite(PUMP, 0);
  digitalWrite(XV1, LOW);
}

// Deflate the pouches
void deflation()
{
  digitalWrite(XV1, HIGH);
  digitalWrite(XV3, HIGH);
  digitalWrite(XV4, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(PUMP, 255);

  while(elapsedTime < timeout * 1000 && !checkForBreak())
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
void unjam()
{
  digitalWrite(XV2, HIGH);
  digitalWrite(XV3, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;

  // while(elapsedTime < timeout * 1000 && !checkForBreak())
  // {
  //   analogWrite(PUMP, 255);
  //   currentTime = millis();
  //   elapsedTime = currentTime - startTime;
  // }

  //analogWrite(PUMP, 0);
  digitalWrite(XV2, LOW);
  digitalWrite(XV3, LOW);
}

// Sink the robot
void sink()
{
  digitalWrite(BXV2, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(BPUMP, 130);

  while(elapsedTime < timeout * 1000 && !checkForBreak())
  {
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }
  analogWrite(PUMP, 0);
  digitalWrite(XV2, LOW);
}
// Float the robot
void floatUp()
{
  digitalWrite(BXV1, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(BPUMP, 130);

  while(elapsedTime < timeout * 1000 && !checkForBreak())
  {
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }

  analogWrite(BPUMP, 0);
  digitalWrite(BXV1, LOW);
}

// Reads positive pressure
float readPositivePressure()
{
  int readPres = analogRead(posSense);
  return pressureCalc(readPres, 15);
}

// Reads negative pressure
float readNegativePressure()
{
  int readPres = analogRead(negSense);
  return pressureCalc(readPres, 30);
}

// Converts analog read values into PSI
float pressureCalc (int rawData, int rating)
{
  return (rawData * (5.0 / 1023.0) - (0.1 * 5.0)) * (rating / (0.8 * 5.0));
}
