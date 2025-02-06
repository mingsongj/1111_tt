//  10/03/2023 Jiefeng sun, Billy Yang, Estaban, and Erick
//  Add the sensing and break in the code, it will break the inflation or jamming when "0"
// Esteban will complete the `timeout` modification - DONE

// Esteban will complete the `timeout` modification

// archived this -- working version
#include <Adafruit_INA260.h>
Adafruit_INA260 ina260 = Adafruit_INA260();


String command_type = "";
int timeout = 0;
bool sensor_flag = 0; 


// Flipper Control Pinout Assignments
const int PUMP = 3;
const int XV1 = 12;
const int XV2 = 11;
const int XV3 = 10;
const int XV4 = 9;
// Buoyancy Control Pinout Assignments
const int BPUMP = 5;
const int BXV1 = 8;
const int BXV2 = 7;
// Pressure Sensor Assignment
const double IPS = A7;    // Inflation sensor (15 PSI)
const double LJPS = A2;   // Layer Jamming sensor (30 PSI)

bool hasRun = false;

unsigned long startTime, currentTime, elapsedTime;

// Kalman Variables
float SensorData, KalmanFilterData; 
float Xt, Xt_update, Xt_prev; 
float Pt, Pt_update, Pt_prev; 
float Kt, R, Q;  

void setup() {
  Serial.begin(115200);

  // Kalman declarations
  // R should be 1 or 10 for balance between preserved data and reduced noise
  R=1; Q=0.1; Pt_prev=1; 

    // Wait until serial port is opened
    // Estaban add if sensor_flag == 0 don't check sensor
  while (!Serial) { delay(10); }
  Serial.println("Adafruit INA260 Test");
  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("after begin");

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

void loop() {
  // might have to remove this delay
  // it's supposed to give arduino time to prepare for a message from python
  delay(100);
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    delay(100);
    // Parse the command and timeout from the received message
    int commaIndex = msg.indexOf(',');
    if (commaIndex != -1) {
      command_type = msg.substring(0, commaIndex);
      timeout = msg.substring(commaIndex + 1).toInt();
      Serial.println(command_type);
      Serial.println(timeout);
    }
  }
    if (command_type == "b") {
     blink_test();
    }
    else if (command_type == "j") {
      jam(timeout);
    }
    else if (command_type == "u") {
      unjam(timeout);
    }
    else if (command_type == "i") {
      inflation(timeout);
    }
    else if (command_type == "d") {
      deflation(timeout);
    }
    else if (command_type == "f") {
      floatUp(timeout);
    }
    else if (command_type == "s") {
      sink(timeout);
    }
    else if (command_type == "k") {
      continuousInflate(timeout);
    }
    else if (command_type == "m") {
      continuousJam(timeout);
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

void blink_test() {
  unsigned long start_time = millis();
  while (!checkForBreak()) {
    Serial.println("Hello from Arduino! Turn LED ON");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    Serial.println("Turn LED OFF");
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void reset_command() {
  command_type = "";
  timeout = 0;
}

void power_sense_once() {
  // if fla == 0 skip the following: 
  float voltage_mV = ina260.readBusVoltage();
  float power_mW = ina260.readPower();
  // Concatenate voltage and power into a single string with a comma as a delimiter
  String sensorData = String(voltage_mV) + "," + String(power_mW);
  // Print the concatenated sensor datas
  Serial.println(sensorData);
  delay(100); // Adjust the delay as needed. Now at 10Hz
}

// Normal State of System
void normal() {
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
void inflation(int inflation_time ) {
  digitalWrite(XV2, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(PUMP, 255);

  bool stopInflation = false;

  while (elapsedTime < inflation_time  * 1000 && !checkForBreak()) {
    power_sense_once();
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }
  analogWrite(PUMP, 0);
  digitalWrite(XV2, LOW);
}

// Jam the layers
void jam(int jam_time) { 
  digitalWrite(XV1, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(PUMP, 255);

  while(elapsedTime < jam_time * 1000 && !checkForBreak()) {
    power_sense_once();
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }

  analogWrite(PUMP, 0);
  digitalWrite(XV1, LOW);
}

// Deflate the pouches
void deflation(int deflation_time) {
  digitalWrite(XV1, HIGH);
  digitalWrite(XV3, HIGH);
  digitalWrite(XV4, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;

  analogWrite(PUMP, 255);

  while(elapsedTime < deflation_time * 1000 && !checkForBreak()) {
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
void unjam(int unjam_time) {
  digitalWrite(XV2, HIGH);
  digitalWrite(XV3, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;

  while(elapsedTime < unjam_time* 1000 && !checkForBreak()) {
    analogWrite(PUMP, 255);
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }

  analogWrite(PUMP, 0);
  digitalWrite(XV2, LOW);
  digitalWrite(XV3, LOW);
}

// Sink the robot
void sink(int sink_time) {
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
void floatUp(int floatup_time) {
  digitalWrite(BXV1, HIGH);
  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  analogWrite(BPUMP, 130);

  while(elapsedTime < floatup_time * 1000 && !checkForBreak()) {
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }

  analogWrite(BPUMP, 0);
  digitalWrite(BXV1, LOW);
}

void continuousInflate(float pressure) {
  Serial.println("Entering Continous Inflation Mode.");
  while (true) {
    float currentPressure = kalman(readPositivePressure());
    Serial.print("Current Positive Pressure: ");
    Serial.println(currentPressure);

    if (currentPressure < pressure) {
      Serial.println("Inflating to reach pressure threshold.");
      analogWrite(PUMP, 255);
    } else {
      analogWrite(PUMP, 0);
      Serial.println("Pressure is adequate. No inflation needed.");
    }

    if(checkForBreak()) {
      Serial.println("Exiting continous inflation mode.");
      break;
    }
    delay(500);
  }
  digitalWrite(PUMP, LOW);
}


void continuousJam(float pressure) {
  Serial.println("Entering Continous Jamming Mode.");
  while (true) {
    float currentPressure = kalman(readNegativePressure());
    Serial.print("Current Negative Pressure: ");
    Serial.println(currentPressure);

    if (currentPressure < pressure) {
      Serial.println("Jamming to reach pressure threshold.");
      analogWrite(PUMP, 255);
    } else {
      analogWrite(PUMP, 0);
      Serial.println("Pressure is adequate. No jamming needed.");
    }

    if(checkForBreak()) {
      Serial.println("Exiting continous jamming mode.");
      break;
    }
    delay(500);
  }
  digitalWrite(PUMP, LOW);
}

// Reads positive pressure
float readPositivePressure() {
  int readPres = analogRead(IPS);
  return posPressureCalc(readPres);
}

// Reads negative pressure
float readNegativePressure() {
  int readPres = analogRead(LJPS);
  return negPressureCalc(readPres);
}

// Converts analog read values into PSI
float posPressureCalc (int rawData) {
  return ((0.0275 * rawData) + (-2.05));
}

// Converts analog read values into PSI
float negPressureCalc (int rawData) {
  return ((0.0529 * rawData) + (-3.58));
}

float kalman(float sensorData)
{ 
  Xt_update = Xt_prev;   
  Pt_update = Pt_prev + Q;    
  Kt = Pt_update / (Pt_update + R);   
  Xt = Xt_update + ( Kt * (sensorData - Xt_update));   
  Pt = (1 - Kt) * Pt_update;    
  Xt_prev = Xt;   
  Pt_prev = Pt;    
  KalmanFilterData=Xt;      
  
  delayMicroseconds(100);
  return KalmanFilterData; 
}
