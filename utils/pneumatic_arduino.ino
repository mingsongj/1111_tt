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
    String msg = Serial.readString();
    delay(100);
   // Serial.print("Received: ");
    //Serial.println(msg);
    //Serial.println("ACK");
    if (msg == "BLINK"){
      unsigned long start_time = millis();
      while (millis() - start_time < time_out){
        Serial.println("Hello from Arduino! Turn LED ON");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        Serial.println("Turn LED OFF");
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
      }
      Serial.println("Stopped Blinking");
    }
    else if (msg == "JAM"){
      layerJam();
    }
    else if (msg == "UNJAM"){
      releaseJam();
    }
    else if (msg == "INFLATE"){
      inflation();
    }
    else if (msg == "DEFLATE"){
      releasePouch();
    }
    else if (msg == "FLOAT"){
      floating(float_duration);
    }
    else if (msg == "SINK"){
      sinking(sink_duration);
    }
    Serial.print(msg);
    Serial.println(" complete!");
    
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
  
  while(elapsedTime < 10000)
  {
    analogWrite(PUMP, 255);
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
  
  while(elapsedTime < 10000)
  {
    analogWrite(PUMP, 255);
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }

  analogWrite(PUMP, 0);
  digitalWrite(XV1, LOW);
}

// Deflate the pouches
void deflate()
{
  digitalWrite(XV1, HIGH);
  digitalWrite(XV3, HIGH);
  digitalWrite(XV4, HIGH);

  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  
  while(elapsedTime < 10000)
  {
    analogWrite(PUMP, 255);
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
  
  while(elapsedTime < 5000)
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
void sink()
{
  digitalWrite(BXV2, HIGH);

  startTime = millis();
  currentTime = millis();
  elapsedTime = currentTime - startTime;
  
  while(elapsedTime < 10000)
  {
    analogWrite(BPUMP, 130);
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
  
  while(elapsedTime < 100000)
  {
    analogWrite(BPUMP, 130);
    currentTime = millis();
    elapsedTime = currentTime - startTime;
  }

  analogWrite(BPUMP, 0);
  digitalWrite(BXV1, LOW);
}