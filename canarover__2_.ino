#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <DFRobot_OxygenSensor.h>


// ---------------- PINS ----------------

// L298N motor driver (remove the ENA and ENB jumpers on the board)
int ENA = 5;    // left side speed
int IN1 = 22;
int IN2 = 23;
int IN3 = 24;
int IN4 = 25;
int ENB = 6;    // right side speed

int servoPin = 9;     // arm that moves the sensors up and down

int h2sPin = A0;      // MQ-136 (hydrogen sulphide)
int ch4Pin = A1;      // MQ-4 (methane)

int buzzer = 30;
int greenLed = 31;
int yellowLed = 32;
int redLed = 33;

// Serial1 = HC-05 for my phone     (TX1 pin 18, RX1 pin 19)
// Serial2 = HC-05 for the fan box  (TX2 pin 16, RX2 pin 17)
// Serial3 = MH-Z19 CO2 sensor      (TX3 pin 14, RX3 pin 15)
// O2 sensor and BME280 both go on SDA pin 20 and SCL pin 21
// HC-05 RX pins are 3.3V: use a voltage divider from the Mega's TX pins


// ---------------- SETTINGS ----------------
// change these after testing

int motorSpeed = 180;   // 0 to 255. my motors are 3-6V so don't go too high

// FIX 1: if no command arrives for this long, the rover stops by itself
// (works when the phone app keeps sending the letter while a button is held)
unsigned long commandTimeout = 1000;   // milliseconds

int armDown = 10;       // servo angle when arm is down (for heavy gases)
int armUp = 170;        // servo angle when arm is up (for light gases)

// MQ sensors give a raw number from 0 to 1023, NOT ppm.
// log clean air first, then set these a bit above normal.
int h2sDanger = 400;
int h2sSafe = 300;
int ch4Danger = 400;
int ch4Safe = 300;

// FIX 2: the check is now ">=" so a sensor stuck at the top of its range
// still counts as danger. co2Danger must be at or below your sensor's
// max range (MH-Z19B default range is 5000 ppm).
int co2Danger = 5000;   // ppm
int co2Safe = 2000;     // ppm

float o2Danger = 19.5;  // below this = not enough oxygen
float o2Safe = 20.0;

unsigned long warmUpTime = 180000;  // 3 minutes for the MQ and CO2 sensors to heat up
unsigned long dangerWait = 10000;   // danger has to last 10 seconds before the fan starts


// ---------------- OTHER STUFF ----------------

Servo arm;
Adafruit_BME280 bme;
DFRobot_OxygenSensor oxygen;

int h2s = 0;
int ch4 = 0;
int co2 = 0;
float o2 = 0;
float temp = 0;
float humidity = 0;

// FIX 6: remember if a sensor failed to start
bool bmeOk = false;
bool o2Ok = false;

bool danger = false;
bool fanOn = false;
bool autoFan = false;       // fan only works on its own after I send 'A' from the phone
unsigned long dangerStart = 0;

unsigned long lastRead = 0;
unsigned long lastFanMsg = 0;
unsigned long lastCmd = 0;  // FIX 1: time of the last phone command
bool beepState = false;


void setup() {
  Serial.begin(9600);     // USB, so I can see readings on the laptop while testing
  Serial1.begin(9600);    // phone
  Serial2.begin(9600);    // fan box
  Serial3.begin(9600);    // CO2 sensor

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(buzzer, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(redLed, OUTPUT);

  arm.attach(servoPin);
  arm.write(armDown);

  Wire.begin();

  // some BME280 boards use 0x77 instead of 0x76
  bmeOk = bme.begin(0x76);
  if (!bmeOk) {
    Serial.println("BME280 not found, check wires");
  }

  o2Ok = oxygen.begin(ADDRESS_3);
  if (!o2Ok) {
    Serial.println("O2 sensor not found, check wires");
  }

  stopCar();
  Serial.println("CanaRover ready");
}


void loop() {
  // check the phone every time so driving never feels slow
  readPhone();

  // FIX 1: no command for a while = stop (for example if Bluetooth drops)
  if (millis() - lastCmd > commandTimeout) {
    stopCar();
  }

  // read the sensors once every second
  if (millis() - lastRead >= 1000) {
    lastRead = millis();

    readSensors();

    // MQ sensors give wrong high numbers while warming up, so wait
    if (millis() > warmUpTime) {
      checkDanger();
      controlFan();
    }

    updateLights();
    sendReadings();
  }

  // tell the fan box what to do twice a second
  // (if the fan box stops hearing this, it turns the fan on by itself)
  if (millis() - lastFanMsg >= 500) {
    lastFanMsg = millis();
    if (fanOn) {
      Serial2.print('1');
    } else {
      Serial2.print('0');
    }
  }
}


// ---------------- PHONE ----------------

void readPhone() {
  // read every waiting letter so old commands don't pile up
  while (Serial1.available()) {
    char c = Serial1.read();

    // ignore line endings some apps send
    if (c == '\n' || c == '\r') continue;

    lastCmd = millis();   // FIX 1

    if (c == 'F') forward();
    if (c == 'B') backward();
    if (c == 'L') turnLeft();
    if (c == 'R') turnRight();
    if (c == 'S') stopCar();

    if (c == 'U') arm.write(armUp);
    if (c == 'D') arm.write(armDown);

    // FIX 3: two separate commands instead of one toggle
    if (c == 'A') autoFan = true;
    if (c == 'M') autoFan = false;
  }
}

// FIX 8: print piece by piece instead of building a String
void printReadings(Print &out) {
  out.print("H2S:");
  out.print(h2s);
  out.print(" CH4:");
  out.print(ch4);
  out.print(" CO2:");
  out.print(co2);

  out.print(" O2:");
  if (o2Ok) {
    out.print(o2, 1);
  } else {
    out.print("FAULT");
  }

  if (bmeOk) {
    out.print(" T:");
    out.print(temp, 1);
    out.print(" H:");
    out.print(humidity, 0);
  } else {
    out.print(" BME:FAULT");
  }

  if (millis() < warmUpTime) {
    out.print(" WARMING UP");
  } else if (danger) {
    out.print(" DANGER");
  } else {
    out.print(" SAFE");
  }

  if (fanOn) {
    out.print(" FAN:ON");
  } else {
    out.print(" FAN:OFF");
  }

  if (autoFan) {
    out.println(" AUTO:ON");
  } else {
    out.println(" AUTO:OFF");
  }
}

void sendReadings() {
  printReadings(Serial);    // laptop
  printReadings(Serial1);   // phone
}


// ---------------- SENSORS ----------------

// FIX 9: read the pin twice and keep the second value,
// so the last pin's reading doesn't leak into this one
int readAnalogClean(int pin) {
  analogRead(pin);
  delayMicroseconds(100);
  return analogRead(pin);
}

void readSensors() {
  h2s = readAnalogClean(h2sPin);
  ch4 = readAnalogClean(ch4Pin);
  co2 = readCO2();

  // FIX 4: average of the last 3 readings, so it reacts in about 3 seconds
  if (o2Ok) {
    o2 = oxygen.getOxygenData(3);
  }

  if (bmeOk) {
    temp = bme.readTemperature();
    humidity = bme.readHumidity();
  }
}

// asks the MH-Z19 for a reading and gets back 9 bytes
int readCO2() {
  byte ask[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  byte answer[9];

  // throw away any old bytes
  while (Serial3.available()) {
    Serial3.read();
  }

  Serial3.write(ask, 9);

  // wait for the answer, but not forever
  unsigned long startTime = millis();
  while (Serial3.available() < 9) {
    if (millis() - startTime > 100) {
      return co2;   // no answer, keep the last value
    }
  }

  for (int i = 0; i < 9; i++) {
    answer[i] = Serial3.read();
  }

  // a good answer always starts with FF 86
  if (answer[0] != 0xFF || answer[1] != 0x86) {
    return co2;
  }

  // FIX 5: check the checksum in the last byte
  byte sum = 0;
  for (int i = 1; i < 8; i++) {
    sum += answer[i];
  }
  sum = 0xFF - sum + 1;
  if (answer[8] != sum) {
    return co2;   // garbled answer, keep the last value
  }

  return answer[2] * 256 + answer[3];
}


// ---------------- DANGER AND FAN ----------------

void checkDanger() {
  // is anything bad?
  bool somethingBad = false;
  if (h2s > h2sDanger) somethingBad = true;
  if (ch4 > ch4Danger) somethingBad = true;
  if (co2 >= co2Danger) somethingBad = true;   // FIX 2
  if (o2Ok && o2 < o2Danger) somethingBad = true;
  if (!o2Ok) somethingBad = true;   // FIX 6: no oxygen reading = not safe

  // is everything back to safe?
  bool allSafe = true;
  if (h2s > h2sSafe) allSafe = false;
  if (ch4 > ch4Safe) allSafe = false;
  if (co2 > co2Safe) allSafe = false;
  if (!o2Ok || o2 < o2Safe) allSafe = false;

  // danger starts when something goes bad...
  if (danger == false && somethingBad) {
    danger = true;
    dangerStart = millis();
  }

  // ...and only ends when everything is safe again
  // (two different limits stop it flicking on and off)
  if (danger == true && allSafe) {
    danger = false;
  }
}

void controlFan() {
  if (autoFan == false) {
    fanOn = false;
    return;
  }

  // wait a bit before starting the fan, so people can see the gas first
  if (danger && millis() - dangerStart > dangerWait) {
    fanOn = true;
  }

  if (danger == false) {
    fanOn = false;
  }
}

void updateLights() {
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  digitalWrite(redLed, LOW);
  digitalWrite(buzzer, LOW);

  if (millis() < warmUpTime) {
    digitalWrite(yellowLed, HIGH);   // still warming up
  } else if (danger) {
    digitalWrite(redLed, HIGH);
    // beep on and off every second
    beepState = !beepState;
    digitalWrite(buzzer, beepState ? HIGH : LOW);
  } else {
    digitalWrite(greenLed, HIGH);
  }
}


// ---------------- DRIVING ----------------
// if one side spins the wrong way, swap that motor's two wires

void forward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
