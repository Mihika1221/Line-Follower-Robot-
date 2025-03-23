#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
int P;
int I;
int D;
int lastError = 0;
bool onoff = false;  // Use bool for clarity

const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

const int mode = 8;
const int aphase = 9;
const int aenbl = 6;
const int bphase = 5;
const int benbl = 3;

const int buttoncalibrate = 17; // or pin A3
const int buttonstart = 2;

void setup() {
  // Initialize QTR sensor settings
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10, 11, 12, 14, 15, 16, 18, 19}, SensorCount);
  qtr.setEmitterPin(7); // LED ON pin

  // Set motor control pins as outputs
  pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  digitalWrite(mode, HIGH);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Set button pins as inputs
  pinMode(buttoncalibrate, INPUT);
  pinMode(buttonstart, INPUT);

  // Wait for the calibration button to be pressed
  bool calibrated = false;
  while (!calibrated) { 
    if (digitalRead(buttoncalibrate) == HIGH) {
      calibration(); 
      calibrated = true;
    }
    delay(10); // Small delay for debounce
  }
  forward_brake(0, 0); // Stop the motors
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // Toggle motor state when the start button is pressed (with debounce)
  if (digitalRead(buttonstart) == HIGH) {
    onoff = !onoff;
    delay(200); // Debounce delay
  }
  
  if (onoff) {
    PID_control();
  } else {
    forward_brake(0, 0);
  }
}

void forward_brake(int posa, int posb) {
  digitalWrite(aphase, LOW);
  digitalWrite(bphase, LOW);
  analogWrite(aenbl, posa);
  analogWrite(benbl, posb);
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues); 
  int error = 3500 - position; 

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;
  
  int motorspeed = (int)(P * Kp + I * Ki + D * Kd);
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  // Constrain motor speeds within allowed limits
  motorspeeda = constrain(motorspeeda, 0, maxspeeda);
  motorspeedb = constrain(motorspeedb, 0, maxspeedb);
  
  forward_brake(motorspeeda, motorspeedb);
}
