#include <Servo.h>

Servo roller;

#define TOP_ROLLER_ENCODER 2
volatile int pulseCount = 0;
const float PULSE_PER_REV = 7.0;


double feedback, setpoint, control;
double Kp, Ki, Kd;
int T;
unsigned long lastTime;
double totalErr, lastErr;
int controlLimit;

void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);

  roller.attach(9);
  roller.writeMicroseconds(1500);

  delay(5000);

  pinMode(TOP_ROLLER_ENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(TOP_ROLLER_ENCODER), countPulse, RISING);

  Kp = 0.1;
  Ki = 0.001;
  Kd = 0;

  T = 25;

  setpoint = 1000;
  lastTime = 0;
  totalErr = 0;
  lastErr = 0;
  controlLimit = 500; // 0-500 -> -500 - +500 -> 1000 - 2000
}

void loop() {

  // roller.writeMicroseconds(1560);
  // delay(2000);
  // roller.writeMicroseconds(1500);
  // delay(2000);

  // copy the pulse count value
  noInterrupts();
  int pulseCounted = pulseCount;
  pulseCount = 0;
  interrupts();

  float rev = pulseCounted / PULSE_PER_REV;
  float rpm = rev / (T / 60000.);

  // // Serial.println(rpm);
  // // delay(T);

  if (Serial.available()) {
    setpoint = Serial.parseInt();
  }

  feedback = rpm;

  // unsigned long now = millis();

  // if (now - lastTime >= T) {

  double err = setpoint - feedback;

  totalErr += err;
  totalErr = constrain(totalErr, -controlLimit, controlLimit);

  double deltaErr = err - lastErr;

  control = Kp*err + (Ki*T)*totalErr + (Kd/T)*deltaErr;

  control = constrain(control, -controlLimit, controlLimit);

  lastErr = err;
  // lastTime = millis();
  
  // }

  roller.writeMicroseconds((int)control + 1500);
  // Serial.println((int)control + 1500);

  delay(T);
}
