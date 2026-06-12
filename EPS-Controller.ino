#include <Bluepad32.h>
#include <PID_v1.h>

#define DAC1 25
#define DAC2 26
#define AS1 14 // White Wire (TOP PIN)
#define AS2 13 // Red Wire (Bottom Pin)

#define MIN_DUTY 10.0f
#define MAX_DUTY 90.0f
#define DUTY_SPAN (MAX_DUTY - MIN_DUTY)
#define DEG_PER_PCT (180.0f / DUTY_SPAN)

#define AS1_CENTER 80.0f
#define AS1_CCW_POSITIVE false

#define AS2_CENTER 30.0f
#define AS2_CCW_POSITIVE true

#define KP 1.25f
#define KI 0.0f
#define KD 0.2f

#define TARGET_ANGLE 45.0f
#define MAX_OUTPUT 40.0f
#define ANGLE_DEADBAND 1.0f

#define MIN_MOTOR_OUTPUT 20.0f

const uint8_t centerV = 198;
const uint8_t v1 = 234;
const uint8_t v2 = 148;

uint8_t t1 = 198;
uint8_t t2 = 198;

volatile uint32_t riseTime1 = 0;
volatile uint32_t highTime1 = 0;
volatile uint32_t lowTime1 = 0;
volatile uint32_t lastFallTime1 = 0;
volatile bool newData1 = false;

volatile uint32_t riseTime2 = 0;
volatile uint32_t highTime2 = 0;
volatile uint32_t lowTime2 = 0;
volatile uint32_t lastFallTime2 = 0;
volatile bool newData2 = false;

float lastDuty1 = 80.0;
float lastDuty2 = 30.0;

float accumedAngle1 = 0;
float accumedAngle2 = 0;

bool intialized1 = true;
bool intialized2 = true;

float avgAngle = 0;

double pidSetpoint = TARGET_ANGLE;
double pidInput = 0;
double pidOutput = 0;

PID steeringPID(&pidInput, &pidOutput, &pidSetpoint, KP, KI, KD, DIRECT);

ControllerPtr controllers[BP32_MAX_GAMEPADS];

float dutyToAngleDelta(float newDuty, float oldDuty, bool ccwPositive) {
  float delta = newDuty - oldDuty;

  float halfSpan = DUTY_SPAN / 2.0;
  if (delta > halfSpan) delta -= DUTY_SPAN;
  if (delta < -halfSpan) delta += DUTY_SPAN;

  float angleDelta = delta * DEG_PER_PCT;
  if (!ccwPositive) angleDelta = -angleDelta;

  return angleDelta;
}

void updateSensor1(float duty) {
  accumedAngle1 += dutyToAngleDelta(duty, lastDuty1, AS1_CCW_POSITIVE);
  lastDuty1 = duty;
}

void updateSensor2(float duty) {
  accumedAngle2 += dutyToAngleDelta(duty, lastDuty2, AS2_CCW_POSITIVE);
  lastDuty2 = duty;
}

void applyTorque(double output) {
  if (abs(pidInput - pidSetpoint) < ANGLE_DEADBAND) {
    t1 = centerV;
    t2 = centerV;
    return;
  }

  if (output > 0 && output < MIN_MOTOR_OUTPUT) output = MIN_MOTOR_OUTPUT;
  if (output < 0 && output < -MIN_MOTOR_OUTPUT) output = -MIN_MOTOR_OUTPUT;

  t1 = constrain((int)centerV - (int)output, v2, v1);
  t2 = constrain((int)centerV + (int)output, v2, v1);
}

void onConnect(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
      controllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
}

void onDisconnect(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller disconnected, index=%d\n", i);
      controllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in controller list");
  }
}

void processGamepad(ControllerPtr ctl) {
  int x = ctl->axisX();

  if (abs(x) < 5) {
    t1 = centerV;
    t2 = centerV;
  }

  t1 = map(x, -512, 512, v1, v2);
  t2 = map(x, -512, 512, v2, v1);

  // printData(ctl);
}

void printData(ControllerPtr ctl) {
  Serial.printf("axis L: %4d, T1: %u, T2: %u\n",
                ctl->axisX(), t1, t2);
}

void processController() {
  for (auto controller : controllers) {
    if (controller && controller->isConnected() && controller->hasData()) {
      if (controller->isGamepad())
        processGamepad(controller);
    }
  }
}

void IRAM_ATTR isr1() {
  uint32_t now = micros();
  
  if (digitalRead(AS1)) {
    lowTime1 = now - lastFallTime1;
    riseTime1 = now;
  } else {
    highTime1 = now - riseTime1;
    lastFallTime1 = now;
    newData1 = true;
  }
}

void IRAM_ATTR isr2() {
  uint32_t now = micros();

  if (digitalRead(AS2)) {
    lowTime2 = now - lastFallTime2;
    riseTime2 = now;
  } else {
    highTime2 = now - riseTime2;
    lastFallTime2 = now;
    newData2 = true;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(AS1, INPUT);
  pinMode(AS2, INPUT);

  attachInterrupt(digitalPinToInterrupt(AS1), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(AS2), isr2, CHANGE);

  BP32.setup(&onConnect, &onDisconnect);

  BP32.forgetBluetoothKeys();

  BP32.enableVirtualDevice(false);

  steeringPID.SetOutputLimits(-MAX_OUTPUT, MAX_OUTPUT);
  steeringPID.SetSampleTime(20);
  steeringPID.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (newData1) {
    noInterrupts();
    uint32_t high = highTime1;
    uint32_t low = lowTime1;
    newData1 = false;
    interrupts();

    float duty1 = ((float)high / (high + low)) * 100.0;
    float freq1 = 1000000.0 / (high + low);

    if (duty1 >= MIN_DUTY && duty1 <= MAX_DUTY)
      updateSensor1(duty1);

    // Serial.print("AS1: ");
    // Serial.print(duty1);
    // Serial.print("% ");
    // Serial.print(freq1);
    // Serial.println(" Hz");
  }

  if (newData2) {
    noInterrupts();
    uint32_t high = highTime2;
    uint32_t low = lowTime2;
    newData2 = false;
    interrupts();

    float duty2 = ((float)high / (high + low)) * 100.0;
    float freq2 = 1000000.0 / (high + low);

    if (duty2 >= MIN_DUTY && duty2 <= MAX_DUTY)
      updateSensor2(duty2);

    // Serial.print("AS2: ");
    // Serial.print(duty2);
    // Serial.print("% ");
    // Serial.print(freq2);
    // Serial.println(" Hz");
  }

  if (intialized1 && intialized2) {
    avgAngle = (accumedAngle1 + accumedAngle2) / 2.0f;
    pidInput = avgAngle;
    // Serial.printf("Steering angle: %.1f deg\n", avgAngle);
  }

  steeringPID.Compute();
  applyTorque(pidOutput);

  Serial.printf("Target: %.1f  Current: %.1f  Output: %.1f  T1: %u  T2: %u\n",
                pidSetpoint, pidInput, pidOutput, t1, t2);

  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processController();

  dacWrite(DAC1, t1);
  dacWrite(DAC2, t2);

  delay(20);
}