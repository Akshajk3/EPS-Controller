#include "TorqueSteer.h"

TorqueSteer* TorqueSteer::instance = nullptr;

TorqueSteer::TorqueSteer(float p, float i, float d)
    : kp(p), ki(i), kd(d)
{
    instance = this;

    steeringPID = new PID(&pidInput, &pidOutput, &pidSetpoint, kp, ki, kd, DIRECT);
}

void TorqueSteer::begin() {
    pinMode(as1, INPUT);
    pinMode(as2, INPUT);

    attachInterrupt(digitalPinToInterrupt(as1), isr1Static, CHANGE);
    attachInterrupt(digitalPinToInterrupt(as2), isr2Static, CHANGE);

    BP32.setup(&onConnectStatic, &onDisconnectStatic);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    steeringPID->SetOutputLimits(-max_output, -max_output);
    steeringPID->SetSampleTime(150);
    steeringPID->SetMode(AUTOMATIC);

    dacWrite(dac1, t1);
    dacWrite(dac2, t2);

    delay(10000);
}

void TorqueSteer::update() {
    if (newData1) {
        noInterrupts();
        uint32_t high = highTime1;
        uint32_t low = lowTime1;
        newData1 = false;
        interrupts();

        float duty1 = ((float)high / (high + low)) * 100.0;
        float freq1 = 1000000.0 / (high + low);

        if (duty1 >= min_duty && duty1 <= max_duty)
            updateSensor1(duty1);
    }

    if (newData2) {
        noInterrupts();
        uint32_t high = highTime2;
        uint32_t low = lowTime2;
        newData2 = false;
        interrupts();

        float duty2 = ((float)high / (high + low)) * 100.0;
        float freq2 = 1000000.0 / (high + low);

        if (duty2 >= min_duty && duty2 <= max_duty)
            updateSensor2(duty2);
    }

    avgAngle = (accumedAngle1 + accumedAngle2) / 2.0f;
    pidInput = avgAngle;

    steeringPID->Compute();
    applyTorque(pidOutput);

    Serial.printf("Target: %.1f  Current: %.1f  Output: %.1f  T1: %u  T2: %u\n",
                pidSetpoint, pidInput, pidOutput, t1, t2);

    bool dataUpdate = BP32.update();

    if (dataUpdate)
        processController();

    dacWrite(dac1, t1);
    dacWrite(dac2, t2);
}

void TorqueSteer::setTargetAngle(float angle) {
    target_angle = angle;
}

float TorqueSteer::getAngle() {
    return avgAngle;
}

void TorqueSteer::handleIsr1() {
    uint32_t now = micros();

    if (digitalRead(as1)) {
        lowTime1 = now - lastFallTime1;
        riseTime1 = now;
    } else {
        highTime1 = now - riseTime1;
        lastFallTime1 = now;
        newData1 = true;
    }
}

void TorqueSteer::handleIsr2() {
    uint32_t now = micros();

    if (digitalRead(as2)) {
        lowTime2 = now - lastFallTime2;
        riseTime2 = now;
    } else {
        highTime2 = now - riseTime2;
        lastFallTime2 = now;
        newData2 = true;
    }
}

void TorqueSteer::applyTorque(double output) {
    if (abs(pidInput - pidSetpoint) < angle_deadband) {
        double holdOutput = (pidSetpoint > 0) ? hold_output : -hold_output;

        if (abs(pidSetpoint) < 1.0) {
            t1 = centerV;
            t2 = centerV;
            return;
        }

        t1 = constrain((int)centerV - (int)holdOutput, v2, v1);
        t2 = constrain((int)centerV - (int)holdOutput, v2, v1);
        return;
    }

    if (output > 0 && output < min_motor_output)
        output = min_motor_output;
    if (output < 0 && output > -min_motor_output)
        output = -min_motor_output;

    t1 = constrain((int)centerV - (int)output, v2, v1);
    t2 = constrain((int)centerV + (int)output, v2, v1);
}

void TorqueSteer::onConnect(ControllerPtr ctl) {
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

void TorqueSteer::onDisconnect(ControllerPtr ctl) {
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

void TorqueSteer::processGamepad(ControllerPtr ctl) {
    int x = ctl->axisX();

    if (abs(x) < 5) {
        t1 = centerV;
        t2 = centerV;
    }

    t1 = map(x, -512, 512, v1, v2);
    t2 = map(x, -512, 512, v2, v1);
}

void TorqueSteer::processController() {
    for (auto controller : controllers) {
        if (controller && controller->isConnected() && controller->hasData()) {
        if (controller->isGamepad())
            processGamepad(controller);
        }
    }
}

void TorqueSteer::printData(ControllerPtr ctl) {
    Serial.printf("axis L: %4d, T1: %u, T2: %u\n",
            ctl->axisX(), t1, t2);
}

void TorqueSteer::updateSensor1(float duty) {
  accumedAngle1 += dutyToAngleDelta(duty, lastDuty1, as1_ccw_positive);
  lastDuty1 = duty;
}

void TorqueSteer::updateSensor2(float duty) {
  accumedAngle2 += dutyToAngleDelta(duty, lastDuty2, as2_ccw_positive);
  lastDuty2 = duty;
}

float TorqueSteer::dutyToAngleDelta(float newDuty, float oldDuty, bool ccwPositive) {
    float delta = newDuty - oldDuty;

    float halfSpan = duty_span / 2.0;
    if (delta > halfSpan) delta -= duty_span;
    if (delta < -halfSpan) delta += duty_span;

    float angleDelta = delta * deg_per_pct;
    if (!ccwPositive) angleDelta = -angleDelta;

    return angleDelta;
}

void IRAM_ATTR TorqueSteer::isr1Static() {
    if (instance) instance->handleIsr1();
}

void IRAM_ATTR TorqueSteer::isr2Static() {
    if (instance) instance->handleIsr2();
}

void TorqueSteer::onConnectStatic(ControllerPtr ctl) {
    if (instance) instance->onConnect(ctl);
}

void TorqueSteer::onDisconnectStatic(ControllerPtr ctl) {
    if (instance) instance->onDisconnect(ctl);
}
