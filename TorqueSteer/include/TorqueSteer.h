#pragma once

#include <Arduino.h>
#include <Bluepad32.h>
#include <PID_v1.h>

class TorqueSteer{
public:
    TorqueSteer(float p, float i, float d);

public:
    void setTargetAngle(float angle);
    void begin();
    void update();
    float getAngle();

private:
    void onConnect(ControllerPtr ctl);
    void onDisconnect(ControllerPtr ctl);
    void processGamepad(ControllerPtr ctl);
    void printData(ControllerPtr ctl);
    void processController();
    void handleIsr1();
    void handleIsr2();
    void applyTorque(double output);
    void updateSensor1(float duty);
    void updateSensor2(float duty);

    float dutyToAngleDelta(float newDuty, float oldDuty, bool ccwPositive);

    static TorqueSteer* instance;

    static void IRAM_ATTR isr1Static();
    static void IRAM_ATTR isr2Static();
    static void onConnectStatic(ControllerPtr ctl);
    static void onDisconnectStatic(ControllerPtr ctl);

private:
    ControllerPtr controllers[BP32_MAX_GAMEPADS];
    PID* steeringPID;

private:
    int dac1 = 25;
    int dac2 = 26;
    int as1;
    int as2;

    float min_duty = 10.0f;
    float max_duty = 90.0f;

    float duty_span = (max_duty - min_duty);
    float deg_per_pct;

    float as1_center = 80.0f;
    bool as1_ccw_positive = false;

    float as2_center = 30.0f;
    bool as2_ccw_positive = true;

    float kp;
    float ki;
    float kd;

    float target_angle;
    float max_output = 40.0f;
    float angle_deadband = 15.0f;

    float min_motor_output = 15.0f;

    float hold_output = 10.0f;

    uint8_t centerV = 198;
    uint8_t v1 = 234;
    uint8_t v2 = 148;

    uint8_t t1 = 198;
    uint8_t t2 = 198;

    volatile uint32_t riseTime1;
    volatile uint32_t highTime1;
    volatile uint32_t lowTime1;
    volatile uint32_t lastFallTime1;
    volatile bool newData1 = false;

    volatile uint32_t riseTime2;
    volatile uint32_t highTime2;
    volatile uint32_t lowTime2;
    volatile uint32_t lastFallTime2;
    volatile bool newData2 = false;

    float lastDuty1 = as1_center;
    float lastDuty2 = as2_center;

    float accumedAngle1;
    float accumedAngle2;

    bool intialized1 = true;
    bool intialized2 = true;

    float avgAngle;

    double pidSetpoint;
    double pidInput;
    double pidOutput;
};