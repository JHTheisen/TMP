/*
  ESP32 Dual Stepper Controller
  -----------------------------
  Receives joystick commands from Python over Serial.

  Format:
      yaw,pitch
      K          record current AS5600 angle as a keyframe
      B          play recorded AS5600 keyframes once
      Z          return AS5600 axis to startup zero

  Example:
      250,-500

  Motor 1
      STEP = GPIO13
      DIR  = GPI026
      EN   = GPI014 (Active LOW)

  Motor 2
      STEP = GPIO22
      DIR  = GPIO21
      EN   = GND (always enabled)

  AS5600
      SDA = GPIO4
      SCL = GPIO5
*/

#include <Wire.h>
#include <AS5600.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef AS5600_RAW_TO_DEGREES
#define AS5600_RAW_TO_DEGREES (360.0 / 4096.0)
#endif

AS5600 encoder;
Adafruit_BNO08x bno = Adafruit_BNO08x();

const int STEP1 = 13;
const int DIR1  = 26;
const int EN1   = 14;

const uint8_t AS5600_ADDRESS = 0x36;

bool pitchEncoderAvailable = false;
float pitchZeroAngle = 0.0;
bool pitchZeroAngleInitialized = false;

bool bnoAvailable = false;
bool northMode = false;
bool levelMode = false;
bool northLevelMode = false;
unsigned long lastBNOInitAttempt = 0;
bool bnoRotationVectorEnabled = false;
bool bnoAccelerometerEnabled = false;
bool bnoLastHeadingAbsolute = false;
bool bnoAccelerometerWarningPrinted = false;

const float NORTH_TARGET_DEG = 0.0;
const float LEVEL_TARGET_DEG = 0.0;
const float NORTH_TOLERANCE_DEG = 5.0;
const float LEVEL_TOLERANCE_DEG = 3.0;
const float NORTH_GAIN = 50.0;
const float LEVEL_GAIN = 70.0;
const float NORTH_LEVEL_MAX_RATE = 500.0;
const bool LEVEL_INVERT = true;
const bool LEVEL_USE_ROLL = false; // Set true if the sensor is mounted so 'roll' is the leveling axis
const int MANUAL_COMMAND_DEADZONE = 80;
const int MANUAL_OVERRIDE_THRESHOLD = 250;
const byte BNO_STARTUP_RETRIES = 5;
const unsigned long BNO_STARTUP_RETRY_DELAY_MS = 250;
const unsigned long BNO_RUNTIME_RETRY_INTERVAL_MS = 2000;
const unsigned long BNO_REPORT_INTERVAL_US = 10000;
const unsigned long BNO_HEADING_MAX_AGE_MS = 750;

const int STEP2 = 22;
const int DIR2  = 21;

const float MAX_RATE = 2000.0;
const float MOTOR1_MAX_RATE = 1000.0;
const float MOTOR2_MAX_RATE = 2000.0;
const float MOTOR1_ACCEL = 3000.0;
const float MOTOR2_ACCEL = 3000.0;
const unsigned int STEP_PULSE_WIDTH_US = 3;
const float TELEMETRY_MOVING_RATE_THRESHOLD = 1.0;
const unsigned int STEP_TIMER_INTERVAL_US = 10;
const unsigned int STEP_TIMER_FREQUENCY_HZ = 1000000;

float motor1Rate = 0.0;
float motor2Rate = 0.0;
float desiredMotor1Rate = 0.0;
float desiredMotor2Rate = 0.0;
unsigned long lastLoopMicros = 0;

hw_timer_t *stepTimer = NULL;
portMUX_TYPE stepTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile float timerMotor1Rate = 0.0;
volatile float timerMotor2Rate = 0.0;
volatile bool timerMotor1Enabled = false;
volatile bool timerMotor2Enabled = false;
volatile bool timerMotor1DirHigh = LOW;
volatile bool timerMotor2DirHigh = LOW;
volatile unsigned long timerMotor1IntervalUs = 0;
volatile unsigned long timerMotor2IntervalUs = 0;
volatile unsigned long timerStep1LastStep = 0;
volatile unsigned long timerStep2LastStep = 0;
volatile unsigned long timerStep1HighUntil = 0;
volatile unsigned long timerStep2HighUntil = 0;
volatile bool timerStep1High = false;
volatile bool timerStep2High = false;

bool returnToZeroMode = false;
float zeroAngle = 0.0;
bool zeroAngleInitialized = false;

const float ZERO_TOLERANCE_DEG = 5.0;
const float ZERO_GAIN = 80.0;
const float ZERO_MIN_RATE = 50.0;

const byte MAX_KEYFRAMES = 12;
float keyframes[MAX_KEYFRAMES];
byte keyframeCount = 0;
byte playbackIndex = 0;
bool keyframePlaybackMode = false;
float playbackRate = 0.0;
unsigned long lastPlaybackUpdate = 0;

const float KEYFRAME_TOLERANCE_DEG = 3.0;
const float KEYFRAME_MIN_RATE = 80.0;
const float KEYFRAME_MAX_RATE = 4200.0;
const float KEYFRAME_GAIN = 85.0;
const float KEYFRAME_ACCEL = 9000.0;

char buffer[40];
byte indexBuffer = 0;

unsigned long lastEncoderPrint = 0;
const unsigned long ENCODER_PRINT_INTERVAL_MS = 250;
unsigned long lastBNOStatusPrint = 0;
bool bnoOrientationEventSeen = false;
float lastAbsoluteHeading = 0.0;
unsigned long lastAbsoluteHeadingMillis = 0;

void setup()
{
    Serial.begin(115200);

    Wire.begin(4, 5);
    Wire1.begin(18, 19);

    Serial.println("Scanning Wire1 I2C bus (pins 18/19)...");
    scanWire1();

    if (!encoder.begin())
    {
        Serial.println("Yaw AS5600 NOT FOUND on Wire (pins 4/5)!");
    }
    else
    {
        encoder.setDirection(AS5600_CLOCK_WISE);
        zeroAngle = readEncoderAngleDegrees();
        zeroAngleInitialized = true;
        Serial.println("Yaw AS5600 Ready");
        Serial.print("Zero angle: ");
        Serial.println(zeroAngle, 2);
    }

    pitchEncoderAvailable = initPitchEncoder();
    if (pitchEncoderAvailable)
    {
        Serial.println("Pitch AS5600 Ready on Wire1 (pins 18/19)");
        Serial.print("Pitch zero angle: ");
        Serial.println(pitchZeroAngle, 2);
    }
    else
    {
        Serial.println("Pitch AS5600 not found on Wire1");
    }

    bnoAvailable = initBNO085WithRetries(BNO_STARTUP_RETRIES, BNO_STARTUP_RETRY_DELAY_MS);
    if (bnoAvailable)
    {
        Serial.println("BNO085 IMU Ready on Wire1");
    }
    else
    {
        Serial.println("BNO085 IMU not found on Wire1");
    }

    pinMode(STEP1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(EN1, OUTPUT);

    pinMode(STEP2, OUTPUT);
    pinMode(DIR2, OUTPUT);

    digitalWrite(STEP1, LOW);
    digitalWrite(STEP2, LOW);

    digitalWrite(EN1, LOW);

    initStepTimer();

    Serial.println("ESP32 Dual Stepper Ready");
    lastLoopMicros = micros();
}

float rampRate(float current, float target, float maxAccel, float dt)
{
    float delta = target - current;
    float maxStep = maxAccel * dt;
    if (fabs(delta) <= maxStep)
        return target;

    return current + copysign(maxStep, delta);
}

void loop()
{
    unsigned long nowLoopMicros = micros();
    float dt = (nowLoopMicros - lastLoopMicros) * 1e-6;
    if (dt <= 0.0)
        dt = 0.0001;
    else if (dt > 0.1)
        dt = 0.1;
    lastLoopMicros = nowLoopMicros;

    readSerial();

    if (!bnoAvailable && millis() - lastBNOInitAttempt >= BNO_RUNTIME_RETRY_INTERVAL_MS)
    {
        lastBNOInitAttempt = millis();
        bnoAvailable = initBNO085();
        if (bnoAvailable)
            Serial.println("BNO085 IMU recovered on Wire1");
    }

    if (northLevelMode)
    {
        updateNorthLevel();
    }
    else if (northMode)
    {
        updateNorth();
    }
    else if (levelMode)
    {
        updateLevel();
    }
    else if (returnToZeroMode)
    {
        updateReturnToZero();
    }
    else if (keyframePlaybackMode)
    {
        updateKeyframePlayback();
    }

    if (northMode || northLevelMode || returnToZeroMode)
        motor1Rate = rampRate(motor1Rate, desiredMotor1Rate, MOTOR1_ACCEL, dt);
    else
        motor1Rate = desiredMotor1Rate;

    if (levelMode || northLevelMode)
        motor2Rate = rampRate(motor2Rate, desiredMotor2Rate, MOTOR2_ACCEL, dt);
    else
        motor2Rate = desiredMotor2Rate;

    updateTimerMotorRates(motor1Rate, motor2Rate);

    bool motorsMoving = fabs(motor1Rate) > TELEMETRY_MOVING_RATE_THRESHOLD ||
                        fabs(motor2Rate) > TELEMETRY_MOVING_RATE_THRESHOLD;
    bool autoModeActive = northMode || levelMode || northLevelMode ||
                          returnToZeroMode || keyframePlaybackMode;
    bool manualMotionActive = motorsMoving && !autoModeActive;
    unsigned long telemetryInterval = motorsMoving ? ENCODER_PRINT_INTERVAL_MS * 2
                                                   : ENCODER_PRINT_INTERVAL_MS;

    if (millis() - lastEncoderPrint >= telemetryInterval)
    {
        lastEncoderPrint = millis();

        uint16_t raw = encoder.readAngle();
        float angle = raw * AS5600_RAW_TO_DEGREES;

        Serial.print("A,");
        Serial.println(angle, 2);

        if (!manualMotionActive && pitchEncoderAvailable)
        {
            float pitchAngle = readPitchEncoderAngleDegrees();
            Serial.print("P,");
            Serial.println(pitchAngle, 2);
        }

        if (!manualMotionActive && bnoAvailable)
        {
            float heading, bnoPitch, bnoRoll;
            if (readBNOOrientation(heading, bnoPitch, bnoRoll))
            {
                bnoOrientationEventSeen = true;
                if (bnoLastHeadingAbsolute)
                {
                    Serial.print("H,");
                    Serial.println(heading, 2);
                    Serial.print("O,");
                    Serial.println(bnoPitch, 2);
                    Serial.print("R,");
                    Serial.println(bnoRoll, 2);
                }
            }
            else if (!bnoOrientationEventSeen && millis() - lastBNOStatusPrint >= BNO_RUNTIME_RETRY_INTERVAL_MS)
            {
                lastBNOStatusPrint = millis();
                Serial.println("BNO085 ready, waiting for rotation vector data");
            }
        }
    }
}

void IRAM_ATTR serviceTimerMotor(
    int stepPin,
    int dirPin,
    bool enabled,
    bool dirHigh,
    unsigned long interval,
    volatile unsigned long &lastStep,
    volatile unsigned long &stepHighUntil,
    volatile bool &stepHigh,
    unsigned long now)
{
    if (stepHigh && (long)(now - stepHighUntil) >= 0)
    {
        digitalWrite(stepPin, LOW);
        stepHigh = false;
    }

    if (!enabled)
        return;

    if (now - lastStep < interval)
        return;

    digitalWrite(dirPin, dirHigh ? HIGH : LOW);
    digitalWrite(stepPin, HIGH);
    stepHigh = true;
    stepHighUntil = now + STEP_PULSE_WIDTH_US;
    lastStep = now;
}

void IRAM_ATTR onStepTimer()
{
    unsigned long now = micros();
    bool motor1EnabledSnapshot;
    bool motor2EnabledSnapshot;
    bool motor1DirHighSnapshot;
    bool motor2DirHighSnapshot;
    unsigned long motor1IntervalSnapshot;
    unsigned long motor2IntervalSnapshot;

    portENTER_CRITICAL_ISR(&stepTimerMux);
    motor1EnabledSnapshot = timerMotor1Enabled;
    motor2EnabledSnapshot = timerMotor2Enabled;
    motor1DirHighSnapshot = timerMotor1DirHigh;
    motor2DirHighSnapshot = timerMotor2DirHigh;
    motor1IntervalSnapshot = timerMotor1IntervalUs;
    motor2IntervalSnapshot = timerMotor2IntervalUs;
    portEXIT_CRITICAL_ISR(&stepTimerMux);

    serviceTimerMotor(STEP2, DIR2, motor1EnabledSnapshot, motor1DirHighSnapshot,
                      motor1IntervalSnapshot, timerStep2LastStep,
                      timerStep2HighUntil, timerStep2High, now);
    serviceTimerMotor(STEP1, DIR1, motor2EnabledSnapshot, motor2DirHighSnapshot,
                      motor2IntervalSnapshot, timerStep1LastStep,
                      timerStep1HighUntil, timerStep1High, now);
}

void initStepTimer()
{
    portENTER_CRITICAL(&stepTimerMux);
    timerMotor1Rate = 0.0;
    timerMotor2Rate = 0.0;
    timerMotor1Enabled = false;
    timerMotor2Enabled = false;
    timerMotor1DirHigh = LOW;
    timerMotor2DirHigh = LOW;
    timerMotor1IntervalUs = 0;
    timerMotor2IntervalUs = 0;
    timerStep1LastStep = micros();
    timerStep2LastStep = timerStep1LastStep;
    timerStep1HighUntil = 0;
    timerStep2HighUntil = 0;
    timerStep1High = false;
    timerStep2High = false;
    portEXIT_CRITICAL(&stepTimerMux);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    stepTimer = timerBegin(STEP_TIMER_FREQUENCY_HZ);
    timerAttachInterrupt(stepTimer, &onStepTimer);
    timerAlarm(stepTimer, STEP_TIMER_INTERVAL_US, true, 0);
#else
    stepTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(stepTimer, &onStepTimer, true);
    timerAlarmWrite(stepTimer, STEP_TIMER_INTERVAL_US, true);
    timerAlarmEnable(stepTimer);
#endif
}

void updateTimerMotorRates(float newMotor1Rate, float newMotor2Rate)
{
    float motor1Speed = fabs(newMotor1Rate);
    float motor2Speed = fabs(newMotor2Rate);

    portENTER_CRITICAL(&stepTimerMux);
    timerMotor1Rate = newMotor1Rate;
    timerMotor2Rate = newMotor2Rate;
    timerMotor1Enabled = motor1Speed >= 1.0;
    timerMotor2Enabled = motor2Speed >= 1.0;
    timerMotor1DirHigh = newMotor1Rate > 0.0;
    timerMotor2DirHigh = newMotor2Rate > 0.0;
    timerMotor1IntervalUs = timerMotor1Enabled ? (unsigned long)(1000000.0 / motor1Speed) : 0;
    timerMotor2IntervalUs = timerMotor2Enabled ? (unsigned long)(1000000.0 / motor2Speed) : 0;
    portEXIT_CRITICAL(&stepTimerMux);
}

float readEncoderAngleDegrees()
{
    uint16_t raw = encoder.readAngle();
    return raw * AS5600_RAW_TO_DEGREES;
}

bool initPitchEncoder()
{
    Wire1.beginTransmission(AS5600_ADDRESS);
    int result = Wire1.endTransmission();
    if (result != 0)
    {
        Serial.print("Pitch AS5600 find failed, Wire1.endTransmission returned ");
        Serial.println(result);
        return false;
    }

    pitchZeroAngle = readPitchEncoderAngleDegrees();
    pitchZeroAngleInitialized = true;
    return true;
}

void scanWire1()
{
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0)
        {
            Serial.print("Wire1 device found at 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            found = true;
        }
    }
    if (!found)
    {
        Serial.println("No devices found on Wire1.");
    }
}

float readPitchEncoderAngleDegrees()
{
    Wire1.beginTransmission(AS5600_ADDRESS);
    Wire1.write(0x0C);
    if (Wire1.endTransmission(false) != 0)
        return NAN;

    if (Wire1.requestFrom(AS5600_ADDRESS, (uint8_t)2) != 2)
        return NAN;

    uint16_t high = Wire1.read();
    uint16_t low = Wire1.read();
    uint16_t raw = ((high << 8) | low) & 0x0FFF;
    return raw * AS5600_RAW_TO_DEGREES;
}

bool initBNO085()
{
    bnoRotationVectorEnabled = false;
    bnoAccelerometerEnabled = false;
    bnoLastHeadingAbsolute = false;
    bnoAccelerometerWarningPrinted = false;

    if (!bno.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1))
    {
        if (!bno.begin_I2C(BNO08x_I2CADDR_DEFAULT + 1, &Wire1))
            return false;

        Serial.println("BNO085 found at alternate address 0x4B");
    }
    else
    {
        Serial.println("BNO085 found at default address 0x4A");
    }

    bnoRotationVectorEnabled = bno.enableReport(SH2_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
    bnoAccelerometerEnabled = bno.enableReport(SH2_ACCELEROMETER, BNO_REPORT_INTERVAL_US);

    if (!bnoRotationVectorEnabled && !bnoAccelerometerEnabled)
        return false;

    if (!bnoRotationVectorEnabled)
        Serial.println("BNO085 absolute rotation vector unavailable");
    if (!bnoAccelerometerEnabled)
        Serial.println("BNO085 accelerometer report unavailable");

    return true;
}

bool initBNO085WithRetries(byte attempts, unsigned long retryDelayMs)
{
    for (byte attempt = 1; attempt <= attempts; attempt++)
    {
        if (initBNO085())
            return true;

        if (attempt < attempts)
            delay(retryDelayMs);
    }

    return false;
}

bool readBNOOrientation(float &heading, float &pitch, float &roll)
{
    sh2_SensorValue_t value;

    if (bno.wasReset())
    {
        Serial.println("BNO085 reset detected, re-enabling orientation reports");
        bnoRotationVectorEnabled = bno.enableReport(SH2_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
        bnoAccelerometerEnabled = bno.enableReport(SH2_ACCELEROMETER, BNO_REPORT_INTERVAL_US);

        if (!bnoRotationVectorEnabled && !bnoAccelerometerEnabled)
        {
            bnoAvailable = false;
            Serial.println("BNO085 orientation/accelerometer report re-enable failed");
            return false;
        }
    }

    if (!bno.getSensorEvent(&value))
        return false;

    if (value.sensorId != SH2_ROTATION_VECTOR &&
        value.sensorId != SH2_ACCELEROMETER)
        return false;

    if (value.sensorId == SH2_ACCELEROMETER)
    {
        float ax = value.un.accelerometer.x;
        float ay = value.un.accelerometer.y;
        float az = value.un.accelerometer.z;

        heading = 0.0;
        pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
        roll = atan2(ay, az) * 180.0 / PI;
        bnoLastHeadingAbsolute = false;

        if (!bnoAccelerometerWarningPrinted)
        {
            bnoAccelerometerWarningPrinted = true;
            Serial.println("BNO085 accelerometer fallback available for level");
        }

        return true;
    }

    bnoLastHeadingAbsolute = true;

    float x = value.un.rotationVector.i;
    float y = value.un.rotationVector.j;
    float z = value.un.rotationVector.k;
    float w = value.un.rotationVector.real;

    float sinp = 2.0 * (w * y - z * x);
    if (sinp >= 1.0)
        pitch = 90.0;
    else if (sinp <= -1.0)
        pitch = -90.0;
    else
        pitch = asin(sinp) * 180.0 / PI;

    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / PI;
    heading = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / PI;

    if (heading < 0.0)
        heading += 360.0;

    lastAbsoluteHeading = heading;
    lastAbsoluteHeadingMillis = millis();

    return true;
}

void stopAllModes()
{
    northMode = false;
    levelMode = false;
    northLevelMode = false;
    returnToZeroMode = false;
    keyframePlaybackMode = false;
    desiredMotor1Rate = 0.0;
    desiredMotor2Rate = 0.0;
}

void updateNorth()
{
    if (!bnoAvailable)
    {
        northMode = false;
        desiredMotor1Rate = 0.0;
        Serial.println("North mode canceled: BNO085 unavailable");
        return;
    }

    float heading, pitch, roll;
    if (!readBNOOrientation(heading, pitch, roll))
        return;

    if (millis() - lastAbsoluteHeadingMillis > BNO_HEADING_MAX_AGE_MS)
    {
        desiredMotor1Rate = 0.0;
        Serial.println("NORTHDBG waiting for absolute BNO heading");
        return;
    }

    heading = lastAbsoluteHeading;
    float error = signedAngleDelta(heading, NORTH_TARGET_DEG);

    if (fabs(error) <= NORTH_TOLERANCE_DEG)
    {
        northMode = false;
        desiredMotor1Rate = 0.0;
        Serial.println("North heading reached");
        return;
    }

    desiredMotor1Rate = constrain(error * NORTH_GAIN, -MOTOR1_MAX_RATE, MOTOR1_MAX_RATE);
}

void updateLevel()
{
    if (!bnoAvailable)
    {
        levelMode = false;
        desiredMotor2Rate = 0.0;
        Serial.println("Level mode canceled: BNO085 unavailable");
        return;
    }

    float heading, pitch, roll;
    if (!readBNOOrientation(heading, pitch, roll))
        return;

    float sensorAngle = LEVEL_USE_ROLL ? roll : pitch;
    float error = sensorAngle - LEVEL_TARGET_DEG;

    if (fabs(error) <= LEVEL_TOLERANCE_DEG)
    {
        levelMode = false;
        desiredMotor2Rate = 0.0;
        Serial.println("Level reached");
        return;
    }

    float levelOutput = LEVEL_INVERT ? error * LEVEL_GAIN : -error * LEVEL_GAIN;
    desiredMotor2Rate = constrain(levelOutput, -MOTOR2_MAX_RATE, MOTOR2_MAX_RATE);
}

void updateNorthLevel()
{
    if (!bnoAvailable)
    {
        northLevelMode = false;
        desiredMotor1Rate = 0.0;
        desiredMotor2Rate = 0.0;
        Serial.println("North and level mode canceled: BNO085 unavailable");
        return;
    }

    float heading, pitch, roll;
    if (!readBNOOrientation(heading, pitch, roll))
        return;

    if (millis() - lastAbsoluteHeadingMillis > BNO_HEADING_MAX_AGE_MS)
    {
        desiredMotor1Rate = 0.0;
        desiredMotor2Rate = 0.0;
        Serial.println("NORTHDBG waiting for absolute BNO heading");
        return;
    }

    heading = lastAbsoluteHeading;
    float headingError = signedAngleDelta(heading, NORTH_TARGET_DEG);
    float levelAngle = LEVEL_USE_ROLL ? roll : pitch;
    float pitchError = levelAngle - LEVEL_TARGET_DEG;

    if (fabs(headingError) <= NORTH_TOLERANCE_DEG && fabs(pitchError) <= LEVEL_TOLERANCE_DEG)
    {
        northLevelMode = false;
        desiredMotor1Rate = 0.0;
        desiredMotor2Rate = 0.0;
        Serial.println("North and level reached");
        return;
    }

    if (fabs(pitchError) > LEVEL_TOLERANCE_DEG)
    {
        desiredMotor1Rate = 0.0;
        float levelOutput = LEVEL_INVERT ? pitchError * LEVEL_GAIN : -pitchError * LEVEL_GAIN;
        desiredMotor2Rate = constrain(levelOutput, -NORTH_LEVEL_MAX_RATE, NORTH_LEVEL_MAX_RATE);
        return;
    }

    desiredMotor2Rate = 0.0;
    desiredMotor1Rate = constrain(headingError * NORTH_GAIN,
                                  -NORTH_LEVEL_MAX_RATE,
                                  NORTH_LEVEL_MAX_RATE);
}

float signedAngleDelta(float currentAngle, float targetAngle)
{
    float delta = targetAngle - currentAngle;

    if (delta > 180.0)
    {
        delta -= 360.0;
    }
    else if (delta < -180.0)
    {
        delta += 360.0;
    }

    return delta;
}

void updateReturnToZero()
{
    if (!zeroAngleInitialized)
    {
        returnToZeroMode = false;
        desiredMotor1Rate = 0.0;
        Serial.println("Return-to-zero canceled: yaw AS5600 unavailable");
        return;
    }

    float currentAngle = readEncoderAngleDegrees();
    if (isnan(currentAngle))
    {
        returnToZeroMode = false;
        desiredMotor1Rate = 0.0;
        Serial.println("Return-to-zero canceled: yaw AS5600 read failed");
        return;
    }

    float error = signedAngleDelta(currentAngle, zeroAngle);

    if (fabs(error) <= ZERO_TOLERANCE_DEG)
    {
        returnToZeroMode = false;
        desiredMotor1Rate = 0.0;
        Serial.println("Zero reached");
        return;
    }

    float returnRate = constrain(fabs(error) * ZERO_GAIN, ZERO_MIN_RATE, MOTOR1_MAX_RATE);
    desiredMotor1Rate = error >= 0.0 ? returnRate : -returnRate;
}

void recordKeyframe()
{
    if (!zeroAngleInitialized)
    {
        Serial.println("Cannot record keyframe: AS5600 unavailable");
        return;
    }

    if (keyframeCount >= MAX_KEYFRAMES)
    {
        Serial.println("Keyframe list full");
        return;
    }

    keyframes[keyframeCount] = readEncoderAngleDegrees();
    keyframeCount++;

    Serial.print("Keyframe ");
    Serial.print(keyframeCount);
    Serial.print(" recorded: ");
    Serial.println(keyframes[keyframeCount - 1], 2);
}

void startKeyframePlayback()
{
    if (!zeroAngleInitialized)
    {
        Serial.println("Cannot play keyframes: AS5600 unavailable");
        return;
    }

    if (keyframeCount < 2)
    {
        Serial.println("Need at least two keyframes");
        return;
    }

    returnToZeroMode = false;
    keyframePlaybackMode = true;
    playbackIndex = 0;
    playbackRate = 0.0;
    lastPlaybackUpdate = micros();

    Serial.print("Keyframe playback started: ");
    Serial.print(keyframeCount);
    Serial.println(" keyframes");
}

void stopKeyframePlayback(const char *reason)
{
    keyframePlaybackMode = false;
    playbackRate = 0.0;
    motor2Rate = 0.0;
    Serial.println(reason);
}

float rampPlaybackRate(float desiredRate)
{
    unsigned long now = micros();
    float dt = (now - lastPlaybackUpdate) / 1000000.0;
    lastPlaybackUpdate = now;

    if (dt <= 0.0 || dt > 0.1)
    {
        dt = 0.001;
    }

    float maxChange = KEYFRAME_ACCEL * dt;
    float delta = constrain(desiredRate - playbackRate, -maxChange, maxChange);
    playbackRate += delta;
    return playbackRate;
}

void updateKeyframePlayback()
{
    float currentAngle = readEncoderAngleDegrees();
    float targetAngle = keyframes[playbackIndex];
    float error = signedAngleDelta(currentAngle, targetAngle);

    if (fabs(error) <= KEYFRAME_TOLERANCE_DEG)
    {
        desiredMotor2Rate = 0.0;
        playbackRate = 0.0;

        if (playbackIndex >= keyframeCount - 1)
        {
            stopKeyframePlayback("Keyframe playback complete");
            return;
        }

        playbackIndex++;
        Serial.print("Moving to keyframe ");
        Serial.println(playbackIndex + 1);
        return;
    }

    float desiredRate = constrain(fabs(error) * KEYFRAME_GAIN,
                                  KEYFRAME_MIN_RATE,
                                  KEYFRAME_MAX_RATE);
    desiredRate = error >= 0.0 ? desiredRate : -desiredRate;
    desiredMotor2Rate = rampPlaybackRate(desiredRate);
}

void readSerial()
{
    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            buffer[indexBuffer] = 0;

            char *comma = strchr(buffer, ',');

            if (strcmp(buffer, "Z") == 0)
            {
                stopAllModes();
                returnToZeroMode = true;
                Serial.println("Return-to-zero request accepted");
            }
            else if (strcmp(buffer, "K") == 0)
            {
                stopAllModes();
                recordKeyframe();
            }
            else if (strcmp(buffer, "B") == 0)
            {
                stopAllModes();
                startKeyframePlayback();
            }
            else if (strcmp(buffer, "L") == 0)
            {
                stopAllModes();
                levelMode = true;
                Serial.println("Level mode accepted");
            }
            else if (strcmp(buffer, "N") == 0)
            {
                stopAllModes();
                northMode = true;
                Serial.println("North heading mode accepted");
            }
            else if (strcmp(buffer, "M") == 0)
            {
                stopAllModes();
                northLevelMode = true;
                Serial.println("North and level mode accepted");
            }
            else if (strcmp(buffer, "C") == 0)
            {
                stopAllModes();
                Serial.println("Control routine canceled");
            }
            else if (comma)
            {
                *comma = 0;

                int yaw = atoi(buffer);
                int pitch = atoi(comma + 1);

                yaw = constrain(yaw, -1000, 1000);
                pitch = constrain(pitch, -1000, 1000);

                if (abs(yaw) < MANUAL_COMMAND_DEADZONE)
                    yaw = 0;
                if (abs(pitch) < MANUAL_COMMAND_DEADZONE)
                    pitch = 0;

                bool autoModeActive = northMode || levelMode || northLevelMode ||
                                      returnToZeroMode || keyframePlaybackMode;
                bool manualOverrideRequested = abs(yaw) > MANUAL_OVERRIDE_THRESHOLD ||
                                               abs(pitch) > MANUAL_OVERRIDE_THRESHOLD;

                if (autoModeActive && manualOverrideRequested)
                {
                    stopAllModes();
                    Serial.println("Manual override: auto mode canceled");
                }

                if (!northMode && !levelMode && !northLevelMode && !returnToZeroMode)
                {
                    desiredMotor2Rate = (yaw / 1000.0) * MOTOR2_MAX_RATE;
                }

                if (keyframePlaybackMode)
                {
                    if (abs(pitch) > 50)
                    {
                        stopKeyframePlayback("Keyframe playback interrupted");
                    }
                }

                if (!keyframePlaybackMode && !levelMode && !northLevelMode)
                {
                    desiredMotor1Rate = (-pitch / 1000.0) * MOTOR1_MAX_RATE;
                }
            }

            indexBuffer = 0;
        }
        else if (c != '\r')
        {
            if (indexBuffer < sizeof(buffer) - 1)
                buffer[indexBuffer++] = c;
            else
                indexBuffer = 0;
        }
    }
}
