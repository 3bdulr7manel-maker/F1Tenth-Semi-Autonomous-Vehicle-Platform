
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <AS5600.h>

// ==========================================
// --- DYNAMIC AVERAGE FILTER ---
// ==========================================
constexpr int MEDIAN_WINDOW = 5;
constexpr int MAX_DYN_MEAN_WINDOW = 25;

// Structure to define the speed range and its corresponding filter window size
struct FilterRange {
    float maxThreshold;  // Maximum RPM for this specific range
    int windowSize;      // Number of samples to average (filter strength)
};

// Define the ranges (adjust these values based on your motor's actual behavior)
const FilterRange rpmRanges[10] = {
    {300.0f,  4},   {600.0f,  6},   {900.0f,  8},   {1200.0f, 10},
    {1500.0f, 14},  {1800.0f, 14},  {2100.0f, 16},  {2400.0f, 18},
    {2700.0f, 20},  {3000.0f, 25}
};

// ==========================================
// 1. PIN DEFINITIONS & I2C
// ==========================================
constexpr uint8_t RPWM1     = 19;
constexpr uint8_t LPWM1     = 23;
constexpr uint8_t RPWM2     = 4;
constexpr uint8_t LPWM2     = 18;
constexpr uint8_t SERVO_PIN = 14;
constexpr uint8_t CH1_PIN   = 15;
constexpr uint8_t CH2_PIN   = 13;
constexpr uint8_t TCA_ADDR  = 0x70;

// ==========================================
// 2. DUAL CORE THREAD SAFETY (MUTEX)
// ==========================================
// targetMux ONLY protects targetRPM_L / targetRPM_R
// (written on Core1 via applyAckermann, read on Core0 in calculatePI)
portMUX_TYPE targetMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE rcMux     = portMUX_INITIALIZER_UNLOCKED;  // RC raw values
TaskHandle_t Core0TaskHandle;

// ==========================================
// 3. STEERING & SERVO SYSTEM
// ==========================================
struct SteeringSystem {
    Servo  object;
    const int minDegrees = 60;
    const int maxDegrees = 120;
    float currentAngle   = 90.0f;  // 90 = center/straight
};
SteeringSystem mySteering;

// ==========================================
// 4. MOTOR & PI CONTROL STRUCTURE
// ==========================================
//  [F6] targetRPM is now in ACTUAL RPM units.
//       Mapping: RC [-2800..2800] PWM-equiv → [-MAX_RPM..MAX_RPM]
//       Adjust MAX_RPM to match your motor datasheet.
constexpr float MAX_RPM = 2800.0f;  // ← Set to your motor's measured max RPM

struct MotorSystem {
    const int   id;
    const float Kp = 0.55f;
    const float Ki = 0.4f;

    int   prevAngle   = 0;
    float integralSum = 0.0f;
    float calcOutput  = 0.0f;   // PWM output [-255..255]

    // Cross-core shared — protected by targetMux
    volatile float targetRPM = 0.0f;
    // Core-0 internal only — NO mutex needed
    float measuredRPM = 0.0f;

    // [F11] I2C error counter
    uint8_t i2cErrorCount = 0;

    // --- Filter Memory Buffers ---
    int medianBuffer[MEDIAN_WINDOW] = {0};
    int medianIndex = 0;

    float meanBuffer[MAX_DYN_MEAN_WINDOW] = {0.0f};
    int meanIndex = 0;
};

MotorSystem motorL = {1};
MotorSystem motorR = {2};

AS5600 encoderL;
AS5600 encoderR;

// ==========================================
// 5. ACKERMANN GEOMETRY
// ==========================================
struct AckermannChassis {
    const float L = 286.52f;  // Wheelbase (mm)
    const float w = 82.0f;    // Track width (mm)
};
AckermannChassis chassis;

// ==========================================
// 6. RC CHANNELS
// ==========================================
struct RCChannel {
    const uint8_t pin;
    volatile unsigned long startTime = 0;
    volatile uint32_t      rawValue  = 1500;
    float filteredValue               = 1500.0f;
};
constexpr float FILTER_WEIGHT = 0.1f;
RCChannel steerCh = {CH1_PIN};
RCChannel speedCh = {CH2_PIN};

// ==========================================
// 7. NON-BLOCKING ULTRASONIC  [F4] [US-FIX]
// ==========================================
struct UltrasonicSensor {
    const uint8_t   trigPin;
    const uint8_t   echoPin;
    int   distanceCm       = 999;
    int   prevDistanceCm   = 999; // المتغير الجديد لحفظ المسافة السابقة وحساب سرعة الاقتراب
    int   obstacleCount    = 0;
    unsigned long lastObstacleTime = 0;
};

UltrasonicSensor adas[4] = {
    {25,  33},   // Front  — index 0
    {5, 5},   // Back   — index 1
    {27, 26},  // Left   — index 2
    {16,17 }   // Right  — index 3
};



unsigned long lastSensorCheck  = 0;


// ==========================================
// 8. CAR STATE MACHINE
// ==========================================
enum CarState { NORMAL_RC, AVOIDANCE_OVERRIDE, BRAKING_SEQUENCE, SYSTEM_HALTED };

struct StateManager {
    CarState      currentState       = NORMAL_RC;
    unsigned long stateTimer         = 0;
    int           overrideSpeed      = 0;
    float         overrideSteering   = 0.0f;
    float         speedBeforeBraking = 0.0f;  // [F10] float, not int
};
StateManager carControl;

// ==========================================
// --- HELPER: TCA9548A I2C MUX SELECT ---
// ==========================================
void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// ==========================================
// --- MOTOR DRIVER OUTPUT ---
// ==========================================
void setMotorBTS(float speed, uint8_t rpwm_pin, uint8_t lpwm_pin) {
    int pwm = (int)constrain(speed, -255.0f, 255.0f);
    if (pwm > 0) {
        analogWrite(rpwm_pin, pwm);
        analogWrite(lpwm_pin, 0);
    } else if (pwm < 0) {
        analogWrite(rpwm_pin, 0);
        analogWrite(lpwm_pin, -pwm);
    } else {
        analogWrite(rpwm_pin, 0);
        analogWrite(lpwm_pin, 0);
    }
}

// ==========================================
// --- RC INTERRUPTS ---
// ==========================================
void IRAM_ATTR readRCChannel(void* arg) {
    RCChannel* ch = static_cast<RCChannel*>(arg);
    if (digitalRead(ch->pin) == HIGH) {
        ch->startTime = micros();
    } else {
        portENTER_CRITICAL_ISR(&rcMux);
        ch->rawValue = micros() - ch->startTime;
        portEXIT_CRITICAL_ISR(&rcMux);
    }
}

void setupRCInputPins() {
    pinMode(steerCh.pin, INPUT_PULLDOWN);
    pinMode(speedCh.pin, INPUT_PULLDOWN);
    attachInterruptArg(digitalPinToInterrupt(steerCh.pin), readRCChannel, &steerCh, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(speedCh.pin), readRCChannel, &speedCh, CHANGE);
}

void getSafeRCValues(int &steer, int &speed) {
    portENTER_CRITICAL(&rcMux);
    steer = steerCh.rawValue;
    speed = speedCh.rawValue;
    portEXIT_CRITICAL(&rcMux);
    if (steer < 900 || steer > 2100) steer = 1500;
    if (speed < 900 || speed > 2100) speed = 1500;
}

void applyRCSmoothingFilter(RCChannel &ch, int safeValue) {
    ch.filteredValue = (FILTER_WEIGHT * safeValue) + ((1.0f - FILTER_WEIGHT) * ch.filteredValue);
}

// ==========================================
// --- ENCODER READING  [F9, F11] ---
// ==========================================
int readAS5600Angle(AS5600 &enc) {
    uint16_t raw = enc.readAngle();
    return (int)raw;
}

void processEncoder(MotorSystem &motor, AS5600 &enc, float deltaT) {
    tcaSelect(motor.id - 1);
    int currAngle = readAS5600Angle(enc);

    if (currAngle < 0) {
        motor.i2cErrorCount++;
        // [F11] Re-initialise bus after 5 consecutive errors
        if (motor.i2cErrorCount >= 5) {
            Wire.end();
            delay(2);
            Wire.begin();
            Wire.setClock(400000);
            enc.begin();
            motor.i2cErrorCount = 0;
            Serial.println("[I2C] Bus recovered.");
        }
        return;
    }
    motor.i2cErrorCount = 0;

    int delta = currAngle - motor.prevAngle;
    if (delta < -2048) delta += 4096;
    else if (delta > 2048) delta -= 4096;

    float velocity = (float)delta / deltaT;
    float rawRPM   = (velocity / 4096.0f) * 60.0f;
    if (motor.id == 2) {
        rawRPM = -rawRPM; 
    }
    // -----------------------------------------------------

    // --- A) MEDIAN FILTER ---
    motor.medianBuffer[motor.medianIndex] = (int)rawRPM;

    // --- A) MEDIAN FILTER ---
    motor.medianBuffer[motor.medianIndex] = (int)rawRPM;
    motor.medianIndex = (motor.medianIndex + 1) % MEDIAN_WINDOW;

    int tempSort[MEDIAN_WINDOW];
    for (int i = 0; i < MEDIAN_WINDOW; i++) tempSort[i] = motor.medianBuffer[i];

    // Inline Bubble Sort (fast for N=5)
    for (int i = 0; i < MEDIAN_WINDOW - 1; i++) {
        for (int j = i + 1; j < MEDIAN_WINDOW; j++) {
            if (tempSort[i] > tempSort[j]) {
                int temp = tempSort[i]; tempSort[i] = tempSort[j]; tempSort[j] = temp;
            }
        }
    }
    float medianCleanValue = (float)tempSort[MEDIAN_WINDOW / 2];

    // --- B) 10-STAGE DYNAMIC MEAN FILTER ---
    motor.meanBuffer[motor.meanIndex] = medianCleanValue;
    motor.meanIndex = (motor.meanIndex + 1) % MAX_DYN_MEAN_WINDOW;

    float absVal = fabsf(medianCleanValue);
    int activeWindow = rpmRanges[9].windowSize;  // Default to max size

    for (int i = 0; i < 10; i++) {
        if (absVal <= rpmRanges[i].maxThreshold) {
            activeWindow = rpmRanges[i].windowSize;
            break;
        }
    }

    float sum = 0.0f;
    for (int i = 0; i < activeWindow; i++) {
        int lookbackIndex = (motor.meanIndex - 1 - i + MAX_DYN_MEAN_WINDOW) % MAX_DYN_MEAN_WINDOW;
        sum += motor.meanBuffer[lookbackIndex];
    }

    motor.measuredRPM = sum / (float)activeWindow;
    motor.prevAngle   = currAngle;
}

// ==========================================
// --- PI CONTROLLER  [F2, F6] ---
// ==========================================
void calculatePI(MotorSystem &motor, float deltaT) {
    // [F2] Only lock what is needed: reading the cross-core targetRPM
    float target;
    portENTER_CRITICAL(&targetMux);
    target = motor.targetRPM;
       portEXIT_CRITICAL(&targetMux);
     
        if (carControl.currentState != NORMAL_RC) {
    motor.integralSum = 0.0f; 
}

    if (target == 0.0f) {
        motor.calcOutput  = 0.0f;
        motor.integralSum = 0.0f;
        return;
    }

    float error = target - motor.measuredRPM;
    float P = motor.Kp * error;

    motor.integralSum += error * deltaT;
    motor.integralSum  = constrain(motor.integralSum, -500.0f, 500.0f);
    float I = motor.Ki * motor.integralSum;

    motor.calcOutput = constrain(P + I, -255.0f, 255.0f);
}

// ==========================================
// --- STEERING ---
// ==========================================
void updateSteering(SteeringSystem &steering, RCChannel &rc) {
    int mappedAngle = (int)map((long)rc.filteredValue, 1000, 2000, -26, 27);
    if (abs(mappedAngle) < 1) mappedAngle = 0;  // Deadband
    int finalAngle = constrain(93 + mappedAngle, steering.minDegrees, steering.maxDegrees);//////////93,87

    // [تعديل هنا]: هنحرك السيرفو بناءً على الريموت بس لو إحنا في الوضع الطبيعي
    // قرايات الريموت لسه شغالةوبتتحسب فوق، بس مش هتتنفذ فيزيائياً وقت الطوارئ
    if (carControl.currentState == NORMAL_RC) {
    steering.object.write(finalAngle);
    steering.currentAngle = (float)finalAngle;
}
}

// ==========================================
// --- ACKERMANN  [F5] ---
// ==========================================
void applyAckermann(float baseRPM, float theta_deg) {
    float speedLeft  = baseRPM;
    float speedRight = baseRPM;

    if (fabsf(theta_deg) >= 1.0f) {
        float theta_rad = theta_deg * DEG_TO_RAD;
        float R = fabsf(chassis.L / tanf(theta_rad));

        float R_outer = R + (chassis.w / 2.0f);
        float R_inner = R - (chassis.w / 2.0f);
        float ratio   = (R_inner > 0.0f) ? (R_inner / R_outer) : 0.0f;
        float speed_inner = baseRPM * ratio;

        if (theta_deg > 0.0f) {  // Right turn
            speedLeft  = baseRPM;
            speedRight = speed_inner;
        } else {                  // Left turn
            speedLeft  = speed_inner;
            speedRight = baseRPM;
        }
    }

    portENTER_CRITICAL(&targetMux);
    motorL.targetRPM = speedLeft;
    motorR.targetRPM = speedRight;
    portEXIT_CRITICAL(&targetMux);
}
// ==========================================
//  SONAR  [F4] [US-FIX] ---
// ==========================================

int getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // قياس النبضة بشكل مخصص ومقاوم للمقاطعات
    // حد أقصى للانتظار 6000 ميكروثانية (حوالي 100 سم كحد أقصى للمسافة).
    // ده بيمنع تجميد النظام لمدة 90 مللي ثانية وبيسمح لفلتر الريموت يشتغل بسرعة عالية.
    const unsigned long TIMEOUT = 4000; 
    unsigned long startWait = micros();

    // 1. انتظار أن يصبح طرف الـ Echo مرتفعاً (HIGH)
    while (digitalRead(echoPin) == LOW) {
        if (micros() - startWait > TIMEOUT) return 999;
    }

    // أخذ الختم الزمني بدقة بعد أن يصبح مرتفعاً
    unsigned long pulseStart = micros();

    // 2. انتظار أن يصبح طرف الـ Echo منخفضاً (LOW)
    while (digitalRead(echoPin) == HIGH) {
        if (micros() - pulseStart > TIMEOUT) return 999;
    }

    // أخذ الختم الزمني بدقة بعد أن يصبح منخفضاً
    unsigned long pulseEnd = micros();
    
    // لأننا نستخدم micros()، مقاطعات الريموت (RC Interrupts) التي تحدث 
    // في الخلفية لن تفسد عملية حساب الوقت بعد الآن.
    long duration = pulseEnd - pulseStart;

    int distance = duration * 0.034 / 2;
    
    // تصفية أي أخطاء أو تعليق في الهاردوير
    if (distance <= 0 || distance > 100) return 999; 
    
    return distance;
}
// ==========================================
// --- OBSTACLE AVOIDANCE  [F7] [US-FIX] ---
// ==========================================
void checkObstacles() {
    // تقليص وقت اللفة لـ 30 مللي ثانية لأننا نقرأ حساساً واحداً فقط في كل لفة
    if (millis() - lastSensorCheck >= 30) {
        lastSensorCheck = millis();
        
        static int scanStep = 0;
        int sensorIndex = 0;
        
        // ====================================================
        // جدول الجدولة المفضلة (Priority Scheduling)
        // اللفة 0: أمام | اللفة 1: يسار | اللفة 2: أمام | اللفة 3: يمين | اللفة 4: أمام | اللفة 5: خلف
        // ====================================================
        if (scanStep == 0 || scanStep == 2 || scanStep == 4) {
            sensorIndex = 0; // الحساس الأمامي (يُفحص 3 مرات في الدورة)
        } else if (scanStep == 1) {
            sensorIndex = 2; // الحساس الأيسر
        } else if (scanStep == 3) {
            sensorIndex = 3; // الحساس الأيمن
        } else if (scanStep == 5) {
            sensorIndex = 1; // الحساس الخلفي
        }
        
        scanStep = (scanStep + 1) % 6; // الانتقال للخطوة التالية في الجدول

        // قراءة الحساس الذي عليه الدور فقط
        adas[sensorIndex].distanceCm = getDistance(adas[sensorIndex].trigPin, adas[sensorIndex].echoPin);
        int distance = adas[sensorIndex].distanceCm;

        // ====================================================
        // 1. منطق الحساس الأمامي المطور (فائق السرعة والأمان)
        // ====================================================
        if (sensorIndex == 0) {
            // مسافة أمان عريضة (45 سم) لتتناسب مع اندفاع السيارة السريع للأمام
            if (distance >= 4 && distance <= 45) {
                adas[0].obstacleCount++;
            } else {
                adas[0].obstacleCount = 0; // تصفير فوري لو القراءة سليمة لقتل الـ Noise
            }
            
            if (carControl.currentState == NORMAL_RC) {
                // تأكيد ثنائي متتالي (يستغرق 60ms فقط الآن بفضل الجدولة المفضلة)
                if (adas[0].obstacleCount == 2) {
                    Serial.print("[ADAS] FRONT Obstacle Confirmed at: "); Serial.println(distance);
                    carControl.currentState = AVOIDANCE_OVERRIDE;
                    carControl.stateTimer = millis();
                    carControl.overrideSpeed = 500;  // ارجع للخلف فوراً (موجب)
                    carControl.overrideSteering = 0; // مستقيم
                }
                // إذا استمر العائق لـ 4 قراءات متتالية، فرملة طوارئ كاملة
                else if (adas[0].obstacleCount >= 4) {
                    Serial.println("[ADAS] FRONT Emergency Braking Sequence!");
                    carControl.currentState = BRAKING_SEQUENCE;
                    carControl.stateTimer = millis();
                    carControl.speedBeforeBraking = motorL.measuredRPM;
                }
            }
            // حماية الفخ للحساس الأمامي أثناء الهروب للأمام
            else if (carControl.currentState == AVOIDANCE_OVERRIDE) {
                if (carControl.overrideSpeed < 0 && adas[0].obstacleCount >= 2) {
                    Serial.println("[ADAS] Front blocked during evasion! Reversing.");
                    carControl.overrideSpeed = 500; 
                    if (carControl.overrideSteering > 0) carControl.overrideSteering = -15;
                    else if (carControl.overrideSteering < 0) carControl.overrideSteering = 15;
                    carControl.stateTimer = millis();
                    adas[0].obstacleCount = 0;
                }
            }
        }
        
        // ====================================================
        // 2. منطق الحساسات الجانبية والخلفية (زي ما فات - استجابة لحظية)
        // ====================================================
        else {
            // مسافة أمان ثابتة للجوانب والخلف (20 سم) وتعمل من القراءة الأولى مباشرة لسرعة الاستجابة
            if (distance >= 3 && distance <= 20) {
                adas[sensorIndex].obstacleCount++;
                
                if (carControl.currentState == NORMAL_RC && adas[sensorIndex].obstacleCount == 1) {
                    Serial.print("[ADAS] Side/Back Triggered! Sensor: "); Serial.println(sensorIndex);
                    carControl.currentState = AVOIDANCE_OVERRIDE;
                    carControl.stateTimer = millis();
                    
                    if (sensorIndex == 1) { carControl.overrideSpeed = -500; carControl.overrideSteering = 0; }   // الخلف مسدود -> اتقدم
                    if (sensorIndex == 2) { carControl.overrideSpeed = -500; carControl.overrideSteering = 15; }  // اليسار مسدود -> اهرب يمين
                    if (sensorIndex == 3) { carControl.overrideSpeed = -500; carControl.overrideSteering = -15; } // اليمين مسدود -> اهرب يسار
                }
            } else {
                adas[sensorIndex].obstacleCount = 0; // تصفير العداد لو الطريق فتح
            }
        }
    }
}
// ==========================================
// --- ADAS STATE MACHINE  [F3] ---
// ==========================================
void applyADASLogic(float &baseRPM, float &steeringAngle) {
    unsigned long elapsed = millis() - carControl.stateTimer;

    switch (carControl.currentState) {
        case NORMAL_RC: break;

        case AVOIDANCE_OVERRIDE:
            if (elapsed <= 700) {
                baseRPM = carControl.overrideSpeed;
                steeringAngle = carControl.overrideSteering;
                mySteering.object.write(93 + (int)steeringAngle);
                mySteering.currentAngle = 93.0f + steeringAngle;
                
            } else {
                Serial.println("[ADAS] Evasive Action Complete.");
                carControl.currentState = NORMAL_RC; 
            }
            break;

        case BRAKING_SEQUENCE:
            steeringAngle = 0;
            mySteering.object.write(93);
            mySteering.currentAngle = 93.0f; 
            if (elapsed < 666) baseRPM = carControl.speedBeforeBraking * 0.66; 
            else if (elapsed < 1333) baseRPM = carControl.speedBeforeBraking * 0.33; 
            else if (elapsed < 2000) baseRPM = 0; 
            else if (elapsed < 5000) baseRPM = 0;
            else {
                Serial.println("[ADAS] SYSTEM HALTED.");
                carControl.currentState = SYSTEM_HALTED;
            }
            break;

        case SYSTEM_HALTED:
            if (steerCh.filteredValue <= 1050 && speedCh.filteredValue <= 1050) {
                Serial.println("[ADAS] System Revived by Transmitter!");
                carControl.currentState = NORMAL_RC;
                for(int i = 0; i < 4; i++) {
                    adas[i].obstacleCount = 0;
                    adas[i].lastObstacleTime = 0;
                }
            } else {
                baseRPM = 0;
                steeringAngle = 0;
            }
            break;
    }
}
// ==========================================
// --- DRIVE KINEMATICS ---
// ==========================================
void updateDriveKinematics() {
    int rcVal     = (int)speedCh.filteredValue;
    float baseRPM = map(rcVal, 1000, 2000, (int)(-MAX_RPM), (int)(MAX_RPM));
    if (fabsf(baseRPM) < (MAX_RPM * 0.05f)) baseRPM = 0.0f;  // 5% deadband

    float theta_deg = mySteering.currentAngle - 93.0f;  // [F5] offset from center
    applyADASLogic(baseRPM, theta_deg);
    applyAckermann(baseRPM, theta_deg);
}

// ==========================================
// --- TELEMETRY ---
// ==========================================
void printDebuggingTelemetry() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 100) return;  // Rate-limit to 10 Hz
    lastPrint = millis();

    Serial.print("Tgt_L:");   Serial.print(motorL.targetRPM,  1);
    Serial.print(",Meas_L:"); Serial.print(motorL.measuredRPM, 1);
    Serial.print(",Tgt_R:");  Serial.print(motorR.targetRPM,  1);
    Serial.print(",Meas_R:"); Serial.print(motorR.measuredRPM, 1);
    Serial.print(",State:");  Serial.print((int)carControl.currentState);
    Serial.print(",Steer:");  Serial.println(mySteering.currentAngle - 90.0f, 1);
}

// ==========================================
// --- CORE 0 TASK  [F2, F8] ---
// ==========================================
void Core0Task_Manager(void* pvParameters) {
    unsigned long taskPrevT = micros();

    for (;;) {
        unsigned long currT = micros();
        float deltaT = (float)(currT - taskPrevT) / 1.0e6f;

        if (deltaT >= 0.001f) {  // [F8] 1 ms guard
            processEncoder(motorL, encoderL, deltaT);
            processEncoder(motorR, encoderR, deltaT);

            calculatePI(motorL, deltaT);  // [F2] mutex only inside calculatePI
            calculatePI(motorR, deltaT);

            if (carControl.currentState == SYSTEM_HALTED) {
                setMotorBTS(0, RPWM1, LPWM1);
                setMotorBTS(0, RPWM2, LPWM2);
            } else {
                setMotorBTS(motorL.calcOutput, RPWM1, LPWM1);
                setMotorBTS(motorR.calcOutput, RPWM2, LPWM2);
            }
            taskPrevT = currT;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // [F8] 1 ms matches deltaT guard
    }
}

// ==========================================
// --- SETUP ---
// ==========================================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    setupRCInputPins();
    mySteering.object.attach(SERVO_PIN);

    pinMode(RPWM1, OUTPUT); pinMode(LPWM1, OUTPUT);
    pinMode(RPWM2, OUTPUT); pinMode(LPWM2, OUTPUT);
    analogWriteFrequency(RPWM1, 20000); analogWriteFrequency(LPWM1, 20000);
    analogWriteFrequency(RPWM2, 20000); analogWriteFrequency(LPWM2, 20000);

    // Sonar pins + non-blocking echo ISRs  [F4]
    for (int i = 0; i < 4; i++) {
        pinMode(adas[i].trigPin, OUTPUT);
        pinMode(adas[i].echoPin, INPUT);
    }
   

    // [F9] Initialise both encoder instances via TCA channels
    tcaSelect(0); encoderL.begin(); motorL.prevAngle = (int)encoderL.readAngle();
    tcaSelect(1); encoderR.begin(); motorR.prevAngle = (int)encoderR.readAngle();


    Serial.println("[INIT] System Ready");

    xTaskCreatePinnedToCore(
        Core0Task_Manager,
        "Core0_Motor",
        10000,
        NULL,
        2,
        &Core0TaskHandle,
        0
    );
}

// ==========================================
// --- MAIN LOOP (CORE 1) ---
// ==========================================
void loop() {

    
     
    int current_steer, current_speed;
    getSafeRCValues(current_steer, current_speed);

    applyRCSmoothingFilter(steerCh, current_steer);
    applyRCSmoothingFilter(speedCh, current_speed);
//
    checkObstacles();           // Non-blocking sonar round-robin  [F4][US-FIX]
    updateSteering(mySteering, steerCh);
    updateDriveKinematics();    // Sends RPM targets to Core0 via mutex
    printDebuggingTelemetry();  // Rate-limited to 10 Hz internally

    vTaskDelay(40 / portTICK_PERIOD_MS);}
