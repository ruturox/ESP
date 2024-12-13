
/*
void setServoAngle(int angle, int servoIndex) {
    // Prüfen, ob der Winkel gültig ist
    if (angle < 0 || angle > 180) return;

    uint8_t lastValueServo;
    uint8_t chanel;

    // Servo-Index überprüfen und Werte setzen
    switch (servoIndex) {
        case 1:
            chanel = CHANEL_SERVO_1;
            lastValueServo = servo1LastValue;
            servo1LastValue = angle;
            break;
        case 2:
            chanel = CHANEL_SERVO_2;
            lastValueServo = servo2LastValue;
            servo2LastValue = angle;
            break;
        default:
            return; // Ungültiger Servo-Index
    }

    // Richtung bestimmen
    int multi = lastValueServo < angle ? 1 : -1;
    int from = map(lastValueServo, 0, 180, 0, 1023);
    int to = map(angle, 0, 180, 0, 1023);
    Serial.println(from);
    Serial.println(to);

    // Sanfte Bewegung vom aktuellen zum Zielwinkel
    for (int i = from; i != to + multi; i += multi) {
        Serial.println(i);
        ledcWrite(chanel, i);
        RemoteXY_delay(10); // Wartezeit für sanfte Bewegung
    }
}
*/

#include <ESP32Servo.h>

//RemoteXYValues
#define REMOTEXY_MODE__ESP32CORE_BLE
#define REMOTEXY_BLUETOOTH_NAME "BesteAuto"

//Pins
#define SENSOR1_PIN GPIO_NUM_2
#define SENSOR2_PIN GPIO_NUM_3
#define MOTOR1_PIN_1 0
#define MOTOR1_PIN_2 5
#define MOTOR2_PIN_1 6
#define MOTOR2_PIN_2 7
#define SERVO1_PIN 9
#define SERVO2_PIN 8

//Channels
#define SERVO1_CHANAL 0
#define SERVO2_CHANAL 1
#define MOTOR1_CHANEL_1 2
#define MOTOR1_CHANEL_2 3
#define MOTOR2_CHANEL_1 4
#define MOTOR2_CHANAL_2 5

//Zeiten
#define TIMING_1 1000
#define TIMING_2 1000
#define TIMING_3 1000

//Time the Rover drives backwords
#define WAITTIME 1000

//Intervall to read Sensor in ms
#define TESTINTERVALL 20

//Coolour Treshholdvalue
#define TRESHHOLD 3500

//Motor Values
#define MOTOR1_FAKTOR 1.0f
#define MOTOR2_FAKTOR 0.95f
#define X_ACHSE_FAKTOR 2.56f
#define Y_ACHSE_FAKTOR 5.12f

//PWM Values
#define RESOLUTION 10
#define FRE 50

//Servo Vaöies
#define MIN_US 650
#define MAX_US 2350
#define SERVO1_POS1 145
#define SERVO1_POS2 177
#define SERVO2_POS1 0
#define SERVO2_POS2 180

//Variables
unsigned long lastCommand = 0;
unsigned long wating = TIMING_1;
int waitForCommand = 0;
unsigned long lastime = 0;
Servo servoUnten; //1
Servo servoOben; //2
uint8_t servoChannels[] = {MOTOR1_CHANEL_1, MOTOR1_CHANEL_2, MOTOR2_CHANEL_1, MOTOR2_CHANAL_2};
uint8_t servoPins[] = {MOTOR1_PIN_1, MOTOR1_PIN_2, MOTOR2_PIN_1, MOTOR2_PIN_2};

#include <RemoteXY.h>

#pragma pack(push, 1)

uint8_t RemoteXY_CONF[] =
  { 255,4,0,0,0,48,0,19,0,0,0,0,31,1,106,200,1,1,3,0,
  5,2,1,104,104,0,2,26,31,2,3,117,101,36,0,2,26,31,31,79,
  78,0,79,70,70,0,1,39,163,32,32,0,2,31,0 };

struct {
    int8_t joy_x; // from -100 to 100
    int8_t joy_y; // from -100 to 100
    uint8_t sw_on; // =1 if switch ON and =0 if OFF
    uint8_t bt_on; // =1 if button pressed, else =0

    uint8_t connect_flag;  // =1 if wire connected, else =0
} RemoteXY;

#pragma pack(pop)

void setupPWM() {
    for (int i = 0; i < 4; i++) {
        ledcSetup(servoChannels[i], FRE, RESOLUTION);
        ledcAttachPin(servoPins[i], servoChannels[i]);
    }
}

void setupServos() {
    ESP32PWM::allocateTimer(SERVO1_CHANAL);
    ESP32PWM::allocateTimer(SERVO2_CHANAL);
    servoOben.setPeriodHertz(50);
    servoUnten.setPeriodHertz(50);
    servoOben.attach(SERVO1_PIN, MIN_US, MAX_US);
    servoUnten.attach(SERVO2_PIN, MIN_US, MAX_US);
}

void setServoAngle(int pAngle, int pServoIndex) {

    //Cheack Angle
    if (pAngle < 0 || pAngle > 180) return;

    switch (pServoIndex) {
        case 1:
            servoUnten.write(pAngle);
        break;
        case 2:
            servoOben.write(pAngle);
        break;
        default:
            break;
    }
}

void setup() {
    setupServos();
    setupPWM();
    RemoteXY_Init();
    Serial.begin(115200);
    lastCommand = millis();
    setServoAngle(SERVO1_POS1, 1);
}

void setMotorSpeed(int16_t pSpeed, int chanel1, int chanel2, int pMotorIndex) {
    pSpeed = constrain(pSpeed, -1023, 1023);
    pSpeed = pMotorIndex == 1 ? pSpeed * MOTOR1_FAKTOR : pSpeed * MOTOR2_FAKTOR;

    ledcWrite(chanel1, pSpeed > 0 ? pSpeed : 0);
    ledcWrite(chanel2, pSpeed < 0 ? abs(pSpeed) : 0);
}

void calculateAndWriteMotorSpeed(int16_t joy_x, int16_t joy_y, bool isOn) {
    int16_t motorSpeedX = isOn ? (abs(joy_x) < 8 ? 0 : joy_x * 2) : 0;
    int16_t motorSpeedY = isOn ? (abs(joy_y) < 8 ? 0 : joy_y + (joy_y < 0 ? -100 : 100)) : 0;

    int16_t motor1 = motorSpeedY * Y_ACHSE_FAKTOR + motorSpeedX * X_ACHSE_FAKTOR;
    int16_t motor2 = motorSpeedY * Y_ACHSE_FAKTOR - motorSpeedX * X_ACHSE_FAKTOR;

    setMotorSpeed(motor2, 2, 3, 1);
    setMotorSpeed(motor1, 4, 5, 2);
}

void loop() {
    RemoteXY_Handler();
    if (millis() - lastime >= TESTINTERVALL) {
        if (analogRead(SENSOR2_PIN) > TRESHHOLD || analogRead(SENSOR1_PIN) > TRESHHOLD) {
            ledcWrite(MOTOR1_CHANEL_1, 0);
            ledcWrite(MOTOR1_CHANEL_2, 1023);
            ledcWrite(MOTOR2_CHANEL_1, 0);
            ledcWrite(MOTOR2_CHANAL_2, 1023);
            delay(WAITTIME);
            ledcWrite(MOTOR1_CHANEL_1, 0);
            ledcWrite(MOTOR1_CHANEL_2, 0);
            ledcWrite(MOTOR2_CHANEL_1, 0);
            ledcWrite(MOTOR2_CHANAL_2, 0);
        }
        lastime = millis();
    }

    if (RemoteXY.connect_flag == 1) {
        calculateAndWriteMotorSpeed(RemoteXY.joy_x, RemoteXY.joy_y, RemoteXY.sw_on);
    }

    if (waitForCommand != -1) {
        if (millis() - lastCommand >= wating) {
            switch (waitForCommand) {
                case 0:
                    setServoAngle(SERVO2_POS1, 2);
                    waitForCommand ++;
                    wating = TIMING_2;
                    lastCommand = millis();
                    break;
                case 1:
                    setServoAngle(SERVO1_POS2, 1);
                    waitForCommand ++;
                    wating = TIMING_3;
                    lastCommand = millis();
                    break;
                case 2:
                    setServoAngle(SERVO2_POS2, 2);
                    waitForCommand = -1;
                    break;
            }
        }
    }

    static bool isPressed = false;
    if (RemoteXY.bt_on == HIGH && !isPressed) {
        if (waitForCommand == -1) {
            isPressed = HIGH;
            lastCommand = millis();
            setServoAngle(SERVO1_POS1, 1);
            waitForCommand = 0;
            wating = TIMING_1;
        }
    } else if (RemoteXY.bt_on == LOW) {
        isPressed = false;
    }
}
