// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <RunningAverage.h>
#include <Servo.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define serial_enable false
#define bt_serial_enable true
#define yaw_I true

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:
   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define e 2.71828
#define g 9.8   //중력가속도
#define b 1.0   //바퀴사이의 거리
#define h 1.0   //무게중심 높이
#define v 1.0   //자전거의 속도
#define D 1.0   //자전거의 관성모멘트
#define fai 1.0 //자전거의 기울어진 각도
#define t 1.0   //시간?
#define m 1.0   //자전거 질량

#define btRX 0
#define btTX 1

static float kp = 3.0;
static float ki = 0.1;
static float kd  = 0.000000; //global variable

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
int32_t data[3];
float euler[3]; //[psi, theta, phi]    Euler angle container
float ypr[3];   //[yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float angle;
void safeServo(float angle, Servo servo);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

int photoPin = 3;
int motor_direction1 = 7;
int motor_direction2 = 8;
int motor_speed = 6;
volatile float currentVel = 0.0;
volatile unsigned long lastHitTime;

RunningAverage velMovAvg(20);
Servo steeringServo;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
    velMovAvg.clear();
    pinMode(photoPin, INPUT);
    pinMode(motor_direction1, OUTPUT);
    pinMode(motor_direction2, OUTPUT);
    pinMode(motor_speed, OUTPUT);
    digitalWrite(motor_direction1, LOW);
    digitalWrite(motor_direction2, HIGH);
    attachInterrupt(1, counting, RISING);

    steeringServo.attach(9);

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
#if serial_enable == true
    Serial.begin(57600);
    while (!Serial)
        ; // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
#endif
#if bt_serial_enable == true
    Serial.begin(9600);
    bool go_next;
    while (!go_next) {
        if (Serial.available()) {
            if (Serial.readString().compareTo("go")){
                go_next = true;
            }
        }
    }
    Serial.print("bt serial starts");

#endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection

#if serial_enable == true || bt_serial_enable == true
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
#else
    mpu.testConnection();
    devStatus = mpu.dmpInitialize();
#endif

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    steeringServo.write(90);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    Serial.print("loop ");
    analogWrite(motor_speed, 230);
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
#if serial_enable == true || bt_serial_enable == true
        Serial.print("currentVel ");
        Serial.print(velMovAvg.getAverage());
        Serial.print(" ");
#else
        velMovAvg.getAverage(); // get value anywhere
#endif
        mpu.dmpGetGyro(data, fifoBuffer);
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

#if serial_enable == true || bt_serial_enable == true
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[2] * 180 / M_PI); // roll 2번
#endif
        Serial.print("data\t");
        Serial.print(data[0]);
        Serial.print("\t");
        Serial.print(data[1]);
        Serial.print("\t");
        Serial.print(data[2]); // roll 2번
        safeServo(calc_pid(data[0], ypr[0]* 180 / M_PI, ypr[2] * 180 / M_PI, 0), steeringServo);

        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

#if bt_serial_enable == true
    if ((Serial.available())) { //string 
        if (Serial.readStringUntil('\n').compareTo("change")) {
            for (int i = 0; i < 3; i++) {
                kd = Serial.readStringUntil('\n').toFloat();
                kp = Serial.readStringUntil('\n').toFloat();
                ki = Serial.readStringUntil('\n').toFloat();
            }
        }
    }
#endif

    }
}

void safeServo(float angle, Servo servo)
{
    const int safe_min = 60;
    const int safe_max = 130;
    if (safe_min > angle + 90)
    {
        servo.write(safe_min);
    }
    else if (safe_max < angle + 90)
    {
        servo.write(safe_max);
    }
    else
    {
        servo.write(angle + 90);
    }
}

void counting()
{
    unsigned long currentTime = micros();
    currentVel = 1 / 20.0 / (((float)(currentTime - lastHitTime) / 1000000) / 60);
    lastHitTime = currentTime;
    velMovAvg.addValue(currentVel);
}

float calc_pid(int32_t gyroX, float yaw, float roll, float target)
{
    static unsigned long lastTime = 0;
    const float maxInteg = 5;
    unsigned long currentTime;
    static long error_sum = 0;
    float target_degree;
    float error;
    float de;
    float dt;
    float angle;

    currentTime = millis();
    error = target - roll;
#if yaw_I == false
    dt = currentTime - lastTime;
    error_sum += error * dt;
    error_sum = min(maxInteg / ki, error_sum);
    error_sum = max(-maxInteg / ki, error_sum);
#else
    error_sum = yaw;
#endif
    angle = kp * error + kd * gyroX + ki * error_sum;
#if serial_enable == true || bt_serial_enable == true
    Serial.print(" kp: ");
    Serial.print(kp * error);
    Serial.print(" kd: ");
    Serial.print(kd * gyroX);
    Serial.print(" ki: ");
    Serial.print(ki * error_sum);
    Serial.print(" angle: ");
    Serial.println(angle);
#endif
    return angle; //최종적으로 돌아가야되는 서보모터 각도
}

float calc_degree()
{
    float degree;
    degree = g * b * sin(fai) / (v * v) * (1 - pow(e, (h * v * m * t / D)));
    return degree;
}