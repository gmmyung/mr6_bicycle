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


#define SERIAL_PRINT false
#define PID_PRINT false

#define yaw_I false

MPU6050 mpu;

#define MPU_INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define ENCODER_INTERRUPT_PIN 3
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define e 2.71828
#define g 9.8   //중력가속도
#define b 1.0   //바퀴사이의 거리
#define h 1.0   //무게중심 높이
#define v 1.0   //자전거의 속도
#define D 1.0   //자전거의 관성모멘트
#define fai 1.0 //자전거의 기울어진 각도
#define t 1.0   //시간?
#define m 1.0   //자전거 질량

#define SAFE_SERVO_RANGE 45

#define BTRX 0
#define BTTX 1


/* data 참고
static float kp = 3.0;
static float ki = 0.1;
static float kd  = 0.000000;
*/

float constants[4] = {0.0, 0.0, 0.0, 0.0}; // kp, ki, kd, roll offset

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

float init_servo_offset = 0.0;
float servo_offset = 0.0;
float current_vel = 0.0;

int32_t global_gyroX;
float global_yaw = 0.0;
float global_roll = 0.0;

float motor_spd = 230;
float roll_unit_step = 2.0;
float roll_target_num = 0.0;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
    // DebugPrint("interrupt!\n");
}

int photoPin = 3;
int motor_direction1 = 7;
int motor_direction2 = 8;
int motor_spd_pin = 6;
volatile float currentVel = 0.0;
volatile unsigned long lastHitTime;

boolean fitting_mode = false; //determine will it ask kp, ki, kd, and roll offset at every initalization.

RunningAverage velMovAvg(20);
Servo steeringServo;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

float prev = millis();
float now = millis();

void setup()
{
    velMovAvg.clear();
    pinMode(photoPin, INPUT);

    pinMode(motor_direction1, OUTPUT);
    pinMode(motor_direction2, OUTPUT);
    pinMode(motor_spd_pin, OUTPUT);

    digitalWrite(motor_direction1, HIGH);
    digitalWrite(motor_direction2, LOW);

    attachInterrupt(digitalPinToInterrupt(ENCODER_INTERRUPT_PIN), counting, RISING);

    steeringServo.attach(9);

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Wire.setWireTimeout(3000, true);

    // initialize serial communication
    Serial.begin(9600);
    Serial.println("serial connected");
    while (!Serial)
        ;

    bool go_next = false;
    Serial.println("fitting mode?: (y/n)");
    while (!go_next)
    {
        if (Serial.available())
        {
            String d = Serial.readStringUntil('\n');
            Serial.println(d);
            if (d.equals("y"))
            {
                go_next = true;
                fitting_mode = true;
            }
            else if (d.equals("n")) 
            {
                go_next = true;
                fitting_mode = false;
            }
        }
    }

    if (fitting_mode)
    {
        Serial.println("input k values");
        for (int i = 0; i < 3; i++)
        {
            while (!Serial.available())
                ;
            constants[i] = Serial.readStringUntil('\n').toFloat();
        }
        Serial.println("input roll offset");
        while (!Serial.available())
            ;
        constants[3] = Serial.readStringUntil('\n').toFloat();
    }
    else 
    {
        Serial.println("changed constants according to pre-setted values");
        adaptive_constant_changer(motor_spd);
    }
    StatusPrint();
    
    Serial.println("input servo offset");
    while (!Serial.available())
        ;
    servo_offset = Serial.readStringUntil('\n').toFloat();
    Serial.println(F("Initializing I2C devices..."));

    mpu.initialize();
    pinMode(MPU_INTERRUPT_PIN, INPUT);

    // verify connection

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));

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

        mpuIntStatus = mpu.getIntStatus();

        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));

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

void DebugPrint(const char *c)
{
#if SERIAL_PRINT == true
    Serial.print(c);
#endif
}
void DebugPrint(int i)
{
#if SERIAL_PRINT == true
    Serial.print(i);
#endif
}
void DebugPrint(int32_t i)
{
#if SERIAL_PRINT == true
    Serial.print(i);
#endif
}
void DebugPrint(float f)
{
#if SERIAL_PRINT == true        
    Serial.print(f);
#endif
}
void DebugPrint(double d)
{
#if SERIAL_PRINT == true
    Serial.print(d);
#endif
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    if (!dmpReady)
        return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
        // Serial.print(millis() / 1000.0);
        analogWrite(motor_spd_pin, motor_spd);
        counting();
        // if programming failed, don't try to do anything
        DebugPrint("currentVel ");
        DebugPrint(velMovAvg.getAverage());
        DebugPrint(" ");
        mpu.dmpGetGyro(data, fifoBuffer);
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }

    // DebugPrint("ypr\t");
    // DebugPrint(ypr[0] * 180 / M_PI);
    // DebugPrint("\t");
    // DebugPrint(ypr[1] * 180 / M_PI);
    // DebugPrint("\t");
    // DebugPrint(ypr[2] * 180 / M_PI); // roll 2번

    // DebugPrint("data\t");
    // DebugPrint(data[0]);
    // DebugPrint("\t");
    // DebugPrint(data[1]);
    // DebugPrint("\t");
    // DebugPrint(data[2]); // roll 2번

    safeServo(calc_pid(data[0], ypr[0] * 180 / M_PI, ypr[2] * 180 / M_PI, roll_target_num * roll_unit_step + constants[3]), steeringServo);
    
    blinkState = (millis() / 1000) % 2;
    digitalWrite(LED_PIN, blinkState);
    if (Serial.available())
    {
        String data = Serial.readStringUntil('\n');
        if (data.equals("l"))
        {
            roll_target_num += 1;
        }
        else if (data.equals("r"))
        {
            roll_target_num -= 1;   
        }
        else if (data.equals("c"))
        {
            roll_target_num = 0;
            Serial.println("roll target initalized");
        }
        else if (data.equals("u"))
        {
            motor_spd += 5;
            if (!fitting_mode) 
            {
                adaptive_constant_changer(motor_spd);
            }
        }
        else if (data.equals("d"))
        {
            motor_spd -= 5;
            if (!fitting_mode) 
            {
                adaptive_constant_changer(motor_spd);
            }
        }
        if (motor_spd > 255)
        {
            motor_spd = 255;
        }
        else if (motor_spd < 100)
        {
            motor_spd = 100;
        }
        StatusPrint();
    }

}

void safeServo(float angle, Servo servo)
{
    const int safe_min = 90 - SAFE_SERVO_RANGE;
    const int safe_max = 90 + SAFE_SERVO_RANGE;
    float real_angle = angle + 90 + servo_offset;
    if (safe_min > real_angle)
    {
        servo.write(safe_min);
    }
    else if (safe_max < real_angle)
    {
        servo.write(safe_max);
    }
    else
    {
        servo.write(real_angle);
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
    const float maxInteg = 2;
    unsigned long currentTime;
    static long error_sum = 0;
    float target_degree;
    float error;
    float de;
    float dt;
    float angle;
    float kp, kd, ki;

    kp = constants[0];
    ki = constants[1];
    kd = constants[2];

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
    DebugPrint(" p: ");
    DebugPrint(kp * error);
    DebugPrint(" d: ");
    DebugPrint(kd * gyroX);
    DebugPrint(" i: ");
    DebugPrint(ki * error_sum);
    DebugPrint(" angle: ");
    DebugPrint(angle);
    return angle; //최종적으로 돌아가야되는 서보모터 각도
}

float calc_degree()
{
    float degree;
    degree = g * b * sin(fai) / (v * v) * (1 - pow(e, (h * v * m * t / D)));
    return degree;
}

void adaptive_constant_changer(float m_spd) 
{
    int bound_num = 2; //except last bound
    float spd_bound[bound_num + 1] = {120.0, 150.0, 255.0};
    float mapped_constants[bound_num + 1][4] = {{3.0, 0.05, 0.0, 0.75},
                                                {2.8, 0.0, 0.0, 0.5},
                                                {2.5, 0.0, 0.0, 0.0}}; //preset {kp, ki, kd}

    for (int i = 0; i < bound_num + 1; i++)
    {
        if (m_spd < spd_bound[i])
        {
            for (int j = 0; j < 4; j++) 
            {
                constants[j] = mapped_constants[i][j];
            }
            break;
        }
    }
}

void StatusPrint() 
{
    //setting
    Serial.print(" roll_target: ");
    Serial.println(roll_target_num * roll_unit_step);

    Serial.print(" kp: ");
    Serial.print(constants[0]);
    Serial.print(" ki: ");
    Serial.print(constants[1]);
    Serial.print(" kd: ");
    Serial.println(constants[2]);
    
    Serial.print(" motor speed: ");
    Serial.print(motor_spd / 255 * 100.0);
    Serial.print("% ");
    Serial.println(motor_spd);


    //values
    /*
    Serial.print(" speed: ");
    Serial.print(currentVel);
    */

}
