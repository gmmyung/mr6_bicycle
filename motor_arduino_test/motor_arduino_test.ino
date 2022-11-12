#define motor_dir1 7
#define motor_dir2 8
#define motor_spd 6
#define servo 9

#include <Servo.h>
Servo s;
void setup() {
    pinMode(motor_dir1, OUTPUT);
    pinMode(motor_dir2, OUTPUT);
    pinMode(motor_spd, OUTPUT);
    digitalWrite(motor_dir1, HIGH);
    digitalWrite(motor_dir1, LOW);

    s.attach(9);
}

void loop() {
    analogWrite(motor_spd, 100);
}