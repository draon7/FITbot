// Autor: Ondřej Deingruber
#include <pigpiod_if2.h>
#include <iostream>

class Servo {
    public:
        Servo(unsigned int servo_pin, float min_angle, float max_angle);
        void setAngle(float angle);
    private:
        unsigned int servo_pin;
        float min_angle;
        float max_angle;
        int daemon_id;
        unsigned int angle_to_pwm(float angle);
};

Servo::Servo(unsigned int servo_pin, float min_angle, float max_angle) : servo_pin(servo_pin), min_angle(min_angle),
        max_angle(max_angle) {

    daemon_id = pigpio_start(NULL, NULL);
    set_mode(daemon_id, servo_pin, PI_OUTPUT);
    return;
}

void Servo::setAngle(float angle) {
    set_servo_pulsewidth(daemon_id, servo_pin, angle_to_pwm(angle));
    return;
}

unsigned int Servo::angle_to_pwm(float angle) {
    // limit angle to set limits
    if (angle < min_angle) {
        angle = min_angle;
    }
    else if (angle > max_angle) {
        angle = max_angle;
    }
    
    // chack if range isn't given in bad order
    float range = max_angle - min_angle;
    if (range < 0) {    // abs(range), sanity chack
        range = -range;
    }

    // servo command values 500 - 2500
    unsigned int pwm = ((float)(angle - min_angle) / (float)(range/2000.0)) + 500.0;

    return pwm;
}
