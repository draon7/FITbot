// Autor: Ondřej Deingruber
#include <pigpiod_if2.h>

enum direction {
    clockwise,
    counterclockwise
};

enum motor {
    motorA,
    motorB
};

class HBridge {
    public: 
        HBridge(int pwmA1pin, int pwmA2pin, int pwmB1pin, int pwmB2pin);
        void setSpeed(motor mot, direction dir, unsigned char speed);
    private:
        int pwmA1pin;
        int pwmA2pin;
        int pwmB1pin;
        int pwmB2pin;
        int daemon_id;
};

HBridge::HBridge(int pwmA1pin, int pwmA2pin, int pwmB1pin, int pwmB2pin) : pwmA1pin(pwmA1pin), pwmA2pin(pwmA2pin),
        pwmB1pin(pwmB1pin), pwmB2pin(pwmB2pin) {
    
    daemon_id = pigpio_start(NULL, NULL);
    
    // set pins as output
    set_mode(daemon_id, pwmA1pin, PI_OUTPUT);
    set_mode(daemon_id, pwmA2pin, PI_OUTPUT);
    set_mode(daemon_id, pwmB1pin, PI_OUTPUT);
    set_mode(daemon_id, pwmB2pin, PI_OUTPUT);
}

void HBridge::setSpeed(motor mot, direction dir, unsigned char speed) {
    if (mot == motorA) {
        if (dir == clockwise) {
            set_PWM_dutycycle(daemon_id, pwmA2pin, 0);   // disable second channel to prevent short
            set_PWM_dutycycle(daemon_id, pwmA1pin, speed);
        }
        else { // counterclockwise
            set_PWM_dutycycle(daemon_id, pwmA1pin, 0);   // disable second channel to prevent short
            set_PWM_dutycycle(daemon_id, pwmA2pin, speed);
        }
    }
    else {  // motor B
        if (dir == clockwise) {
            set_PWM_dutycycle(daemon_id, pwmB2pin, 0);   // disable second channel to prevent short
            set_PWM_dutycycle(daemon_id, pwmB1pin, speed);
        }
        else { // counterclockwise
            set_PWM_dutycycle(daemon_id, pwmB1pin, 0);   // disable second channel to prevent short
            set_PWM_dutycycle(daemon_id, pwmB2pin, speed);
        }
    }
}