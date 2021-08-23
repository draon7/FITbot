// Autor: Ondřej Deingruber
#include <pigpiod_if2.h>

class Encoder {
    public:
        Encoder(unsigned int pinA, unsigned int pinB);
        int64_t get_steps();
        ~Encoder();
        
    private:
        int64_t steps;
        static void pulseInterruptHandle(int pi, unsigned int gpio, unsigned int level, unsigned int tick, void *userdata);
        unsigned int pinA;
        unsigned int pinB;
        int stateA;
        int stateB;
        int lastGpio;
        int daemon_id;
        int callback_pinA;
        int callback_pinB;
};

Encoder::Encoder(unsigned int pinA, unsigned int pinB) : steps(0), pinA(pinA), pinB(pinB), stateA(0), stateB(0) {
    // connect to library
    daemon_id = pigpio_start(NULL, NULL);
    
    // set pins as input
    set_mode(daemon_id, pinA, PI_INPUT);
    set_mode(daemon_id, pinB, PI_INPUT);

    // turn off pullup / pulldown
    set_pull_up_down(daemon_id, pinA, PI_PUD_UP);
    set_pull_up_down(daemon_id, pinB, PI_PUD_UP);

    // set interupt handeler
    callback_pinA = callback_ex(daemon_id, pinA, EITHER_EDGE, pulseInterruptHandle, this);
    callback_pinB = callback_ex(daemon_id, pinB, EITHER_EDGE, pulseInterruptHandle, this);
}

Encoder::~Encoder() {
    // cancel callback
    callback_cancel(callback_pinA);
    callback_cancel(callback_pinB);
    // stop daemon
    pigpio_stop(daemon_id);
}

/*

             +---------+         +---------+      1
             |         |         |         |
   A         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 0

       +---------+         +---------+            1
       |         |         |         |
   B   |         |         |         |
       |         |         |         |
   ----+         +---------+         +---------+  0

*/

void Encoder::pulseInterruptHandle(int pi, unsigned int gpio, unsigned int level, __attribute__ ((unused)) unsigned int tick, void *userdata) {
    Encoder *encoder = reinterpret_cast<Encoder*>(userdata); 
    if (level == 2 || pi != encoder->daemon_id) {       // watchdog timeout
        return;
    }
    if (gpio == encoder->pinA) {        // interupt on pin A
        if (encoder->stateB == 1) {
            if (level == 0) {           // high-low 
                encoder->steps--;
            }
            else if(level == 1)         // level == 1, low-high
                {encoder->steps++;}
        }
        else {  // stateB == 0
            if (level == 0)             // high-low
                {encoder->steps++;}
            else if(level == 1)         // level == 1, low-high
                {encoder->steps--;}
        }
        encoder->stateA = level;
    }
    else {  // pinB, interrupt on pin B
        if (encoder->stateA == 1) {
            if (level == 0)             // high-low
                {encoder->steps++;}
            else if(level == 1)         // level == 1, low-high
                {encoder->steps--;}
        }
        else {  // stateA == 0
            if (level == 0) // high-low
                {encoder->steps--;}
            else if(level == 1)         // level == 1, low-high
                {encoder->steps++;}
        }
        encoder->stateB = level;
    } 
}

int64_t Encoder::get_steps() {
    return steps;
}