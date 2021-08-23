// ros deps
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int64.h"

// drivers
#include "h_bridge_driver.cpp"
#include "encoder_driver.cpp"
#include "servo_driver.cpp"

// other libs
#include <iostream>
#include <cmath>        // std::abs
#include <algorithm>    // std::upper_bound
#include <math.h>       // M_PI

// set H-bridge and servo as global variable to be acessible by callback
HBridge * bridgep;
Servo * servop;

double SetSpeed = 0;
double SetWheelsAngle = 0;
ros::Time set_speed_time;

// tables for wheel angle interpolation
const float wheel_angles[45] = {
    0.         ,  0.75167547, 1.49704683 ,  2.23779056,  2.97547221,  3.71155658,
    4.44741611 ,  5.18433761, 5.92352754 ,  6.6661158 ,  7.41315842,  8.16563879,
    8.92446781 ,  9.69048282, 10.46444521, 11.24703686, 12.0388551 , 12.84040629,
    13.65209768, 14.47422745, 15.30697258, 16.15037421, 17.00431988, 17.86852212,
    18.74249247, 19.62550967, 20.5165809 , 21.4143941 , 22.31725958, 23.22303921,
    24.12906236, 25.03203067, 25.92792095, 26.81191035, 27.67837806, 28.52109147,
    29.33376635, 30.1112774 , 30.85178761, 31.55974958, 32.24892966, 32.94366205,
    33.67660784, 34.483117  , 35.39447277
};
const float rack_positions[45] = {
     0.0       ,  1.64564423,  3.27280288,  4.880724  ,  6.46866376,  8.03588539,
     9.58165834, 11.10525747, 12.60596249, 14.08305732, 15.53582958, 16.96357016,
    18.36557272, 19.74113324, 21.08954962, 22.41012116, 23.70214818, 24.96493141,
    26.19777159, 27.39996881, 28.57082192, 29.70962791, 30.81568108, 31.88827233,
    32.92668818, 33.93020983, 34.89811202, 35.82966179, 36.72411716, 37.58072548,
    38.39872177, 39.17732674, 39.91574456, 40.61316039, 41.2687375 , 41.88161409,
    42.45089954, 42.97567025, 43.45496473, 43.887778  , 44.2730551 , 44.6096835 ,
    44.89648427, 45.13220158, 45.3154904 
};
// speed have to be -1 to 1
void motor_speed_callback(const std_msgs::Float64 speed) {
    // limit speed
    double speed_recv = speed.data;
    set_speed_time = ros::Time::now();
    if (speed_recv > 1.0) {
        speed_recv = 1;
    }
    else if (speed_recv < -1.0) {
        speed_recv = -1;
    }
    SetSpeed = speed_recv;

    return;
}

// sets speed using hbridge, hbridge must be initialized
void set_motor_speed(double speed) {
    unsigned char pwm_speed = (unsigned char)((double)std::abs(speed)*255.0);
    if (speed > 0) {
        bridgep->setSpeed(motorA, clockwise, pwm_speed);
    }
    else {
        bridgep->setSpeed(motorA, counterclockwise, pwm_speed);
    }
    return;
}

// transforms steering rack position to servo angle according to gearing
float rack_pos_to_servo_angle(float rack_pos) {
    return rack_pos/0.555332034 + 90;   // servo angle is 0-180, servo angle to rack pos const 0.555332034
                                        // gear ration 35/22 and 20 mm diameter of last gear
}

// transforms desired steering angle to angle of the servo using interpolation table
float wheels_angle_to_servo_angle(float wheels_angle) {
    float rack_desired_pos;
    size_t array_size = sizeof(wheel_angles)/sizeof(wheel_angles[0]);
    if (wheels_angle >= wheel_angles[array_size - 1]) {
        rack_desired_pos = wheel_angles[array_size - 1];
    }
    else if (wheels_angle <= -wheel_angles[array_size - 1]) {
        rack_desired_pos = -wheel_angles[array_size - 1];
    }
    else if (wheels_angle == 0.0) {
        rack_desired_pos = 0.0;
    }
    else {
        bool is_negative = false;
        if (wheels_angle < 0) {
            wheels_angle = -wheels_angle;
            is_negative = true;
        }
        // find neighbouring values and interpolate
        const float * upper_angle = std::upper_bound(std::cbegin(wheel_angles), std::cend(wheel_angles), wheels_angle);
        const float * lower_angle = upper_angle - 1;

        // find corresponding rack position
        int upper_index = upper_angle - &wheel_angles[0];
        float upper_rack_pos = rack_positions[upper_index];
        float lower_rack_pos = rack_positions[upper_index - 1];

        // interpolate
        float relative_angle = ((wheels_angle - *lower_angle)/((*upper_angle) - (*lower_angle)));
        float rack_delta = upper_rack_pos - lower_rack_pos;
        rack_desired_pos = lower_rack_pos + relative_angle*rack_delta;
        if (is_negative) {
            rack_desired_pos = -rack_desired_pos;
        }
    }
    float ret_servo_angle = rack_pos_to_servo_angle(rack_desired_pos);

    return ret_servo_angle;
}

// transforms angle from radians to degrees
float radians_to_degrees(float angle) {
    return angle * 180.0 / M_PI;
}

float degrees_to_radians(float angle) {
    return angle * M_PI / 180.0;
}

// sets wheel angle using servo, servo must be initialized
void set_wheels_angle(float angle) {
    float set_angle = wheels_angle_to_servo_angle(-angle); // reverse angle
    servop->setAngle(set_angle);
    return;
}

// todo move to the loop
// callback for set angle message, sets angle of the servo
void wheels_set_angle_callback(const std_msgs::Float32 angle) {
    SetWheelsAngle = radians_to_degrees(angle.data);
    return;
}

double steps_to_speed(int64_t steps, int measurements_per_s) {
    // 22 pulses per revolution
    // gear ratio 1:124
    // convert to RPS
    double speed = (double)steps/44.0*measurements_per_s/124;
    // convert to Rad/s
    speed = speed * 2 * M_PI;
    return speed;
}

int main(int argc, char **argv) {
    // set up encoder and h bridge
    bridgep = new HBridge(13, 19, 0, 0); // using only port A
    servop = new Servo(22, 0, 180);
    Encoder encoder(17, 27);

    // initialize ros and set up nodes
    ros::init(argc, argv, "robot_driver_gpio");
    ros::NodeHandle n;
    ros::Publisher robot_driver_gpio_pub = n.advertise<std_msgs::Float64>("main_motor_speed", 1);
    ros::Publisher robot_steering_angle_pub = n.advertise<std_msgs::Float32>("wheels_get_steering_angle", 1);
    ros::Subscriber motor_set_speed_sub = n.subscribe("main_motor_set_speed", 1000, motor_speed_callback);
    ros::Subscriber wheels_set_angle_sub = n.subscribe("wheels_set_steering_angle", 1000, wheels_set_angle_callback);

    uint64_t last_steps = 0;

    // set up loop
    int rate = 30;
    ros::Time ts = ros::Time::now();
    ros::Duration d;

    // set msg_recv_time
    set_speed_time = ros::Time::now();

    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        double commanded_speed = SetSpeed;
        float commanded_wheel_angle = SetWheelsAngle;
        
        // limit commanded angle
        size_t wheel_angles_size = sizeof(wheel_angles)/sizeof(wheel_angles[0]);
        if (commanded_wheel_angle > wheel_angles[wheel_angles_size - 1]) {
            commanded_wheel_angle = wheel_angles[wheel_angles_size - 1];
        }
        else if (commanded_wheel_angle < -wheel_angles[wheel_angles_size - 1]) {
            commanded_wheel_angle = -wheel_angles[wheel_angles_size - 1];
        }


        set_wheels_angle(commanded_wheel_angle);
        set_motor_speed(commanded_speed);


        // create and publish speed message

        std_msgs::Float64 speed_msg;

        // read current number of steps and get speed
        uint64_t steps = encoder.get_steps();

        // update time
        d = ros::Time::now() - ts;
        ts = ros::Time::now();

        int64_t steps_until_last = steps - last_steps;      // todo affect of owerflow
        last_steps = steps;
        double speed = steps_to_speed(steps_until_last, 1/d.toSec());

        // set message
        speed_msg.data = speed;

        // ROS_INFO("Loop rate %f hz", 1/d.toSec());

        robot_driver_gpio_pub.publish(speed_msg);


        // create and publish steering angle message

        std_msgs::Float32 steering_angle_msg;
        steering_angle_msg.data = degrees_to_radians(commanded_wheel_angle);
        robot_steering_angle_pub.publish(steering_angle_msg);

        // chack for motor timeout
        ros::Duration since_last_speed_command = ros::Time::now() - set_speed_time;
        if (since_last_speed_command.toSec() > 0.5) {
            bridgep->setSpeed(motorA, clockwise, 0.0);  // turn off motor
        }

        // let ros call all callbacks
        ros::spinOnce();
        
        // wait for next iteration to send data
        loop_rate.sleep();
    }
    // turn off motor
    bridgep->setSpeed(motorA, clockwise, 0.0);
    
    return 0;
}