// Autor: Ondřej Deingruber
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

// class inplementing communication with nodes controlling the hardware, converting it to standard interfaces
class FitBot : public hardware_interface::RobotHW {
  public:
    FitBot(ros::NodeHandle * nh) : n(nh) {
    robot_main_motor_speed_pid_pub = n->advertise<std_msgs::Float64>("main_motor_set_speed_pid", 1000);
    robot_steering_angle_pub = n->advertise<std_msgs::Float32>("wheels_set_steering_angle", 1000);
    motor_get_speed_sub = n->subscribe("main_motor_speed", 1000, &FitBot::get_motor_speed_callback, this);
    wheels_get_steering_angle_sub = n->subscribe("wheels_get_steering_angle", 1000, &FitBot::get_steering_angle_callback, this);
    
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("imaginary_center_front_wheel_steering_joint", &front_wheel_steering_pos, &front_wheel_steering_vel, &front_wheel_steering_eff); //pos vel eff outputs of the state message...
    jnt_state_interface.registerHandle(state_handle_a);
    hardware_interface::JointStateHandle state_handle_b("imaginary_center_back_wheel_joint", &drive_wheel_pos, &drive_wheel_vel, &drive_wheel_eff); //pos vel eff outputs of the state message...
    jnt_state_interface.registerHandle(state_handle_b);
    registerInterface(&jnt_state_interface);

    

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("imaginary_center_front_wheel_steering_joint"), &front_wheel_steering_cmd); //cmd is the commanded value depending on the controller.
    jnt_pos_interface.registerHandle(pos_handle_a);
    registerInterface(&jnt_pos_interface);

    hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("imaginary_center_back_wheel_joint"), &drive_wheel_cmd); //cmd is the commanded value depending on the controller.
    jnt_vel_interface.registerHandle(vel_handle_b);
    registerInterface(&jnt_vel_interface);
  }

    virtual ~FitBot()
    {}

    void write() {
      // send speed message
      std_msgs::Float64 speed_msg;
      speed_msg.data = drive_wheel_cmd;
      robot_main_motor_speed_pid_pub.publish(speed_msg);

      // send wheel angle message
      std_msgs::Float32 angle_msg;
      angle_msg.data = front_wheel_steering_cmd;
      robot_steering_angle_pub.publish(angle_msg);
    }

    // callback to update motor speed
    void get_motor_speed_callback(const std_msgs::Float64 speed) {
        drive_wheel_vel = speed.data;
        return;
    }

    // callback to update steering angle
    void get_steering_angle_callback(const std_msgs::Float32 angle) {
        front_wheel_steering_pos = angle.data;
        return;
    }


    void read(ros::Duration d) {
      drive_wheel_pos += drive_wheel_vel * d.toSec();
      front_wheel_steering_vel = (front_wheel_steering_pos - front_wheel_steering_last_pos) / d.toSec();
      return;
    }

  private:
    ros::NodeHandle * n;

    // interfaces
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;

    // state variables
    double front_wheel_steering_last_pos = 0;
    double front_wheel_steering_pos = 0;
    double front_wheel_steering_vel = 0;
    double front_wheel_steering_eff = 0;
    double front_wheel_steering_cmd = 0;
    double drive_wheel_pos = 0;
    double drive_wheel_vel = 0;
    double drive_wheel_eff = 0;
    double drive_wheel_cmd = 0;

    // ros publishers and listeners
    ros::Publisher robot_main_motor_speed_pid_pub;
    ros::Publisher robot_steering_angle_pub;
    ros::Subscriber motor_get_speed_sub;
    ros::Subscriber wheels_get_steering_angle_sub;
};


#include <ros/callback_queue.h>
int main(int argc, char** argv)
{
  // init
  ros::init(argc, argv, "test_iface_node");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  // set hw interfaces
  FitBot robot(&nh);
  controller_manager::ControllerManager cm(&robot,nh);

  // set up spinner
  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  ros::Time ts = ros::Time::now();

  // start loop
  ros::Rate rate(30);
  while (ros::ok())
  {
     ros::Duration d = ros::Time::now() - ts;
     ts = ros::Time::now();
     robot.read(d);
     cm.update(ts, d);
     robot.write();
     rate.sleep();
  }

  spinner.stop();

  return 0;
}
