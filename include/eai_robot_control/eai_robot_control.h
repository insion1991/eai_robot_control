#include "ros/ros.h"
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

using namespace std;

class RobotControl
{
public:
    RobotControl();
    ~RobotControl();

    void moveCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
    float real_turn_angle(float turn_angle);//计算在一个周期内转动的角度
    void getImuCallback(const std_msgs::Float32::ConstPtr& msg);
    void getOdomCallback(const nav_msgs::Odometry::ConstPtr& data);

    bool move_rotate(float angle);
    bool move_linear(float linear_distance);
    void excute_rotate();
    void excute_linear();
    void publishRobotState(const ros::TimerEvent&);
    int getRobotState();
    bool move_linear_with_adjust(float linear_distance,float angle_adjust);
    void excute_linear_with_adjust();
    float getCurrentImuAngle();
    bool run_straight_line(geometry_msgs::Pose start_pose,geometry_msgs::Pose target_pose);
    void laserPauseCallback(const std_msgs::Int16::ConstPtr &msg);
    void laserChangeRobotState(const ros::TimerEvent&);


private:
    float current_robot_angel,last_robot_angle,target_rotate_angele;
    float one_rotate_angle;
    float curren_rotate_speed,min_rotation_speed,max_rotation_speed;
    float rotate_accel,rotate_decel;
    float move_period;
    float primary_threshold,secondary_threshold;
    float angle_offset; //转动允许的偏差
    float angle_threshold; //判断是打大角度转动，还是小角度转动阀值
    float remain_turn_angle,target_angle;
    float turn_angle;
    float w;
    int count;

    float target_distance,remain_distance,distance_threshold,run_distance;
    nav_msgs::Odometry start_odom,current_odom;
    float current_linear_v;
    float move_accel,move_decel;
    float max_linear_speed,min_linear_speed;
    float v,dx2,dy2;
    float angle_adjust;

    ros::NodeHandle nh_,imu_nh_;
    ros::Subscriber move_cmd,imu_data,odom_data,laser_Pause_switch;
    ros::Publisher cmd_vel_pub,robot_state_pub;
    geometry_msgs::Twist robot_cmd_vel;
    ros::Timer timer1,laserChangeRobotStateTimer2;

    bool laserPauseSwitch;

public:
    enum RunState
    {
        STOP          = 0,
        ROTATION      = 1,
        LINEAR        = 2,
        LINEAR_ADJUST = 3,
        PAUSE         = 4,
        CANCEL        = 5
    }robot_state,last_robot_state;
    bool excuteState;

};
