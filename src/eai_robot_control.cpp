#include "eai_robot_control/eai_robot_control.h"

RobotControl::RobotControl()
{
    cout<<"**start RobotControl"<<endl;
    current_robot_angel=last_robot_angle=0.0;
    one_rotate_angle=0.0;
    curren_rotate_speed=0.0;
    min_rotation_speed=0.1;
    max_rotation_speed=0.5;
    rotate_accel=0.8;
    rotate_decel=1.6;
    move_period=0.1;
    primary_threshold=0.3;
    secondary_threshold=0.2;
    angle_offset=0.01; //单位弧度，即0.9度,0.1弧度为6度
    angle_threshold=0.15; //单位弧度，即9度
    w=0;

    target_distance=0.0;
    remain_distance=0.0;
    distance_threshold=0.01;
    move_accel=0.8;
    move_decel=1.6;
    max_linear_speed=0.5;
    min_linear_speed=0.05;
    angle_adjust=0;
    current_linear_v=0.0;
    v=0.0;
    dx2=dy2=0.0;

    robot_state=STOP;
    last_robot_state=STOP;
    count=0;
    laserPauseSwitch=false;

    imu_data=imu_nh_.subscribe<std_msgs::Float32>("imu_angle",1,boost::bind(&RobotControl::getImuCallback,this,_1));
    move_cmd=nh_.subscribe<geometry_msgs::Twist>("move_fixed",1,boost::bind(&RobotControl::moveCmdCallback,this,_1));
    odom_data=imu_nh_.subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&RobotControl::getOdomCallback,this,_1));
    laser_Pause_switch=nh_.subscribe<std_msgs::Int16>("is_passed",1,boost::bind(&RobotControl::laserPauseCallback,this,_1));

    cmd_vel_pub=nh_.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);
    robot_state_pub=nh_.advertise<std_msgs::Int32>("/robot_state", 5);

    timer1 = nh_.createTimer(ros::Duration(0.1), boost::bind(&RobotControl::publishRobotState,this,_1));
    laserChangeRobotStateTimer2 = nh_.createTimer(ros::Duration(5), boost::bind(&RobotControl::laserChangeRobotState,this,_1));
    
    //ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    //spinner.start();
    //ros::waitForShutdown();
}

void RobotControl::laserChangeRobotState(const ros::TimerEvent&)
{

    if((robot_state==PAUSE)&&laserPauseSwitch==false)
    {
        cout<<"change robot state to : "<<last_robot_state<<endl;
        robot_state=last_robot_state;
    }
}

void RobotControl::laserPauseCallback(const std_msgs::Int16::ConstPtr &msg)
{
    //cout<<"in laserPauseCallback, "<<msg->data<<endl;
    if(msg->data>2)
    {
        cout<<"laser switch is true, start to pause "<<endl;
        laserPauseSwitch=true;
    }
    else
    {
        laserPauseSwitch=false;
    }
}

void RobotControl::publishRobotState(const ros::TimerEvent&)
{
    //cout<<"***robot state ="<<robot_state<<endl;
    std_msgs::Int32 st;
    st.data=robot_state;
    robot_state_pub.publish(st);
}

void RobotControl::getImuCallback(const std_msgs::Float32::ConstPtr& msg)
{
    current_robot_angel=msg->data;

    if(robot_state==ROTATION)
    {
        excute_rotate();
    }
}

void RobotControl::getOdomCallback(const nav_msgs::Odometry::ConstPtr& data)
{
    current_odom=*data;

    if(robot_state==LINEAR)
    {
        excute_linear();
    }
    else if(robot_state==LINEAR_ADJUST)
    {
        excute_linear_with_adjust();
    }

}


void RobotControl::moveCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ros::Rate r(10);
    cout<<"get move cmd"<<endl;
    if((fabs(msg->linear.x)<0.001)&&(fabs(msg->linear.y)<0.001)
      &&(fabs(msg->linear.z)<0.001)&&(fabs(msg->angular.x)<0.001)
      &&(fabs(msg->angular.y)<0.001)&&(fabs(msg->angular.z)<0.001)
      )
    {
        return;
    }
    //转动
    if((fabs(msg->angular.y)>0.001))
    {
        remain_turn_angle=msg->angular.y;
        target_angle=remain_turn_angle;
        count=0;
        robot_state=ROTATION;
    }
    while(ros::ok())
    {
        if(getRobotState() ==STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
    //走直线
    if((fabs(msg->linear.y)>0.001))
    {
        target_distance=msg->linear.y;
        remain_distance=msg->linear.y;
        count=0;
        robot_state=LINEAR;
    }
    while(ros::ok())
    {
        if(getRobotState() ==STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }

}

float RobotControl::real_turn_angle(float turn_angle)
{
    turn_angle=fmod(fmod(turn_angle, 2*M_PI)+ 2*M_PI, 2*M_PI);
    if(turn_angle>M_PI)
    {
        turn_angle =turn_angle-2*M_PI;
    }
    else if(turn_angle< ((-1.0)*M_PI))
    {
        turn_angle=turn_angle+2*M_PI;
    }
    else
    {
        turn_angle=turn_angle;
    }
    return turn_angle;

}

bool RobotControl::move_rotate(float angle)
{
    cout<<"in RobotControl move_rotate"<<endl;
    remain_turn_angle=angle;
    target_angle=remain_turn_angle;
    count=0;
    robot_state=ROTATION;
    return true;
}

bool RobotControl::move_linear(float linear_distance)
{
    cout<<"int RobotControl move_linear"<<endl;
    target_distance=linear_distance;
    remain_distance=linear_distance;
    count=0;
    robot_state=LINEAR;
}

void RobotControl::move_linear_with_adjust(float linear_distance,float angle)
{
    target_distance=linear_distance;
    remain_distance=linear_distance;
    angle_adjust=angle;
    count=0;
    robot_state=LINEAR_ADJUST;
}

void RobotControl::excute_rotate()
{
    if(count==0)
    {
        last_robot_angle=current_robot_angel;
        //cout<<"last_robot_angle= "<<last_robot_angle<<" current_robot_angel"<<current_robot_angel<<endl;
        count++;
    }

    if((fabs(remain_turn_angle)>angle_offset)&&(ros::ok()))
    {
        turn_angle=real_turn_angle(current_robot_angel-last_robot_angle);
        //cout<<"curren ="<<current_robot_angel<<" ,last="<<last_robot_angle<<endl;
        remain_turn_angle -= turn_angle;
       // cout<<"remain_turn_angle="<<remain_turn_angle<<endl;
        if(fabs(remain_turn_angle)>angle_threshold)
        {
            one_rotate_angle=(curren_rotate_speed*curren_rotate_speed)/(2.0*rotate_decel);
            if(one_rotate_angle >=(fabs(remain_turn_angle)-primary_threshold))
            {
                //cout<<"primary_threshold "<<primary_threshold<<endl;
                curren_rotate_speed -= rotate_decel*move_period;
                if(curren_rotate_speed<min_rotation_speed)
                {
                    curren_rotate_speed=min_rotation_speed;
                }
            }
            else //加速
            {
                if(curren_rotate_speed <max_rotation_speed)
                {
                    curren_rotate_speed += rotate_accel*move_period;
                }
                else
                {
                    curren_rotate_speed=max_rotation_speed;
                }
            }
            w=curren_rotate_speed * (remain_turn_angle/fabs(remain_turn_angle));
        }
        else //小于0.15弧度转动
        {
            if(fabs(remain_turn_angle)>angle_offset)
            {
                w=min_rotation_speed* (remain_turn_angle/fabs(remain_turn_angle));
            }
            else
            {
                curren_rotate_speed=0.0;
                w=0.0;
                remain_turn_angle=0.0;
            }
        }
       // cout<<"w= "<<w<<endl;
        robot_cmd_vel.linear.x  = 0.0;
        robot_cmd_vel.angular.z = w;
        cmd_vel_pub.publish(robot_cmd_vel);

        last_robot_angle=current_robot_angel;
    }
    else
    {
        //cout<<"robot state is stop"<<endl;
        robot_state=STOP;
    }

}

void RobotControl::excute_linear()
{
    if(count==0)
    {
        start_odom=current_odom;
        count++;
    }
    if((fabs(remain_distance)>distance_threshold)&&ros::ok())
    {
        dx2=pow(current_odom.pose.pose.position.x-start_odom.pose.pose.position.x,2);
        dy2=pow(current_odom.pose.pose.position.y-start_odom.pose.pose.position.y,2);
        run_distance=sqrt(dx2+dy2);
        remain_distance=fabs(target_distance)-run_distance;
        //cout<<"run_distance= "<<run_distance<<" ,remain_distance="<<remain_distance<<endl;
        if(remain_distance>distance_threshold)
        {   //线性加速
            current_linear_v += move_accel*move_period;
            //用距离差，pid 加减速，防止线性加速不合理
            if(remain_distance>max_linear_speed)
            {
                v=max_linear_speed;
            }
            else
            {
                v=remain_distance;
            }
            if(v<min_linear_speed)
            {
                v=min_linear_speed;
            }
            //取pid 与线性速度的最小值，为了更平滑
            if(v<current_linear_v)
            {
                current_linear_v=v;
            }
        }
        else
        {
            current_linear_v=0.0;

        }
        v=current_linear_v *(target_distance/fabs(target_distance));
       // cout<<"v= "<<v<<endl;
        robot_cmd_vel.linear.x  = v;
        robot_cmd_vel.angular.z = 0.0;

        //直线行走时，使用激光检测前方是否有障碍物
        if(laserPauseSwitch)
        {
            robot_cmd_vel.linear.x  = 0;
            //保存此时的状态，之后恢复使用
            last_robot_state=robot_state;
            current_linear_v=v=0.0;
            robot_state=PAUSE;
        }
        cmd_vel_pub.publish(robot_cmd_vel);
    }
    else
    {
        //cout<<"robot state is stop"<<endl;
        robot_state=STOP;
    }
}

void RobotControl::excute_linear_with_adjust()
{
    if(count==0)
    {
        start_odom=current_odom;
        count++;
    }
    if((fabs(remain_distance)>distance_threshold)&&ros::ok())
    {
        dx2=pow(current_odom.pose.pose.position.x-start_odom.pose.pose.position.x,2);
        dy2=pow(current_odom.pose.pose.position.y-start_odom.pose.pose.position.y,2);
        run_distance=sqrt(dx2+dy2);
        remain_distance=fabs(target_distance)-run_distance;
        //cout<<"run_distance= "<<run_distance<<" ,remain_distance="<<remain_distance<<endl;
        if(remain_distance>distance_threshold)
        {   //线性加速
            current_linear_v += move_accel*move_period;
            //用距离差，pid 加减速，防止线性加速不合理
            if(remain_distance>max_linear_speed)
            {
                v=max_linear_speed;
            }
            else
            {
                v=remain_distance;
            }
            if(v<min_linear_speed)
            {
                v=min_linear_speed;
            }
            //取pid 与线性速度的最小值，为了更平滑
            if(v<current_linear_v)
            {
                current_linear_v=v;
            }
        }
        else
        {
            current_linear_v=0.0;

        }
        v=current_linear_v *(target_distance/fabs(target_distance));
        //机器人角速度为正，逆时针转，角度增大;角速度为负，顺时针转，角度减小
        //此时方向偏左从0变到0.15， 0.05约3度
        if(real_turn_angle(current_robot_angel-angle_adjust)>=0.05)
        {//需要向右稍转调整,即需要逆时针转
            //cout<<"right adjust"<<endl;
            robot_cmd_vel.angular.z = -0.08;
        }//此时方向偏右，从0变到-0.15,  0.05约3度
        else if(real_turn_angle(current_robot_angel-angle_adjust)<=-0.05)
        {//需要向左稍转调整
            //cout<<"left adjust"<<endl;
            robot_cmd_vel.angular.z = 0.08;
        }
        else
        {//无需调整
            robot_cmd_vel.angular.z = 0.0;
        }

        robot_cmd_vel.linear.x  = v;
        cout<<"v= "<<v<<" ,w= "<<robot_cmd_vel.angular.z<<endl;
        //判断速度异常的处理
        if(fabs(robot_cmd_vel.linear.x)>1.5)
        {
            cout<<"v  too big or small ,is error"<<endl;
            robot_cmd_vel.linear.x=min_linear_speed;
            robot_state=STOP;
        }
        //直线行走时，使用激光检测前方是否有障碍物
        if(laserPauseSwitch)
        {
            robot_cmd_vel.linear.x  = 0;
            //保存此时的状态，之后恢复使用
            last_robot_state=robot_state;
            current_linear_v=v=0.0;
            robot_state=PAUSE;
        }

        cmd_vel_pub.publish(robot_cmd_vel);
    }
    else
    {
        //cout<<"robot state is stop"<<endl;
        robot_state=STOP;
    }
}

bool RobotControl::run_straight_line(geometry_msgs::Pose start_pose,geometry_msgs::Pose target_pose)
{
    //步骤1： 原地转到特定距离
    tf::Quaternion q;
    tf::quaternionMsgToTF(start_pose.orientation,q);
    float start_yaw=q.getAngle();
    cout<<"start_yaw= "<<start_yaw<<endl;
    float tran_angle =atan2(target_pose.position.y-start_pose.position.y ,
                            target_pose.position.x-start_pose.position.x);
    float turn_angle=real_turn_angle(tran_angle-start_yaw);
    cout<<"***1 turn angle= "<<turn_angle<<endl;
    move_rotate(turn_angle);
    while(ros::ok())
    {
        if(getRobotState() ==STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
    //步骤2：从起点走到终点
    float run_distance=sqrt(
                        pow(target_pose.position.x-start_pose.position.x,2)+
                        pow(target_pose.position.y-start_pose.position.y,2)
                        );
    cout<<"2 run distance = "<<run_distance<<endl;
    move_linear_with_adjust(run_distance,turn_angle);
    while(ros::ok())
    {
        if(getRobotState() ==STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
    //步骤3：到达目标点后，再原地转动特定角度
    tf::quaternionMsgToTF(target_pose.orientation,q);
    float target_yaw=q.getAngle();
    float turn_angle2=real_turn_angle(target_yaw-start_yaw-turn_angle);
    cout<<"***3 turn angle= "<<turn_angle2<<endl;
    move_rotate(turn_angle2);
    while(ros::ok())
    {
        if(getRobotState() ==STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
    return true;
}

int  RobotControl::getRobotState()
{
    return robot_state;
}

float RobotControl::getCurrentImuAngle()
{
    return current_robot_angel;
}

RobotControl::~RobotControl()
{

}

