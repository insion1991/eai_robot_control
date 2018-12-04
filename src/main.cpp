#include "eai_robot_control/eai_robot_control.h"


geometry_msgs::Pose start_pose,target_pose;


bool run_straight_line(geometry_msgs::Pose start_pose,geometry_msgs::Pose target_pose)
{
    RobotControl rc;
    tf::Quaternion q;
    tf::quaternionMsgToTF(start_pose.orientation,q);
    float start_yaw=q.getAngle();
    cout<<"start_yaw= "<<start_yaw<<endl;
    float tran_angle =atan2(target_pose.position.y-start_pose.position.y ,
                            target_pose.position.x-start_pose.position.x);
    float turn_angle=tran_angle-start_yaw;
    cout<<"***1 turn angle= "<<turn_angle<<endl;
    rc.move_rotate(turn_angle);
    while(ros::ok())
    {
        if(rc.getRobotState() ==RobotControl::STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
    float run_distance=sqrt(
                        pow(target_pose.position.x-start_pose.position.x,2)+
                        pow(target_pose.position.y-start_pose.position.y,2)
                        );
    cout<<"2 run distance = "<<run_distance<<endl;
    rc.move_linear_with_adjust(run_distance,turn_angle);
    while(ros::ok())
    {
        if(rc.getRobotState() ==RobotControl::STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
    tf::quaternionMsgToTF(target_pose.orientation,q);
    float target_yaw=q.getAngle();
    float turn_angle2=target_yaw-start_yaw-turn_angle;
    cout<<"***3 turn angle= "<<turn_angle2<<endl;
    rc.move_rotate(turn_angle2);
    while(ros::ok())
    {
        if(rc.getRobotState() ==RobotControl::STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
    return true;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"eai_robot_control");
    RobotControl rc;

 #if 0  //原地转动
    rc.move_rotate(3.14);
    while(ros::ok())
    {
        if(rc.getRobotState() ==RobotControl::STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
#endif

#if 0  //不带校准的直线行走
    rc.move_linear(10.0);
    while(ros::ok())
    {
        if(rc.getRobotState() ==RobotControl::STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
#endif

#if 0  //带校准的直线行走
    rc.move_linear_with_adjust(1.0,0.0);
    while(ros::ok())
    {
        if(rc.getRobotState() ==RobotControl::STOP)
        {
            cout<<"excute complate"<<endl;
            break;
        }
        ros::spinOnce();
    }
#endif

    //两点之间走直线
    start_pose.position.x=0.0;
    start_pose.position.y=0.0;
    start_pose.position.z=0.0;
    start_pose.orientation=tf::createQuaternionMsgFromYaw(0);

    target_pose.position.x=1.0;
    target_pose.position.y=0.0;
    target_pose.position.z=0.0;
    target_pose.orientation=tf::createQuaternionMsgFromYaw(0);

    //run_straight_line(start_pose,target_pose);
    //rc.run_straight_line(start_pose,target_pose);
    //rc.move_linear(1.0);
    //loop.sleep();
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();
    ros::waitForShutdown();
    //ros::spin();
    //ros::spinOnce();
    return 0;
}
