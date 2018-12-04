#include "eai_robot_control/eai_robot_control.h"


geometry_msgs::Pose start_pose,target_pose;

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
    rc.run_straight_line(start_pose,target_pose);
    //rc.move_linear(1.0);
    //loop.sleep();
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();
    ros::waitForShutdown();
    //ros::spin();
    //ros::spinOnce();
    return 0;
}
