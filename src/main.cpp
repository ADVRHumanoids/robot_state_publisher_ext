#include <ros/ros.h>
#include <idyn_ros_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_state_publisher_ext");
    ros::NodeHandle n;

    idyn_ros_interface coman;

    ROS_INFO("Starting Robot State Publisher Extended Node");

    double hz = 50.0;
    n.param("rate", hz, 50.0);
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        coman.publishCoMtf();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
