#ifndef _IDYN_ROS_INTERFACE_H_
#define _IDYN_ROS_INTERFACE_H_

#include <ros/ros.h>
#include <drc_shared/idynutils.h>
#include <yarp/sig/all.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class idyn_ros_interface
{
public:
    iDynUtils idynutils;

    idyn_ros_interface();
    ~idyn_ros_interface();

    void publishCoMtf(const ros::Time& t);
    //void publishBaseFootPrint();
    void publishWorld(const ros::Time& t);
private:
    ros::NodeHandle _n;
    ros::Subscriber _q_subs;
    tf::TransformBroadcaster _br;
    tf::TransformListener _lr;

    yarp::sig::Vector _q;

    std::string reference_frame_CoM;
    std::string reference_frame_base_foot_print;

    void updateIdynCallBack(const sensor_msgs::JointState &msg);
    void fillKinematicChainConfig(const kinematic_chain& kc,
                                  std::map<std::string, double>& joint_names_values);
};

#endif
