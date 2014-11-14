#ifndef _IDYN_ROS_INTERFACE_H_
#define _IDYN_ROS_INTERFACE_H_

#include <ros/ros.h>
#include <idynutils/idynutils.h>
#include <yarp/sig/all.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <idynutils/convex_hull.h>
#include <visualization_msgs/Marker.h>
#include <kdl/frames.hpp>

class idyn_ros_interface
{
public:
    iDynUtils robot;
    idynutils::convex_hull convex_hull;

    idyn_ros_interface();
    ~idyn_ros_interface();

    void publishCoMtf();
    //void publishBaseFootPrint();
    void publishWorld();
    void publishConvexHull();
private:
    ros::NodeHandle _n;
    ros::Subscriber _q_subs;
    tf::TransformBroadcaster _br;
    tf::TransformListener _lr;
    ros::Publisher _vis_pub;


    yarp::sig::Vector _q;

    std::string reference_frame_CoM;
    std::string reference_frame_base_foot_print;

    void updateIdynCallBack(const sensor_msgs::JointState &msg);
    void fillKinematicChainConfig(const kinematic_chain& kc,
                                  std::map<std::string, double>& joint_names_values);


};

#endif
