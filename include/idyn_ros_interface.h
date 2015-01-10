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
#include <geometry_msgs/WrenchStamped.h>
#include <numeric>

class idyn_ros_interface
{
public:
    class ft_sensor{
    public:
        ft_sensor(const unsigned int buffer_size = 10):
            _buffer_size(buffer_size)
        {}

        std::string ft_frame;
        std::string zmp_frame;
        yarp::sig::Vector zmp;
        ros::Subscriber subscribers;
        yarp::sig::Vector forces;
        yarp::sig::Vector torques;

    private:
        unsigned int _buffer_size;
        std::list<yarp::sig::Vector> _zmp_average;

    public: yarp::sig::Vector averageZMP(const yarp::sig::Vector& last_computed_zmp)
        {
            if(_buffer_size == 0)
                _buffer_size = 1;

            if(_zmp_average.size() == _buffer_size)
                _zmp_average.pop_front();
            _zmp_average.push_back(last_computed_zmp);

            yarp::sig::Vector zmp_av(3, 0.0);
            for(std::list<yarp::sig::Vector>::iterator it = _zmp_average.begin();
                it != _zmp_average.end(); it++){
                yarp::sig::Vector zmp_i = *it;
                zmp_av[0] += zmp_i[0];
                zmp_av[1] += zmp_i[1];}
            zmp_av[0] = zmp_av[0]/(double)_buffer_size;
            zmp_av[1] = zmp_av[1]/(double)_buffer_size;
            zmp_av[2] = zmp[2];
            return zmp_av;
        }

    public: yarp::sig::Vector getAverageZMP()
        {
            if(_buffer_size == 0)
                _buffer_size = 1;

            yarp::sig::Vector zmp_av(3, 0.0);
            for(std::list<yarp::sig::Vector>::iterator it = _zmp_average.begin();
                it != _zmp_average.end(); it++){
                yarp::sig::Vector zmp_i = *it;
                zmp_av[0] += zmp_i[0];
                zmp_av[1] += zmp_i[1];}
            zmp_av[0] = zmp_av[0]/(double)_buffer_size;
            zmp_av[1] = zmp_av[1]/(double)_buffer_size;
            zmp_av[2] = zmp[2];
            return zmp_av;
        }
    };

    iDynUtils robot;
    idynutils::convex_hull convex_hull;

    idyn_ros_interface(const std::string& robot_name,
                       const std::string& urdf_path,
                       const std::string& srdf_path,
                       const std::string& tf_prefix,
                       XmlRpc::XmlRpcValue& ft_frames,
                       XmlRpc::XmlRpcValue& ZMP_frames);
    ~idyn_ros_interface();

    void publishCoMtf(const ros::Time& t);
    void publishWorld(const ros::Time& t);
    void publishConvexHull(const ros::Time& t);
    void publishZMPs(const ros::Time& t);
private:
    ros::NodeHandle _n;
    ros::Subscriber _q_subs;
    tf::TransformBroadcaster _br;
    tf::TransformListener _lr;
    ros::Publisher _vis_pub;

    std::string _tf_prefix;

    yarp::sig::Vector _q;

    std::vector<ft_sensor> _ft_sensors;


    std::string reference_frame_CoM;
    
    void updateIdynCallBack(const sensor_msgs::JointState &msg);
    void updateFromFTSensor(const geometry_msgs::WrenchStamped &msg);
    void fillKinematicChainConfig(const kinematic_chain& kc,
                                  std::map<std::string, double>& joint_names_values);
};

#endif
