#ifndef _IDYN_ROS_INTERFACE_H_
#define _IDYN_ROS_INTERFACE_H_

#include <ros/ros.h>
#include <idynutils/idynutils.h>
#include <yarp/sig/all.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <idynutils/cartesian_utils.h>
#include <idynutils/convex_hull.h>
#include <visualization_msgs/Marker.h>
#include <kdl/frames.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <numeric>
#include <robot_state_publisher_ext/StringArray.h>
#include <sensor_msgs/Imu.h>
#include <boost/utility.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

using namespace yarp::math;

yarp::sig::Vector operator/(const yarp::sig::Vector& a, const long unsigned int b)
{
    return a*(1.0/double(b));
}

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
        std::list<yarp::sig::Vector> forces_window;
        std::list<yarp::sig::Vector> torques_window;

    private:
        unsigned int _buffer_size;

    public: yarp::sig::Vector averageVal(const yarp::sig::Vector& new_val, std::list<yarp::sig::Vector>& window)
        {
            if(_buffer_size == 0)
                _buffer_size = 1;

            if(window.size() == _buffer_size)
                window.pop_front();
            window.push_back(new_val);

            yarp::sig::Vector average(3, 0.0);
            for(std::list<yarp::sig::Vector>::iterator it = window.begin(); it != window.end(); it++){
                yarp::sig::Vector val_i = *it;
                average[0] += val_i[0];
                average[1] += val_i[1];
                average[2] += val_i[2];}
            average[0] = average[0]/(double)_buffer_size;
            average[1] = average[1]/(double)_buffer_size;
            average[2] = average[2]/(double)_buffer_size;
            return average;
        }

    public: yarp::sig::Vector getAveragedVal(const std::list<yarp::sig::Vector>& window)
        {
            if(_buffer_size == 0)
                _buffer_size = 1;

            yarp::sig::Vector average(3, 0.0);
            for(std::list<yarp::sig::Vector>::const_iterator it = window.begin(); it != window.end(); it++){
                yarp::sig::Vector val_i = *it;
                average[0] += val_i[0];
                average[1] += val_i[1];
                average[2] += val_i[2];}
            average[0] = average[0]/(double)_buffer_size;
            average[1] = average[1]/(double)_buffer_size;
            average[2] = average[2]/(double)_buffer_size;
            return average;
        }
    };

    iDynUtils robot;
    idynutils::convex_hull convex_hull;

    idyn_ros_interface(const std::string& robot_name,
                       const std::string& urdf_path,
                       const std::string& srdf_path,
                       const std::string& tf_prefix,
                       XmlRpc::XmlRpcValue& ft_frames,
                       XmlRpc::XmlRpcValue& ZMP_frames,
                       const double dT);
    ~idyn_ros_interface();

    void publishCoMtf(const ros::Time& t);
    void publishWorld(const ros::Time& t);
    void publishConvexHull(const ros::Time& t);
    void publishZMPs(const ros::Time& t);
    void publishCP(const ros::Time& t);
private:
    ros::NodeHandle _n;
    ros::Subscriber _q_subs;
    double _dT;

    ros::Subscriber _imu_subs;

    tf::TransformBroadcaster _br;
    tf::TransformListener _lr;
    ros::Publisher _vis_pub;

    std::string _tf_prefix;

    yarp::sig::Vector _q;
    yarp::sig::Vector _q_dot;

    std::vector<ft_sensor> _ft_sensors;


    std::string reference_frame_CoM;

    ros::Subscriber _world_subs;

    std::list<std::string> _links_in_contact;
    ros::Subscriber _links_in_contact_subs;

    boost::accumulators::accumulator_set< yarp::sig::Vector, boost::accumulators::stats<
        boost::accumulators::tag::rolling_mean > > _floating_base_velocity_in_world;


    void updateIMUCallBack(const sensor_msgs::Imu &msg);
    void updateIdynCallBack(const sensor_msgs::JointState &msg);
    void updateNewWorld(const geometry_msgs::TransformStamped &msg);
    void updateLinksInContactCallBack(const robot_state_publisher_ext::StringArray &msg);
    void updateFromFTSensor(const geometry_msgs::WrenchStamped &msg);
    void fillKinematicChainConfig(const kinematic_chain& kc,
                                  std::map<std::string, double>& joint_names_values);
    void fillKinematicChainConfig(const kinematic_chain& kc,
                                  std::map<std::string, double>& joint_names_values,
                                  std::map<std::string, double>& joint_names_vel);
};

#endif
