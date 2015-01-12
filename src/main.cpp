#include <ros/ros.h>
#include <idyn_ros_interface.h>
#include <ros/package.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_state_publisher_ext");
    ros::NodeHandle n("~");

    std::string robot_name;
    n.getParam("robot_name", robot_name);
    if(robot_name.empty()){
        ROS_ERROR("robot_name param not provided!");
        return 0;}

    std::string urdf_path;
    n.getParam("urdf_path", urdf_path);
    if(urdf_path.empty()){
        ROS_ERROR("urdf_path param not provided!");
        return 0;}

    std::string srdf_path;
    n.getParam("srdf_path", srdf_path);
    if(srdf_path.empty()){
        ROS_ERROR("srdf_path param not provided!");
        return 0;}

    XmlRpc::XmlRpcValue ft_frames;
    n.param("ft_frames", ft_frames, ft_frames);
    if(ft_frames.getType() == XmlRpc::XmlRpcValue::TypeInvalid)
        ROS_WARN("NO ft_frames provided!");
    else
        ROS_INFO("Provided %i ft_frames", ft_frames.size());

    XmlRpc::XmlRpcValue ZMP_frames;
    n.param("ZMP_frames", ZMP_frames, ZMP_frames);
    if(ZMP_frames.getType() == XmlRpc::XmlRpcValue::TypeInvalid)
        ROS_WARN("NO ZMP_frames provided!");
    else
        ROS_INFO("Provided %i ZMP_frames", ZMP_frames.size());

    std::string tf_prefix;
    n.param("tf_prefix", tf_prefix, std::string(""));

    idyn_ros_interface robot(robot_name, urdf_path, srdf_path, tf_prefix, ft_frames, ZMP_frames);

    ROS_INFO("Starting Robot State Publisher Extended Node");

    double hz;
    n.param("/rate", hz, 50.0);
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
		ros::Time t = ros::Time::now();

        robot.publishWorld(t);
        robot.publishCoMtf(t);
        robot.publishConvexHull(t);
        robot.publishZMPs(t);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
