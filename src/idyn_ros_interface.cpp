#include <idyn_ros_interface.h>
#include <kdl/frames.hpp>

using namespace iCub::iDynTree;

idyn_ros_interface::idyn_ros_interface():
    robot(),
    _n(),
    _q_subs(),
    _br(),
    _lr(),
    _q(robot.iDyn3_model.getNrOfDOFs(), 0.0),
    reference_frame_CoM("world"),
    reference_frame_base_foot_print("CoM")
{
    _q_subs = _n.subscribe("/joint_states", 100, &idyn_ros_interface::updateIdynCallBack, this);

    robot.updateiDyn3Model(_q, _q, _q, true);
}

idyn_ros_interface::~idyn_ros_interface()
{

}

void idyn_ros_interface::updateIdynCallBack(const sensor_msgs::JointState &msg)
{
    std::map<std::string, double> joint_name_value_map;
    for(unsigned int i = 0; i < msg.name.size(); ++i)
        joint_name_value_map[msg.name[i]] = msg.position[i];

    fillKinematicChainConfig(robot.left_arm, joint_name_value_map);
    fillKinematicChainConfig(robot.left_leg, joint_name_value_map);
    fillKinematicChainConfig(robot.right_arm, joint_name_value_map);
    fillKinematicChainConfig(robot.right_leg, joint_name_value_map);
    fillKinematicChainConfig(robot.torso, joint_name_value_map);

    yarp::sig::Vector foo(_q.size(), 0.0);
    robot.updateiDyn3Model(_q, foo, foo, true);
}

void idyn_ros_interface::fillKinematicChainConfig(const kinematic_chain &kc,
                                                  std::map<std::string, double> &joint_names_values)
{
    for(unsigned int i = 0; i < kc.joint_names.size(); ++i)
        _q[kc.joint_numbers[i]] = joint_names_values[kc.joint_names[i]];
}

void idyn_ros_interface::publishCoMtf()
{
    yarp::sig::Vector CoM( robot.iDyn3_model.getCOM());
                           //idynutils.iDyn3_model.getLinkIndex(reference_frame_CoM)) );

    tf::Transform CoM_transform;
    CoM_transform.setIdentity();
    CoM_transform.setOrigin(tf::Vector3(CoM[0], CoM[1], CoM[2]));

    _br.sendTransform(tf::StampedTransform(CoM_transform, ros::Time::now(),
                                           reference_frame_CoM, "CoM"));

}

void idyn_ros_interface::publishWorld()
{
    KDL::Frame world_T_base_link =  robot.iDyn3_model.getWorldBasePoseKDL();

    double qx, qy, qz, qw;
    world_T_base_link.M.GetQuaternion(qx, qy, qz, qw);

    tf::Transform world_T_base_link_tf;
    world_T_base_link_tf.setOrigin(tf::Vector3(world_T_base_link.p.x(), world_T_base_link.p.y(), world_T_base_link.p.z()));
    world_T_base_link_tf.setRotation(tf::Quaternion(qx, qy, qz, qw));

    _br.sendTransform(tf::StampedTransform(world_T_base_link_tf, ros::Time::now(),
                                           "world", "base_link"));
}

void idyn_ros_interface::publishConvexHull()
{
    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;
    idynutils::convex_hull::getSupportPolygonPoints(robot,points);
    if(convex_hull.getConvexHull(points,ch)) {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "CoM";
        marker.header.stamp = ros::Time::now();
        marker.ns = "robot_state_publisher_ext/convex_hull";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point p;
        for(std::list<KDL::Vector>::iterator i = points.begin(); i != points.end(); ++i)
        {
            p.x = i->x();
            p.y = i->y();
            p.z = i->z();
            marker.points.push_back(p);
        }
        p.x = points.begin()->x();
        p.y = points.begin()->y();
        p.z = points.begin()->z();
        marker.points.push_back(p);

        marker.scale.x = 1;

        //needs publisher
    }
}
