#include <idyn_ros_interface.h>


using namespace iCub::iDynTree;

idyn_ros_interface::idyn_ros_interface(const std::string &robot_name,
                                       const std::string &urdf_path,
                                       const std::string &srdf_path):
    robot(robot_name, urdf_path, srdf_path),
    _n(),
    _q_subs(),
    _br(),
    _lr(),
    _q(robot.iDyn3_model.getNrOfDOFs(), 0.0),
    reference_frame_CoM("world")
{
    _q_subs = _n.subscribe("/joint_states", 100, &idyn_ros_interface::updateIdynCallBack, this);
    _vis_pub = _n.advertise<visualization_msgs::Marker>( "robot_state_publisher_ext_viz", 0 );

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

void idyn_ros_interface::publishCoMtf(const ros::Time &t)
{
    yarp::sig::Vector CoM( robot.iDyn3_model.getCOM());
                           //idynutils.iDyn3_model.getLinkIndex(reference_frame_CoM)) );

    tf::Transform CoM_transform;
    CoM_transform.setIdentity();
    CoM_transform.setOrigin(tf::Vector3(CoM[0], CoM[1], CoM[2]));

    _br.sendTransform(tf::StampedTransform(CoM_transform, t,
                                           reference_frame_CoM, "CoM"));

    visualization_msgs::Marker com_projected_marker;

    com_projected_marker.header.frame_id = reference_frame_CoM;
    com_projected_marker.header.stamp = t;
    com_projected_marker.ns = "com_projected";
    com_projected_marker.id = 1;
    com_projected_marker.type = visualization_msgs::Marker::SPHERE;
    com_projected_marker.action = visualization_msgs::Marker::ADD;

    com_projected_marker.pose.orientation.x = 0.0;
    com_projected_marker.pose.orientation.y = 0.0;
    com_projected_marker.pose.orientation.z = 0.0;
    com_projected_marker.pose.orientation.w = 1.0;
    com_projected_marker.pose.position.x = CoM[0];
    com_projected_marker.pose.position.y = CoM[1];
    com_projected_marker.pose.position.z = 0.0;

    com_projected_marker.color.a = 1.0;
    com_projected_marker.color.r = 1.0;
    com_projected_marker.color.g = 0.0;
    com_projected_marker.color.b = 1.0;

    com_projected_marker.scale.x = 0.015;
    com_projected_marker.scale.y = 0.015;
    com_projected_marker.scale.z = 0.015;
    _vis_pub.publish(com_projected_marker);
}

void idyn_ros_interface::publishWorld(const ros::Time &t)
{
    KDL::Frame world_T_base_link =  robot.iDyn3_model.getWorldBasePoseKDL();

    double qx, qy, qz, qw;
    world_T_base_link.M.GetQuaternion(qx, qy, qz, qw);

    tf::Transform world_T_base_link_tf;
    world_T_base_link_tf.setOrigin(tf::Vector3(world_T_base_link.p.x(), world_T_base_link.p.y(), world_T_base_link.p.z()));
    world_T_base_link_tf.setRotation(tf::Quaternion(qx, qy, qz, qw));

    _br.sendTransform(tf::StampedTransform(world_T_base_link_tf, t,
                                           "world", "base_link"));
}

void idyn_ros_interface::publishConvexHull(const ros::Time& t)
{
    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;
    idynutils::convex_hull::getSupportPolygonPoints(robot,points);
    if(convex_hull.getConvexHull(points,ch)) {
        yarp::sig::Vector CoM( robot.iDyn3_model.getCOM());
        visualization_msgs::Marker ch_marker;

        ch_marker.header.frame_id = "CoM";
        ch_marker.header.stamp = t;
        ch_marker.ns = "convex_hull";
        ch_marker.id = 0;
        ch_marker.type = visualization_msgs::Marker::LINE_STRIP;
        ch_marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point p;
        for(std::vector<KDL::Vector>::iterator i =ch.begin(); i != ch.end(); ++i)
        {
            p.x = i->x();
            p.y = i->y();
            p.z = i->z()-CoM[2];
            ch_marker.points.push_back(p);
        }
        p.x = ch.begin()->x();
        p.y = ch.begin()->y();
        p.z = ch.begin()->z()-CoM[2];
        ch_marker.points.push_back(p);

        ch_marker.color.a = 1.0;
        ch_marker.color.r = 0.0;
        ch_marker.color.g = 1.0;
        ch_marker.color.b = 0.0;

        ch_marker.scale.x = 0.01;

        _vis_pub.publish(ch_marker);
    }
}
