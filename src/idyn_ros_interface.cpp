#include <idyn_ros_interface.h>


using namespace iCub::iDynTree;

idyn_ros_interface::idyn_ros_interface(const std::string &robot_name,
                                       const std::string &urdf_path,
                                       const std::string &srdf_path,
                                       const std::string &tf_prefix,
                                       XmlRpc::XmlRpcValue& ft_frames,
                                       XmlRpc::XmlRpcValue& ZMP_frames):
    robot(robot_name, urdf_path, srdf_path),
    _n(),
    _q_subs(),
    _br(),
    _lr(),
    _q(robot.iDyn3_model.getNrOfDOFs(), 0.0),
    reference_frame_CoM("world"),
    _tf_prefix(tf_prefix)
{
    _q_subs = _n.subscribe("/joint_states", 100, &idyn_ros_interface::updateIdynCallBack, this);


    std::string marker_viz_name = tf::resolve(_tf_prefix, "robot_state_publisher_ext_viz");
    _vis_pub = _n.advertise<visualization_msgs::Marker>( marker_viz_name, 0 );

    robot.updateiDyn3Model(_q, _q, _q, true);

    if(!(ft_frames.getType() == XmlRpc::XmlRpcValue::TypeInvalid)){
        for(unsigned int i = 0; i < ft_frames.size(); ++i){
            _ft_frames.push_back(ft_frames[i]);
            _ft_vals.push_back(yarp::sig::Vector(6, 0.0));
            ROS_ERROR("%s", _ft_frames[i].c_str());
            _ft_subscribers.push_back(_n.subscribe("/" + _ft_frames[i] +"/ft_sensor", 100,
                                                   &idyn_ros_interface::updateFromFTSensor, this));}
    }

    if(!(ZMP_frames.getType() == XmlRpc::XmlRpcValue::TypeInvalid)){
        for(unsigned int i = 0; i < ZMP_frames.size(); ++i){
            _ZMP_frames.push_back(ZMP_frames[i]);
            _ZMPs.push_back(yarp::sig::Vector(3, 0.0));}
    }


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

void idyn_ros_interface::updateFromFTSensor(const geometry_msgs::WrenchStamped &msg)
{
    int ft_index = distance(_ft_frames.begin(),find(_ft_frames.begin(), _ft_frames.end(), msg.header.frame_id));

    _ft_vals[ft_index][0] = msg.wrench.force.x;
    _ft_vals[ft_index][1] = msg.wrench.force.y;
    _ft_vals[ft_index][2] = msg.wrench.force.z;
    _ft_vals[ft_index][3] = msg.wrench.torque.x;
    _ft_vals[ft_index][4] = msg.wrench.torque.y;
    _ft_vals[ft_index][5] = msg.wrench.torque.z;

    yarp::sig::Matrix sensor_to_sole = robot.iDyn3_model.getPosition(
                    robot.iDyn3_model.getLinkIndex(_ft_frames[ft_index]),
                    robot.iDyn3_model.getLinkIndex(_ZMP_frames[ft_index])
                );

    double d = fabs(sensor_to_sole(2,3));

    _ZMPs[ft_index] = cartesian_utils::computeFootZMP(_ft_vals[ft_index].subVector(0,2),
                                                      _ft_vals[ft_index].subVector(3,5), d, 1.0);
}

void idyn_ros_interface::publishZMPs(const ros::Time& t)
{
    visualization_msgs::Marker ZMP_marker;

    for(unsigned int i = 0; i < _ZMPs.size(); ++i)
    {
        ZMP_marker.header.frame_id = _ft_frames[i];
        ZMP_marker.header.stamp = t;
        ZMP_marker.ns = tf::resolve(_tf_prefix, "ZMP"+_ft_frames[i]);
        ZMP_marker.id = 2+i;
        ZMP_marker.type = visualization_msgs::Marker::SPHERE;
        ZMP_marker.action = visualization_msgs::Marker::ADD;

        ZMP_marker.pose.orientation.x = 0.0;
        ZMP_marker.pose.orientation.y = 0.0;
        ZMP_marker.pose.orientation.z = 0.0;
        ZMP_marker.pose.orientation.w = 1.0;
        ZMP_marker.pose.position.x = _ZMPs[i][0];
        ZMP_marker.pose.position.y = _ZMPs[i][1];
        ZMP_marker.pose.position.z = _ZMPs[i][2];

        ZMP_marker.color.a = 1.0;
        ZMP_marker.color.r = 0.0;
        ZMP_marker.color.g = 1.0;
        ZMP_marker.color.b = 1.0;

        ZMP_marker.scale.x = 0.015;
        ZMP_marker.scale.y = 0.015;
        ZMP_marker.scale.z = 0.015;
        _vis_pub.publish(ZMP_marker);
    }

    yarp::sig::Vector ZMP = cartesian_utils::computeZMP(_ft_vals[0].subVector(0,2), _ft_vals[0].subVector(3,5), _ZMPs[0],
                                       _ft_vals[1].subVector(0,2), _ft_vals[1].subVector(3,5), _ZMPs[1], 1.0);

    ///TODO: Transform ZMP in world frame!

    ZMP_marker.header.frame_id = "world";
    ZMP_marker.header.stamp = t;
    ZMP_marker.ns = tf::resolve(_tf_prefix, "ZMP");
    ZMP_marker.id = 2+_ZMPs.size()+1;
    ZMP_marker.type = visualization_msgs::Marker::SPHERE;
    ZMP_marker.action = visualization_msgs::Marker::ADD;

    ZMP_marker.pose.orientation.x = 0.0;
    ZMP_marker.pose.orientation.y = 0.0;
    ZMP_marker.pose.orientation.z = 0.0;
    ZMP_marker.pose.orientation.w = 1.0;
    ZMP_marker.pose.position.x = ZMP[0];
    ZMP_marker.pose.position.y = ZMP[1];
    ZMP_marker.pose.position.z = ZMP[2];

    ZMP_marker.color.a = 1.0;
    ZMP_marker.color.r = 0.0;
    ZMP_marker.color.g = 1.0;
    ZMP_marker.color.b = 1.0;

    ZMP_marker.scale.x = 0.015;
    ZMP_marker.scale.y = 0.015;
    ZMP_marker.scale.z = 0.015;
    _vis_pub.publish(ZMP_marker);
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

    std::string frame_id = tf::resolve(_tf_prefix, reference_frame_CoM);
    std::string child_frame_id = tf::resolve(_tf_prefix, "CoM");
    _br.sendTransform(tf::StampedTransform(CoM_transform, t, frame_id, child_frame_id));

    visualization_msgs::Marker com_projected_marker;

    com_projected_marker.header.frame_id = frame_id;
    com_projected_marker.header.stamp = t;
    com_projected_marker.ns = tf::resolve(_tf_prefix, "com_projected");
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

    std::string frame_id = tf::resolve(_tf_prefix, "world");
    std::string child_frame_id = tf::resolve(_tf_prefix, "base_link");
    _br.sendTransform(tf::StampedTransform(world_T_base_link_tf, t, frame_id, child_frame_id));
}

void idyn_ros_interface::publishConvexHull(const ros::Time& t)
{
    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;
    idynutils::convex_hull::getSupportPolygonPoints(robot,points);
    if(convex_hull.getConvexHull(points,ch)) {
        yarp::sig::Vector CoM( robot.iDyn3_model.getCOM());
        visualization_msgs::Marker ch_marker;

        std::string frame_id = tf::resolve(_tf_prefix, "CoM");

        ch_marker.header.frame_id = frame_id;
        ch_marker.header.stamp = t;
        ch_marker.ns = tf::resolve(_tf_prefix, "convex_hull");
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
