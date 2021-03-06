#include <idyn_ros_interface.h>
#include <tf/transform_datatypes.h>
#include <kdl_conversions/kdl_msg.h>


#define FT_SWITCHING_TH 40.0

using namespace iCub::iDynTree;

idyn_ros_interface::idyn_ros_interface(const std::string &robot_name,
                                       const std::string &urdf_path,
                                       const std::string &srdf_path,
                                       const std::string &tf_prefix,
                                       XmlRpc::XmlRpcValue& ft_frames,
                                       XmlRpc::XmlRpcValue& ZMP_frames,
                                       const double dT):
    robot(robot_name, urdf_path, srdf_path),
    _n(),
    _q_subs(),
    _dT(dT),
    _imu_subs(),
    _br(),
    _lr(),
    _q(robot.iDyn3_model.getNrOfDOFs(), 0.0),
    _q_dot(_q.size(), 0.0),
    reference_frame_CoM("world"),
    _tf_prefix(tf_prefix),
    _floating_base_velocity_in_world(yarp::sig::Vector(3, 0.0), boost::accumulators::tag::rolling_mean::window_size = 10)
{
    _q_subs = _n.subscribe("/joint_states", 100, &idyn_ros_interface::updateIdynCallBack, this);
    _imu_subs = _n.subscribe("/imu", 100, &idyn_ros_interface::updateIMUCallBack, this);
    _world_subs = _n.subscribe("/anchor_to_world_pose", 100, &idyn_ros_interface::updateNewWorld, this);
    _links_in_contact_subs = _n.subscribe("/links_in_contact", 100, &idyn_ros_interface::updateLinksInContactCallBack, this);


    std::string marker_viz_name = tf::resolve(_tf_prefix, "robot_state_publisher_ext_viz");
    _vis_pub = _n.advertise<visualization_msgs::Marker>( marker_viz_name, 0 );

    if(!(ft_frames.getType() == XmlRpc::XmlRpcValue::TypeInvalid)){
        ft_sensor ft(10);
        for(unsigned int i = 0; i < ft_frames.size(); ++i){
            ft.ft_frame = std::string(ft_frames[i]);
            ft.forces = yarp::sig::Vector(3, 0.0);
            ft.torques = yarp::sig::Vector(3, 0.0);
            ft.subscribers = _n.subscribe("/" + ft.ft_frame +"/ft_sensor", 100,
                                                   &idyn_ros_interface::updateFromFTSensor, this);
            if(!(ZMP_frames.getType() == XmlRpc::XmlRpcValue::TypeInvalid))
                ft.zmp_frame = std::string(ZMP_frames[i]);
            else
                ft.zmp_frame = std::string(ft_frames[i]);
            ft.zmp = yarp::sig::Vector(3, 0.0);

            _ft_sensors.push_back(ft);
        }
    }

    // We start considering both the feet in the ground
    _links_in_contact.push_back("l_foot_lower_left_link");
    _links_in_contact.push_back("l_foot_lower_right_link");
    _links_in_contact.push_back("l_foot_upper_left_link");
    _links_in_contact.push_back("l_foot_upper_right_link");
    _links_in_contact.push_back("r_foot_lower_left_link");
    _links_in_contact.push_back("r_foot_lower_right_link");
    _links_in_contact.push_back("r_foot_upper_left_link");
    _links_in_contact.push_back("r_foot_upper_right_link");

    robot.updateiDyn3Model(_q, _q_dot, _q, true);
}

idyn_ros_interface::~idyn_ros_interface()
{

}

void idyn_ros_interface::updateIdynCallBack(const sensor_msgs::JointState &msg)
{
    std::map<std::string, double> joint_name_value_map;
    std::map<std::string, double> joint_name_vel_map;
    for(unsigned int i = 0; i < msg.name.size(); ++i){
        joint_name_value_map[msg.name[i]] = msg.position[i];
        joint_name_vel_map[msg.name[i]] = msg.velocity[i];}

    fillKinematicChainConfig(robot.left_arm, joint_name_value_map, joint_name_vel_map);
    fillKinematicChainConfig(robot.left_leg, joint_name_value_map, joint_name_vel_map);
    fillKinematicChainConfig(robot.right_arm, joint_name_value_map, joint_name_vel_map);
    fillKinematicChainConfig(robot.right_leg, joint_name_value_map, joint_name_vel_map);
    fillKinematicChainConfig(robot.torso, joint_name_value_map, joint_name_vel_map);
    fillKinematicChainConfig(robot.head, joint_name_value_map, joint_name_vel_map);

    yarp::sig::Vector foo(_q.size(), 0.0);
    robot.updateiDyn3Model(_q, _q_dot, foo, true);
}

void idyn_ros_interface::updateNewWorld(const geometry_msgs::TransformStamped &msg)
{
    if(robot.switchAnchor(msg.header.frame_id) && msg.child_frame_id == "world"){
        KDL::Frame anchor_T_world;
        tf::transformMsgToKDL(msg.transform, anchor_T_world);
        robot.setAnchor_T_World(anchor_T_world);
        ROS_WARN("Set anchor in link %s", msg.header.frame_id.c_str());
        ROS_WARN("Set Transformation from anchor to world as:");
        cartesian_utils::printKDLFrame(anchor_T_world);
    }
    else
        ROS_ERROR("WRONG ANCHOR FRAME!");
}

void idyn_ros_interface::updateFromFTSensor(const geometry_msgs::WrenchStamped &msg)
{
    if(!_ft_sensors.empty())
    {
        for(unsigned int i = 0; i < _ft_sensors.size(); ++i)
        {
            if(_ft_sensors[i].ft_frame == msg.header.frame_id)
            {
                _ft_sensors[i].forces[0] = msg.wrench.force.x;
                _ft_sensors[i].forces[1] = msg.wrench.force.y;
                _ft_sensors[i].forces[2] = msg.wrench.force.z;
                _ft_sensors[i].torques[0] = msg.wrench.torque.x;
                _ft_sensors[i].torques[1] = msg.wrench.torque.y;
                _ft_sensors[i].torques[2] = msg.wrench.torque.z;

                _ft_sensors[i].averageVal(_ft_sensors[i].forces, _ft_sensors[i].forces_window);
                _ft_sensors[i].averageVal(_ft_sensors[i].torques, _ft_sensors[i].torques_window);
            }
        }

        //Update model according to the last reads
        yarp::sig::Vector foo(_q.size(), 0.0);
        robot.updateiDyn3Model(_q, _q_dot, foo, true);

        for(unsigned int i = 0; i < _ft_sensors.size(); ++i)
        {
            yarp::sig::Matrix sensor_to_sole = robot.iDyn3_model.getPosition(
                            robot.iDyn3_model.getLinkIndex(_ft_sensors[i].ft_frame),
                            robot.iDyn3_model.getLinkIndex(_ft_sensors[i].zmp_frame));
            double d = fabs(sensor_to_sole(2,3));

            _ft_sensors[i].zmp = cartesian_utils::computeFootZMP(
                        _ft_sensors[i].getAveragedVal(_ft_sensors[i].forces_window),
                        _ft_sensors[i].getAveragedVal(_ft_sensors[i].torques_window), d, FT_SWITCHING_TH);
        }
    }
}

//Note: CP marker is RED
void idyn_ros_interface::publishCP(const ros::Time &t)
{

    visualization_msgs::Marker CP_marker;

    std::string child_frame_id = "";

    yarp::sig::Vector com_velocity = robot.iDyn3_model.getVelCOM();
    yarp::sig::Vector com_pose = robot.iDyn3_model.getCOM();

    yarp::sig::Vector CP = cartesian_utils::computeCapturePoint(
                boost::accumulators::rolling_mean(_floating_base_velocity_in_world), com_velocity, com_pose);

    child_frame_id = tf::resolve(_tf_prefix, "world");

    CP_marker.header.frame_id = child_frame_id;
    CP_marker.header.stamp = t;
    CP_marker.ns = "CP";
    CP_marker.id = 3+_ft_sensors.size();
    CP_marker.type = visualization_msgs::Marker::SPHERE;
    CP_marker.action = visualization_msgs::Marker::ADD;

    CP_marker.pose.orientation.x = 0.0;
    CP_marker.pose.orientation.y = 0.0;
    CP_marker.pose.orientation.z = 0.0;
    CP_marker.pose.orientation.w = 1.0;
    CP_marker.pose.position.x = CP[0];
    CP_marker.pose.position.y = CP[1];
    CP_marker.pose.position.z = CP[2];

    CP_marker.color.a = 1.0;
    CP_marker.color.r = 1.0;
    CP_marker.color.g = 0.0;
    CP_marker.color.b = 0.0;

    CP_marker.scale.x = 0.015;
    CP_marker.scale.y = 0.015;
    CP_marker.scale.z = 0.015;
    _vis_pub.publish(CP_marker);
}

//Note: ZMP Marker is Azure
void idyn_ros_interface::publishZMPs(const ros::Time& t)
{
    if(!_ft_sensors.empty()){
        visualization_msgs::Marker ZMP_marker;

        // Here we compute the ZMP frame of ref:
        std::string child_frame_id = "";
        yarp::sig::Vector ZMP(3, 0.0);
        // if both the feet are in the ground we use l_leg_ft as reference frame OR
        if(_ft_sensors[0].forces[2] > FT_SWITCHING_TH && _ft_sensors[1].forces[2] > FT_SWITCHING_TH)
        {
            KDL::Frame ft1_to_ft2 = robot.iDyn3_model.getPositionKDL(
                            robot.iDyn3_model.getLinkIndex(_ft_sensors[0].ft_frame),
                            robot.iDyn3_model.getLinkIndex(_ft_sensors[1].ft_frame));

            KDL::Frame ZMPR_in_l_leg_ft_frame; ZMPR_in_l_leg_ft_frame = ZMPR_in_l_leg_ft_frame.Identity();
            ZMPR_in_l_leg_ft_frame.p[0] = _ft_sensors[1].zmp[0];
            ZMPR_in_l_leg_ft_frame.p[1] = _ft_sensors[1].zmp[1];
            ZMPR_in_l_leg_ft_frame.p[2] = _ft_sensors[1].zmp[2];
            ZMPR_in_l_leg_ft_frame = ft1_to_ft2 * ZMPR_in_l_leg_ft_frame;

            yarp::sig::Vector ZMPL(3,0.0);
            ZMPL[0] = _ft_sensors[0].zmp[0]; ZMPL[1] = _ft_sensors[0].zmp[1]; ZMPL[2] = _ft_sensors[0].zmp[2];
            yarp::sig::Vector ZMPR(3,0.0);
            ZMPR[0] = ZMPR_in_l_leg_ft_frame.p.x(); ZMPR[1] = ZMPR_in_l_leg_ft_frame.p.y(); ZMPR[2] = ZMPR_in_l_leg_ft_frame.p.z();
            ZMP = cartesian_utils::computeZMP(_ft_sensors[0].forces[2], _ft_sensors[1].forces[2],
                    ZMPL, ZMPR, FT_SWITCHING_TH);

            child_frame_id = tf::resolve(_tf_prefix, _ft_sensors[0].ft_frame);
        }
        // if only left foot is in the ground we use l_leg_ft as reference frame
        else if(_ft_sensors[0].forces[2] > FT_SWITCHING_TH)
        {
            ZMP[0] = _ft_sensors[0].zmp[0]; ZMP[1] = _ft_sensors[0].zmp[1]; ZMP[2] = _ft_sensors[0].zmp[2];
            child_frame_id = tf::resolve(_tf_prefix, _ft_sensors[0].ft_frame);
        }
        // if only right foot is in the ground we use r_leg_ft as reference frame
        else if(_ft_sensors[1].forces[2] > FT_SWITCHING_TH)
        {
            ZMP[0] = _ft_sensors[1].zmp[0]; ZMP[1] = _ft_sensors[1].zmp[1]; ZMP[2] = _ft_sensors[1].zmp[2];
            child_frame_id = tf::resolve(_tf_prefix, _ft_sensors[1].ft_frame);
        }
        // probably the robot is not in contact with the feet so we assume it fell and we return
        else
            return;

        ZMP_marker.header.frame_id = child_frame_id;
        ZMP_marker.header.stamp = t;
        ZMP_marker.ns = "ZMP";
        ZMP_marker.id = 2+_ft_sensors.size();
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
}

void idyn_ros_interface::fillKinematicChainConfig(const kinematic_chain &kc,
                                                  std::map<std::string, double> &joint_names_values,
                                                  std::map<std::string, double>& joint_names_vel)
{
    for(unsigned int i = 0; i < kc.joint_names.size(); ++i){
        _q[kc.joint_numbers[i]] = joint_names_values[kc.joint_names[i]];
        _q_dot[kc.joint_numbers[i]] = joint_names_vel[kc.joint_names[i]];}
}



void idyn_ros_interface::publishCoMtf(const ros::Time &t)
{
    yarp::sig::Vector CoM( robot.iDyn3_model.getCOM());

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

void idyn_ros_interface::updateIMUCallBack(const sensor_msgs::Imu &msg)
{
    KDL::Frame world_T_imu; world_T_imu.Identity();
    world_T_imu.M = world_T_imu.M.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z ,msg.orientation.w);
    yarp::sig::Matrix world_R_imu(3,3); world_R_imu.eye();
    cartesian_utils::fromKDLRotationToYARPMatrix(world_T_imu.M, world_R_imu);
    robot.setIMUOrientation(world_R_imu, msg.header.frame_id);

    yarp::sig::Vector linear_acc(3,0.0);
    linear_acc[0] = msg.linear_acceleration.x;
    linear_acc[1] = msg.linear_acceleration.y;
    linear_acc[2] = msg.linear_acceleration.z;
    _floating_base_velocity_in_world(linear_acc*_dT);
}

void idyn_ros_interface::updateLinksInContactCallBack(const robot_state_publisher_ext::StringArray &msg)
{
    std::list<std::string> tmp_str;
    bool check = true;
    for(unsigned int i = 0; i < msg.strings.size(); ++i){
        if(robot.iDyn3_model.getLinkIndex(msg.strings[i]) == -1){
            check = false;
            ROS_ERROR("Frame %s does not exist!", msg.strings[i].c_str());
            break;}
        else
            tmp_str.push_back(msg.strings[i]);}
    if(check){
        _links_in_contact.clear();
        _links_in_contact = tmp_str;}
}

void idyn_ros_interface::publishConvexHull(const ros::Time& t)
{
    if(!_ft_sensors.empty()){

        if(!_links_in_contact.empty())
            robot.setLinksInContact(_links_in_contact);

        std::list<KDL::Vector> points;
        std::vector<KDL::Vector> ch;
        robot.getSupportPolygonPoints(points,"COM");
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
            for(std::vector<KDL::Vector>::iterator i =ch.begin(); i != ch.end(); ++i){
                p.x = i->x();
                p.y = i->y();
                p.z = i->z()-CoM[2];
                ch_marker.points.push_back(p);}

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
}
