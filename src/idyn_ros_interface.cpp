#include <idyn_ros_interface.h>
#include <kdl/frames.hpp>

using namespace iCub::iDynTree;

idyn_ros_interface::idyn_ros_interface():
    idynutils(),
    _n(),
    _q_subs(),
    _br(),
    _lr(),
    _q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0),
    reference_frame_CoM("world"),
    reference_frame_base_foot_print("CoM")
{
    _q_subs = _n.subscribe("/joint_states", 100, &idyn_ros_interface::updateIdynCallBack, this);

    idynutils.updateiDyn3Model(_q, _q, _q, true);
}

idyn_ros_interface::~idyn_ros_interface()
{

}

void idyn_ros_interface::updateIdynCallBack(const sensor_msgs::JointState &msg)
{
    std::map<std::string, double> joint_name_value_map;
    for(unsigned int i = 0; i < msg.name.size(); ++i)
        joint_name_value_map[msg.name[i]] = msg.position[i];

    fillKinematicChainConfig(idynutils.left_arm, joint_name_value_map);
    fillKinematicChainConfig(idynutils.left_leg, joint_name_value_map);
    fillKinematicChainConfig(idynutils.right_arm, joint_name_value_map);
    fillKinematicChainConfig(idynutils.right_leg, joint_name_value_map);
    fillKinematicChainConfig(idynutils.torso, joint_name_value_map);

    yarp::sig::Vector foo(_q.size(), 0.0);
    idynutils.updateiDyn3Model(_q, foo, foo, true);
}

void idyn_ros_interface::fillKinematicChainConfig(const kinematic_chain &kc,
                                                  std::map<std::string, double> &joint_names_values)
{
    for(unsigned int i = 0; i < kc.joint_names.size(); ++i)
        _q[kc.joint_numbers[i]] = joint_names_values[kc.joint_names[i]];
}

void idyn_ros_interface::publishCoMtf()
{
    yarp::sig::Vector CoM( idynutils.iDyn3_model.getCOM());
                           //idynutils.iDyn3_model.getLinkIndex(reference_frame_CoM)) );

    tf::Transform CoM_transform;
    CoM_transform.setIdentity();
    CoM_transform.setOrigin(tf::Vector3(CoM[0], CoM[1], CoM[2]));

    _br.sendTransform(tf::StampedTransform(CoM_transform, ros::Time::now(),
                                           reference_frame_CoM, "CoM"));

}

void idyn_ros_interface::publishWorld()
{
    KDL::Frame world_T_base_link =  idynutils.iDyn3_model.getWorldBasePoseKDL();

    double qx, qy, qz, qw;
    world_T_base_link.M.GetQuaternion(qx, qy, qz, qw);

    tf::Transform world_T_base_link_tf;
    world_T_base_link_tf.setOrigin(tf::Vector3(world_T_base_link.p.x(), world_T_base_link.p.y(), world_T_base_link.p.z()));
    world_T_base_link_tf.setRotation(tf::Quaternion(qx, qy, qz, qw));

    _br.sendTransform(tf::StampedTransform(world_T_base_link_tf, ros::Time::now(),
                                           "world", "base_link"));
}
