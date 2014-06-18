#include <idyn_ros_interface.h>

using namespace iCub::iDynTree;

idyn_ros_interface::idyn_ros_interface():
    coman_idyn(),
    _n(),
    _q_subs(),
    _br(),
    _q(coman_idyn.coman_iDyn3.getNrOfDOFs(), 0.0),
    reference_frame_CoM("l_sole")
{
    _q_subs = _n.subscribe("/joint_states", 100, &idyn_ros_interface::updateIdynCallBack, this);

    coman_idyn.updateiDyn3Model(_q, _q, _q);
    coman_idyn.setWorldPose();
}

idyn_ros_interface::~idyn_ros_interface()
{

}

void idyn_ros_interface::updateIdynCallBack(const sensor_msgs::JointState &msg)
{
    std::map<std::string, double> joint_name_value_map;
    for(unsigned int i = 0; i < msg.name.size(); ++i)
        joint_name_value_map[msg.name[i]] = msg.position[i];

    fillKinematicChainConfig(coman_idyn.left_arm, joint_name_value_map);
    fillKinematicChainConfig(coman_idyn.left_leg, joint_name_value_map);
    fillKinematicChainConfig(coman_idyn.right_arm, joint_name_value_map);
    fillKinematicChainConfig(coman_idyn.right_leg, joint_name_value_map);
    fillKinematicChainConfig(coman_idyn.torso, joint_name_value_map);

    yarp::sig::Vector foo(_q.size(), 0.0);
    coman_idyn.updateiDyn3Model(_q, foo, foo);
}

void idyn_ros_interface::fillKinematicChainConfig(const kinematic_chain &kc,
                                                  std::map<std::string, double> &joint_names_values)
{
    for(unsigned int i = 0; i < kc.joint_names.size(); ++i)
        _q[kc.joint_numbers[i]] = joint_names_values[kc.joint_names[i]];
}

void idyn_ros_interface::publishCoMtf()
{
    yarp::sig::Vector CoM( coman_idyn.coman_iDyn3.getCOM("",
                           coman_idyn.coman_iDyn3.getLinkIndex(reference_frame_CoM)) );

    tf::Transform CoM_transform;
    CoM_transform.setIdentity();
    CoM_transform.setOrigin(tf::Vector3(CoM[0], CoM[1], CoM[2]));

    _br.sendTransform(tf::StampedTransform(CoM_transform, ros::Time::now(),
                                           reference_frame_CoM, "CoM"));

}
