#include <idyn_ros_interface.h>

using namespace iCub::iDynTree;

idyn_ros_interface::idyn_ros_interface():
    coman_idyn(),
    _n(),
    _q_subs(),
    _br(),
    _lr(),
    _q(coman_idyn.coman_iDyn3.getNrOfDOFs(), 0.0),
    reference_frame_CoM("l_sole"),
    reference_frame_base_foot_print("base_link")
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

void idyn_ros_interface::publishBaseFootPrint()
{
    /*
     * The base_footprint is the representation of the robot position on the floor.
     * The floor is usually the level where the supporting leg rests,
     * i.e. z = min(l_sole_z, r_sole_z) where l_sole_z and r_sole_z are the left and
     * right sole height respecitvely. The translation component of the frame should be
     * the barycenter of the feet projections on the floor. With respect to the odom
     * frame, the roll and pitch angles should be zero and the yaw angle should
     * correspond to the base_link yaw angle.
     */

    tf::Transform base_footprint;
    base_footprint.setIdentity();


    tf::StampedTransform l_foot, r_foot;
    l_foot.setIdentity(); r_foot.setIdentity();
    try
    {
        _lr.lookupTransform(reference_frame_base_foot_print, "/l_sole",
        ros::Time(0), l_foot);
        _lr.lookupTransform(reference_frame_base_foot_print, "/r_sole",
        ros::Time(0), r_foot);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    base_footprint.setOrigin(tf::Vector3(0.0, 0.0, std::min(l_foot.getOrigin().z(),
                                                            r_foot.getOrigin().z())));

    /* Here we will use the IMU when it will work instead of left_foot! */
    tf::StampedTransform base_link;
    base_link.setIdentity();
    tf::Quaternion base_link_rot;
    base_link_rot.setRPY(0.0, 0.0, tf::getYaw(l_foot.inverse().getRotation()));
    base_link.setRotation(base_link_rot);
    base_footprint.setRotation(l_foot.getRotation()*base_link.getRotation());

    _br.sendTransform(tf::StampedTransform(base_footprint, ros::Time::now(),
                                           reference_frame_base_foot_print,
                                           "base_footprint"));
}
