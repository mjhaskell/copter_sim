#include "dronenode.hpp"
#include "drone.hpp"
#include <ros/ros.h>
#include <string>
#include "nav_msgs/Odometry.h"
#include "rosflight_msgs/Command.h"
//#include <osg/Quat>
#include "quat.hpp"

namespace quad
{

DroneNode::DroneNode(int argc, char **argv) :
    m_argc{argc},
    m_argv{argv}
{
    m_odom.pose.pose.position.x = 0;
    m_odom.pose.pose.position.y = 0;
    m_odom.pose.pose.position.z = 0;
    m_odom.pose.pose.orientation.w = 1;
    m_odom.pose.pose.orientation.x = 0;
    m_odom.pose.pose.orientation.y = 0;
    m_odom.pose.pose.orientation.z = 0;
}

DroneNode::~DroneNode()
{
    if(ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

bool DroneNode::init()
{
    ros::init(m_argc,m_argv,m_node_name);
    if (!ros::master::check())
        return false;
    this->setupRosComms();
    return true;
}

bool DroneNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,m_node_name);
    if (!ros::master::check())
        return false;
    this->setupRosComms();
    return true;
}

void DroneNode::run()
{
    ros::Rate publish_rate{500};
    while (ros::ok())
    {
        this->updateDynamics();
        m_state_pub.publish(m_odom);
        ros::spinOnce();
        publish_rate.sleep();
    }
}

void DroneNode::setupRosComms()
{
    ros::start();
    ros::NodeHandle nh;
    int states_queue_size{25};
    int cmd_queue_size{50};
    m_state_pub = nh.advertise<nav_msgs::Odometry>("states", states_queue_size);
    m_cmd_sub = nh.subscribe("commands", cmd_queue_size, &DroneNode::cmdCallback,this);
    start();
}

void DroneNode::updateDynamics()
{
    dyn::uVec u;
    u << 0.8,0.8,0.8,0.8;
    m_drone.sendMotorCmds(u);
    dyn::xVec x{m_drone.getStates()};
    quat::Quatd q{quat::Quatd::from_euler(x(dyn::RX),x(dyn::RY),x(dyn::RZ))};
    m_odom.pose.pose.position.x = x(dyn::PX);
    m_odom.pose.pose.position.y = x(dyn::PY);
    m_odom.pose.pose.position.z = x(dyn::PZ);
//    if (m_odom.pose.pose.position.z > -3)
//    {
//        m_odom.pose.pose.position.z -= 0.01;
//        m_odom.pose.pose.position.y -= 0.01;
//    }

    m_odom.pose.pose.orientation.w = q.w();
    m_odom.pose.pose.orientation.x = q.x();
    m_odom.pose.pose.orientation.y = q.y();
    m_odom.pose.pose.orientation.z = q.z();
    emit statesChanged(&m_odom);
}

void DroneNode::cmdCallback(const rosflight_msgs::CommandConstPtr &msg)
{
    m_control_mode = msg->mode;
    switch(msg->mode)
    {
    case rosflight_msgs::Command::MODE_PASS_THROUGH:
        m_u.u1 = msg->x;
        m_u.u2 = msg->y;
        m_u.u3 = msg->z;
        m_u.u4 = msg->F;
        break;
    case rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
        m_u.phi_c = msg->x;
        m_u.theta_c = msg->y;
        m_u.r_c = msg->z;
        m_u.F = msg->F;
        break;
    default:
        ROS_ERROR("FlightControllerNode: Unhandled command message of type %d",msg->mode);
        break;
    }
}

} // end namespace quad
