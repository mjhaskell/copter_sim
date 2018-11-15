#include "dronenode.hpp"
#include <string>
#include "nav_msgs/Odometry.h"
#include "rosflight_msgs/Command.h"
#include <osg/Quat>

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
    ros::init(m_argc,m_argv,"flight_controller_node");
    if (!ros::master::check())
        return false;
    ros::start();
    ros::NodeHandle nh;
    m_state_pub = nh.advertise<nav_msgs::Odometry>("states", 25);
    m_cmd_sub = nh.subscribe("commands", 50, &DroneNode::cmdCallback,this);
    //    volatile FlightControllerROS fc_ros;
    start();
    return true;
}

bool DroneNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"qt_test");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start();
    ros::NodeHandle nh;
    m_state_pub = nh.advertise<nav_msgs::Odometry>("states", 25);
    m_cmd_sub = nh.subscribe("commands", 50, &DroneNode::cmdCallback,this);
    start();
    return true;
}

void DroneNode::run()
{
    ros::Rate publish_rate(30);
    while (ros::ok())
    {
        this->updateDynamics();
        m_state_pub.publish(m_odom);
        ros::spinOnce();
        publish_rate.sleep();
    }
}

void DroneNode::updateDynamics()
{
    if (m_odom.pose.pose.position.z > -3)
        m_odom.pose.pose.position.z -= 0.01;
    osg::Quat q{m_odom.pose.pose.orientation.x,m_odom.pose.pose.orientation.y,
               m_odom.pose.pose.orientation.z,m_odom.pose.pose.orientation.w};
    q*=osg::Quat{osg::DegreesToRadians(10.0),osg::Vec3d{1,0,0}};
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

} // end namespace fc
