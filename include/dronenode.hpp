#ifndef DRONENODE_HPP
#define DRONENODE_HPP

#include <ros/ros.h>
#include <QThread>
#include "nav_msgs/Odometry.h"
#include "drone.hpp"

namespace quad
{

class DroneNode : public QThread
{
    Q_OBJECT
public:
    DroneNode(int argc, char** argv);
    virtual ~DroneNode();
    bool rosIsConnected();
    bool init();
    bool init(const std::string &master_url,const std::string &host_url);
    void setUseRos(const bool use_ros);
    bool useRos() const;
    void run();

signals:
    void statesChanged(nav_msgs::Odometry* odom);

protected:
    void runRosNode();
    void runNode();
    void setupRosComms(const std::string topic="states");
    void updateDynamics();
    void stateCallback(const nav_msgs::OdometryConstPtr &msg);

private:
    int m_argc;
    char** m_argv;
    bool m_use_ros;
    std::string m_node_name{"drone_node"};
    dyn::Drone m_drone;
    double m_rate;
    nav_msgs::Odometry m_odom;
    ros::Subscriber m_state_sub;
    ros::Publisher m_state_pub;
};

} // end namespace quad
#endif // DRONENODE_HPP
