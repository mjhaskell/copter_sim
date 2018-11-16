#ifndef DRONENODE_HPP
#define DRONENODE_HPP

#include <ros/ros.h>
#include <QThread>
#include "nav_msgs/Odometry.h"
#include "rosflight_msgs/Command.h"

namespace quad
{
typedef struct
{
    double pn;
    double pe;
    double pd;

    double phi;
    double theta;
    double psi;

    double u;
    double v;
    double w;

    double p;
    double q;
    double r;
}state_t;

typedef struct
{
    double u1;
    double u2;
    double u3;
    double u4;

    double F;
    double phi_c;
    double theta_c;
    double r_c;
}input_t;

class DroneNode : public QThread
{
    Q_OBJECT
public:
    DroneNode(int argc, char** argv);
    virtual ~DroneNode();
    bool init();
    bool init(const std::string &master_url,const std::string &host_url);
    void run();

signals:
    void statesChanged(nav_msgs::Odometry* odom);

private:
    std::string m_node_name{"drone_node"};
    int m_argc;
    char** m_argv;
    state_t m_x;
    input_t m_u;
    uint8_t m_control_mode;
    nav_msgs::Odometry m_odom;
    ros::Publisher m_state_pub;
    ros::Subscriber m_cmd_sub;
    void setupRosComms();
    void updateDynamics();
    void cmdCallback(const rosflight_msgs::CommandConstPtr &msg);
};

} // end namespace quad
#endif // DRONENODE_HPP
