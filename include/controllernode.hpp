#ifndef CONTROLLERNODE_HPP
#define CONTROLLERNODE_HPP

#include <QThread>
#include "nav_msgs/Odometry.h"
#include "controller.hpp"

namespace quad
{

class ControllerNode : public QThread
{
    Q_OBJECT
public:
    ControllerNode();
    virtual ~ControllerNode();
//    void init();
    void run();
    void startNode();

signals:
    void sendInputs(const dyn::uVec &inputs);

public slots:
    void updateStates(const dyn::xVec &states);

private:
    dyn::Controller m_controller;
    dyn::xVec m_states;
    nav_msgs::Odometry m_odom;
    dyn::uVec m_cmds;
    double m_rate;
    bool m_is_running;
};

} // end namespace quad
#endif // CONTROLLERNODE_HPP
