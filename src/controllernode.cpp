#include "controllernode.hpp"
#include <chrono>

namespace quad
{

ControllerNode::ControllerNode() :
    m_rate{0.01},
    m_is_running{false}
{
    m_states.setZero(dyn::STATE_SIZE,1);
    m_cmds.setZero(dyn::INPUT_SIZE,1);
    m_odom.pose.pose.position.x = 0;
    m_odom.pose.pose.position.y = 0;
    m_odom.pose.pose.position.z = 0;
    m_odom.pose.pose.orientation.w = 1;
    m_odom.pose.pose.orientation.x = 0;
    m_odom.pose.pose.orientation.y = 0;
    m_odom.pose.pose.orientation.z = 0;
}

quad::ControllerNode::~ControllerNode()
{
    wait();
}

void ControllerNode::run()
{
    m_is_running = true;

    while (m_is_running)
    {
        auto t_start{std::chrono::high_resolution_clock::now()};
        m_cmds = m_controller.calculateControl(m_states);
        emit sendInputs(m_cmds);
        while(std::chrono::duration<double,std::milli>(std::chrono::high_resolution_clock::now()-t_start).count() < m_rate) {}
    }
}

void ControllerNode::startNode()
{
    start();
}

void ControllerNode::updateStates(const dyn::xVec &states)
{
    m_states = states;
}

} // end namespace quad
