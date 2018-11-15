#include "droneupdatecallback.hpp"
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

DroneUpdateCallback::DroneUpdateCallback(osg::ref_ptr<osgGA::TrackballManipulator> manipulator) :
    m_manipulator{manipulator},
    m_q_i2c{0,0,0,1},
    m_max_angle{osg::DegreesToRadians(17.0)},
    m_pos{0,0,0},
    m_att{0,0,0,1},
    m_eye{-5.0,0,-1.0},
    m_center{0,0,0},
    m_up{0,0,-1.0}
{
}

void DroneUpdateCallback::updateManipulator()
{
    osg::Vec3d pos_i{m_pos.x()-m_eye.x(),m_pos.y(),0};
    osg::Vec3d pos_c{m_q_i2c.conj()*pos_i};
    double angle_to_drone{atan2(pos_c.y(),pos_c.x())};
    if (angle_to_drone > m_max_angle)
    {
        double rot_angle{angle_to_drone - m_max_angle};
        m_q_i2c *= osg::Quat{rot_angle,osg::Vec3d{0,0,1}};
    }
    else if (angle_to_drone < -m_max_angle)
    {
        double rot_angle{angle_to_drone + m_max_angle};
        m_q_i2c *= osg::Quat{rot_angle,osg::Vec3d{0,0,1}};
    }
    double mag{pos_c.length()};
    osg::Vec3d cam_center{mag,0,0};
    cam_center = m_q_i2c*cam_center;
    m_center.set(cam_center.x()+m_eye.x(),cam_center.y(),m_pos.z());

    m_manipulator->setTransformation(m_eye,m_center,m_up);
}

void DroneUpdateCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
{
    osg::PositionAttitudeTransform *pat{dynamic_cast<osg::PositionAttitudeTransform*>(node)};
    pat->setPosition(m_pos);
    pat->setAttitude(m_att);
    this->updateManipulator();

    traverse(node, nv);
}

void DroneUpdateCallback::updateStates(osg::Vec3d new_pos, osg::Quat new_att)
{
    m_pos = new_pos;
    m_att = new_att;
}
