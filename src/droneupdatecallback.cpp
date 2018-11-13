#include "droneupdatecallback.hpp"
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

DroneUpdateCallback::DroneUpdateCallback(osg::ref_ptr<osgGA::TrackballManipulator> manipulator) :
    m_manipulator{manipulator},
    m_max_angle{osg::DegreesToRadians(17.0)},
    m_eye{-5.0,0,-1.0},
    m_center{0,0,0},
    m_up{0,0,-1.0}
{
}

void DroneUpdateCallback::updateManipulator()
{
    osg::Vec3d pos_i{m_x-m_eye.x(),m_y,0};
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
    m_center.set(cam_center.x()+m_eye.x(),cam_center.y(),m_z);

    m_manipulator->setTransformation(m_eye,m_center,m_up);
}

void DroneUpdateCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
{
    osg::Vec3d pos{m_x,m_y,m_z};
    osg::PositionAttitudeTransform *pat{dynamic_cast<osg::PositionAttitudeTransform*>(node)};
    pat->setPosition(pos);
    this->updateManipulator();

    if (m_z > -3)
        m_z -= 0.01;
    if(m_y_up)
    {
        m_y_count++;
        m_y -= .02;
    }
    else
    {
        m_y_count--;
        m_y += .02;
    }
    if(m_x_up)
    {
        m_x_count++;
        m_x -= .02;
    }
    else
    {
        m_x_count--;
        m_x += .02;
    }
//    if (m_y < 5)
//    {
//        m_y += 0.03;
//        m_x-=.01;
//    }

    if(m_y_count==250 || m_y_count==-250)
        m_y_up=!m_y_up;
    if(m_y_count==500 || m_y_count==0)
        m_x_up=!m_x_up;

    traverse(node, nv);
}
