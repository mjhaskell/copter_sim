#include "droneupdatecallback.hpp"
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

DroneUpdateCallback::DroneUpdateCallback(osg::ref_ptr<osgGA::TrackballManipulator> manipulator) :
    m_manipulator{manipulator}
{
}

void DroneUpdateCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
{
//    double angle{osg::DegreesToRadians(30.0)};
//    osg::Vec3d axis{0,0,1};
//    osg::Quat q_i2c{angle,axis};
//    osg::Vec3d vec{1,0,0};
//    vec = q_i2c.conj()*vec;
//    q_i2c.getRotate(angle,axis);
//    angle = osg::RadiansToDegrees(angle);
    osg::Vec3d pos{m_x,m_y,m_z};
    osg::PositionAttitudeTransform *pat{dynamic_cast<osg::PositionAttitudeTransform*>(node)};
    pat->setPosition(pos);

    osg::Matrix model_matrix{pat->getWorldMatrices()[0]};
//    osg::Matrix view_matrix{m_manipulator->getv}

    osg::Vec3d pos_i{pos.x(),pos.y(),0};
    osg::Vec3d pos_c{m_q_i2c.conj()*pos_i};
    double angle_to_drone{atan2(pos_c.y(),pos_c.x()+5)};
    if (osg::RadiansToDegrees(angle_to_drone) > 25.0)
    {
        double rot_angle{angle_to_drone - osg::DegreesToRadians(25.0)};
        m_q_i2c *= osg::Quat{rot_angle,osg::Vec3d{0,0,1}};
//        pos = osg::Quat{-rot_angle,osg::Vec3d{0,0,1}}*osg::Vec3d{pos_i.x(),pos_i.y(),m_z};
    }
//    else
//    {
        osg::Vec3d pos_copy{pos_c};
        pos_copy.set(pos_copy.x(),pos_copy.y(),0);
        double mag{pos_copy.length()};
        pos_copy.set(mag,0,0);
        pos_copy = m_q_i2c*pos_copy;
        pos.set(pos_copy.x()-5,pos_copy.y(),pos.z());
//    }



//    pos.set(pos.x(,pos.y(),))
    traverse(node, nv);
    if (m_z > -3)
        m_z -= 0.01;
    if(m_up)
    {
        m_count++;
        m_y += .01;
    }
    else
    {
        m_count--;
        m_y -= .01;
    }
//    m_y += 0.01;
    m_x-=.01;
    osg::Vec3d eye{-7.5,0,-1};
    osg::Vec3d center{pos};
    osg::Vec3d up{0,0,-1};
    m_manipulator->setTransformation(eye,center,up);

    if(m_count==150 || m_count==0)
        m_up=!m_up;
}


//#include "sphereupdatecallback.hpp"
//#include "libs/physics/sphere.hpp"
//#include <osg/PositionAttitudeTransform>

//SphereUpdateCallback::SphereUpdateCallback(phys::Sphere* sphere):
//    m_phys_sphere{sphere}
//{
//}

//void SphereUpdateCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
//{
//    phys::Vec3 position{m_phys_sphere->getPosition()};
//    osg::Vec3 pos{float(position.getX()),float(position.getY()),float(position.getZ())};

//    osg::PositionAttitudeTransform *pat{dynamic_cast<osg::PositionAttitudeTransform*>(node)};
//    pat->setPosition(pos);

//    traverse(node, nv);
//}
