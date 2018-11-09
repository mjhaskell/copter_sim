#include "droneupdatecallback.hpp"
#include <osg/PositionAttitudeTransform>

DroneUpdateCallback::DroneUpdateCallback() :
    m_z{0}
{
}

void DroneUpdateCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
{
    osg::Vec3d pos{0,0,m_z};
    osg::PositionAttitudeTransform *pat{dynamic_cast<osg::PositionAttitudeTransform*>(node)};
    pat->setPosition(pos);
    traverse(node, nv);
    if (m_z > -5)
        m_z -= 0.01;
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
