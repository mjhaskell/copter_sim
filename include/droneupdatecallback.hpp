#ifndef DRONEUPDATECALLBACK_HPP
#define DRONEUPDATECALLBACK_HPP

#include <osg/Node>

class DroneUpdateCallback : public osg::NodeCallback
{
public:
    DroneUpdateCallback();
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);

protected:
    double m_z{0};
};

#endif // DRONEUPDATECALLBACK_HPP

//class SphereUpdateCallback : public osg::NodeCallback
//{
//public:
//    SphereUpdateCallback(phys::Sphere* sphere);
//    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
//    static void setTimerActive(bool timer_active);
//    static bool getTimerActive();

//protected:
//    phys::Sphere *m_phys_sphere{nullptr};
//};
