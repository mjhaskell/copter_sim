#ifndef DRONEUPDATECALLBACK_HPP
#define DRONEUPDATECALLBACK_HPP

#include <osg/Node>

namespace osgGA
{
    class TrackballManipulator;
}

class DroneUpdateCallback : public osg::NodeCallback
{
public:
    DroneUpdateCallback(osg::ref_ptr<osgGA::TrackballManipulator>);
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);

protected:
    osg::ref_ptr<osgGA::TrackballManipulator> m_manipulator{nullptr};
    double m_x{0};
    double m_y{0};
    double m_z{0};
    int m_count{0};
    bool m_up{true};
    osg::Quat m_q_i2c{0.0,0.0,0.0,1.0};
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
