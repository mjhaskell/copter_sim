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
    int m_x_count{0};
    int m_y_count{0};
    bool m_x_up{true};
    bool m_y_up{true};
    osg::Quat m_q_i2c{0.0,0.0,0.0,1.0};
    double m_max_angle{0};
private:
    void updateManipulator();
    osg::Vec3d m_eye{0,0,0};
    osg::Vec3d m_center{0,0,0};
    osg::Vec3d m_up{0,0,0};
};

#endif // DRONEUPDATECALLBACK_HPP
