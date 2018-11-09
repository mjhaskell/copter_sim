#include "osgwidget.hpp"
#include "droneupdatecallback.hpp"

#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osgGA/EventQueue>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osg/PositionAttitudeTransform>
#include <QKeyEvent>
#include <QPainter>
#include <QWheelEvent>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>

OSGWidget::OSGWidget(QWidget* parent,Qt::WindowFlags flags):
    QOpenGLWidget{parent,flags},
    m_graphics_window{new osgViewer::GraphicsWindowEmbedded{this->x(),this->y(),this->width(),this->height()}},
    m_viewer{new osgViewer::CompositeViewer},
    m_view{new osgViewer::View},
    m_manipulator{new osgGA::TrackballManipulator},
    m_root{new osg::Group}
{
    this->setupCameraAndView();

    osg::ref_ptr<osg::Node> floor{this->createFloor()};
    m_root->addChild(floor);

    osg::Vec3d scale_factor{1,1,1};
    osg::ref_ptr<osg::Node> origin_pat{this->createOrigin(scale_factor)};
    m_root->addChild(origin_pat);

    double drone_radius{0.3};
    osg::ref_ptr<osg::PositionAttitudeTransform> drone_pat{this->createDrone(drone_radius)};
    drone_pat->addUpdateCallback(new DroneUpdateCallback);
    m_root->addChild(drone_pat);

    this->setFocusPolicy(Qt::StrongFocus);
    this->setMouseTracking(true);
    this->update();
}

OSGWidget::~OSGWidget()
{
}

void OSGWidget::timerEvent(QTimerEvent *)
{
    update();
}

void OSGWidget::paintEvent(QPaintEvent* paintEvent)
{
    this->makeCurrent();

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    this->paintGL();

    painter.end();

    this->doneCurrent();
}

void OSGWidget::paintGL()
{
    m_viewer->frame();
}

void OSGWidget::resizeGL(int width, int height)
{
    this->getEventQueue()->windowResize(this->x(), this->y(), width, height);
    m_graphics_window->resized(this->x(), this->y(), width, height);

    this->on_resize(width, height);
}

void OSGWidget::keyPressEvent(QKeyEvent* event)
{
    QString keyString{event->text()};
    const char* keyData{keyString.toLocal8Bit().data()};

    if(event->key() == Qt::Key_H)
    {
        m_view->home();
        return;
    }

    this->getEventQueue()->keyPress(osgGA::GUIEventAdapter::KeySymbol(*keyData));
}

void OSGWidget::keyReleaseEvent(QKeyEvent* event)
{
    QString keyString{event->text()};
    const char* keyData{keyString.toLocal8Bit().data()};

    this->getEventQueue()->keyRelease(osgGA::GUIEventAdapter::KeySymbol(*keyData));
}

void OSGWidget::mouseMoveEvent(QMouseEvent* event)
{
    auto pixelRatio{this->devicePixelRatio()};

    this->getEventQueue()->mouseMotion(static_cast<float>(event->x() * pixelRatio),
                                        static_cast<float>(event->y() * pixelRatio));
}

void OSGWidget::mousePressEvent(QMouseEvent* event)
{
    static int left_mouse_button{1};
    static int middle_mouse_button{2};
    static int right_mouse_button{3};

    unsigned int button{0};

    switch(event->button())
    {
    case Qt::LeftButton:
        button = left_mouse_button;
        break;

    case Qt::MiddleButton:
        button = middle_mouse_button;
        break;

    case Qt::RightButton:
        button = right_mouse_button;
        break;

    default:
        break;
    }

    auto pixelRatio{this->devicePixelRatio()};

    this->getEventQueue()->mouseButtonPress(static_cast<float>(event->x() * pixelRatio),
                                             static_cast<float>(event->y() * pixelRatio),
                                             button);

}

void OSGWidget::mouseReleaseEvent(QMouseEvent* event)
{
    static int left_mouse_button{1};
    static int middle_mouse_button{2};
    static int right_mouse_button{3};

    unsigned int button{0};

    switch(event->button())
    {
    case Qt::LeftButton:
        button = left_mouse_button;
        break;

    case Qt::MiddleButton:
        button = middle_mouse_button;
        break;

    case Qt::RightButton:
        button = right_mouse_button;
        break;

    default:
        break;
    }

    auto pixelRatio{this->devicePixelRatio()};

    this->getEventQueue()->mouseButtonRelease(static_cast<float>(pixelRatio * event->x()),
                                               static_cast<float>(pixelRatio * event->y()),
                                               button);
}

void OSGWidget::wheelEvent(QWheelEvent* event)
{
    event->accept();
    int delta{event->delta()};

    osgGA::GUIEventAdapter::ScrollingMotion motion{delta > 0 ?   osgGA::GUIEventAdapter::SCROLL_UP
                                                               : osgGA::GUIEventAdapter::SCROLL_DOWN};
    this->getEventQueue()->mouseScroll(motion);
}

void OSGWidget::on_resize(int width, int height)
{
    std::vector<osg::Camera*> cameras;
    m_viewer->getCameras(cameras);

    auto pixelRatio{this->devicePixelRatio()};

    unsigned int viewport_x{0};
    unsigned int viewport_y{0};
    cameras[0]->setViewport(viewport_x, viewport_y, width * pixelRatio, height * pixelRatio);
}

osgGA::EventQueue* OSGWidget::getEventQueue() const
{
    osgGA::EventQueue *eventQueue{m_graphics_window->getEventQueue()};

    if(eventQueue)
        return eventQueue;
    else
        throw std::runtime_error("Unable to obtain valid event queue");
}

bool OSGWidget::event(QEvent *event)
{
    bool handled{QOpenGLWidget::event(event)};
    repaintOsgGraphicsAfterInteraction(event);
    return handled;
}

void OSGWidget::repaintOsgGraphicsAfterInteraction(QEvent* event)
{
    switch(event->type())
    {
    case QEvent::KeyPress:
    case QEvent::KeyRelease:
    case QEvent::MouseButtonDblClick:
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
    case QEvent::Wheel:
        this->update();
        break;

    default:
        break;
    }
}

void OSGWidget::setupCamera(osg::Camera* camera)
{
    float aspect_ratio{static_cast<float>(this->width()) / static_cast<float>(this->height())};
    auto pixel_ratio{this->devicePixelRatio()};

    unsigned int viewport_x{0};
    unsigned int viewport_y{0};
    camera->setViewport(viewport_x, viewport_y, this->width() * pixel_ratio, this->height() * pixel_ratio);

    osg::Vec4 color_rgba{0.0f, 0.6f, 1.0f, 1.0f};
    camera->setClearColor(color_rgba);

    double angle_of_view{45.0};
    double min_distance{1.0};
    double max_distance{1000.0};
    camera->setProjectionMatrixAsPerspective(angle_of_view, aspect_ratio, min_distance, max_distance);
    camera->setGraphicsContext(m_graphics_window);
}

void OSGWidget::setupView(osg::Camera* camera)
{
    m_manipulator->setAllowThrow(false);

    osg::Vec3d camera_location{0.0,-5.0,-1.0};
    osg::Vec3d camera_center_of_focus{0,0,0};
    osg::Vec3d worlds_up_direction{0,0,-1};
    m_manipulator->setHomePosition(camera_location,camera_center_of_focus,worlds_up_direction);

    m_view->setCamera(camera);
    m_view->setSceneData(m_root.get());
    m_view->addEventHandler(new osgViewer::StatsHandler);
    m_view->setCameraManipulator(m_manipulator);
    m_view->home();
    m_view->setLightingMode(osg::View::SKY_LIGHT);
    osg::Light *light{m_view->getLight()};
    osg::Vec4 light_pos{0,0,-1,0};
    light->setPosition(light_pos);
}

void OSGWidget::setupViewer()
{
    m_viewer->addView(m_view);
    m_viewer->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
    m_viewer->realize();
}

void OSGWidget::setupCameraAndView()
{
    osg::Camera *camera{new osg::Camera};
    this->setupCamera(camera);
    this->setupView(camera);
    this->setupViewer();
}

osg::ref_ptr<osg::Vec3Array> getFloorVertices(float x, float y)
{
    float z{-0.01f};
    osg::ref_ptr<osg::Vec3Array> vertices{new osg::Vec3Array};
    vertices->push_back(osg::Vec3(-x, -y, z));
    vertices->push_back(osg::Vec3(-x,  y, z));
    vertices->push_back(osg::Vec3(x,   y, z));
    vertices->push_back(osg::Vec3(x,  -y, z));
    return vertices;
}

osg::ref_ptr<osg::Vec3Array> getFloorNormals()
{
    osg::ref_ptr<osg::Vec3Array> normals{new osg::Vec3Array};
    osg::Vec3 normal_dir{0.0f, 0.0f, 1.0f};
    normals->push_back(normal_dir);
    normals->push_back(normal_dir);
    normals->push_back(normal_dir);
    normals->push_back(normal_dir);
    return normals;
}

osg::ref_ptr<osg::Vec2Array> getTexCoords(float repetitions)
{
    osg::ref_ptr<osg::Vec2Array> tex_coords{new osg::Vec2Array};
    float zero{0.f};
    tex_coords->push_back(osg::Vec2{zero, zero});
    tex_coords->push_back(osg::Vec2{zero, repetitions});
    tex_coords->push_back(osg::Vec2{repetitions, repetitions});
    tex_coords->push_back(osg::Vec2{repetitions, zero});
    return tex_coords;
}

osg::Geometry* createFloorGeom()
{
    osg::Geometry *geom{new osg::Geometry};

    float x_dim{100.f};
    float y_dim{100.f};
    osg::ref_ptr<osg::Vec3Array> vertices{getFloorVertices(x_dim,y_dim)};
    geom->setVertexArray(vertices);

    osg::ref_ptr<osg::Vec3Array> normals{getFloorNormals()};
    geom->setNormalArray(normals, osg::Array::Binding::BIND_PER_VERTEX);

    float repetions{100.f};
    osg::ref_ptr<osg::Vec2Array> tex_coords{getTexCoords(repetions)};
    geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
    geom->setTexCoordArray(0, tex_coords.get());
    osgUtil::SmoothingVisitor::smooth(*geom);
    geom->setTexCoordArray(0, tex_coords.get(), osg::Array::Binding::BIND_PER_VERTEX);

    return geom;
}

osg::ref_ptr<osg::Texture2D> createFloorTexture()
{
    osg::ref_ptr<osg::Texture2D> texture{new osg::Texture2D};
    osg::ref_ptr<osg::Image> image{osgDB::readImageFile("../obj/tile.jpg")};
    texture->setImage(image);
    texture->setUnRefImageDataAfterApply(true);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    return texture;
}

osg::ref_ptr<osg::Node> OSGWidget::createFloor()
{
    osg::Geometry *geom{createFloorGeom()};
    osg::ref_ptr<osg::Texture2D> texture{createFloorTexture()};

    osg::StateSet *geom_state_set = geom->getOrCreateStateSet();
    geom_state_set->setTextureAttributeAndModes(0, texture.get(), osg::StateAttribute::ON);

    return geom;
}

osg::Geometry* getOriginAxis(int x,int y,int z)
{
    osg::Vec3Array* v{new osg::Vec3Array};
    v->resize(2);
    (*v)[0].set(0, 0, 0);
    (*v)[1].set(x, y, z);

    osg::Geometry* geom{new osg::Geometry};
    geom->setUseDisplayList(false);
    geom->setVertexArray(v);
    osg::Vec4 color{float(z),float(y),float(x),1.f};
    osg::Vec4Array* c{new osg::Vec4Array};
    c->push_back(color);
    geom->setColorArray(c, osg::Array::BIND_OVERALL);

    GLushort idx_lines[2] = {0, 1};
    geom->addPrimitiveSet(new osg::DrawElementsUShort{osg::PrimitiveSet::LINES, 2, idx_lines});
    return geom;
}

osg::ref_ptr<osg::Node> OSGWidget::createOrigin(osg::Vec3d &scale_factor)
{
    osg::ref_ptr<osg::Geometry> x_axis{getOriginAxis(1,0,0)};
    osg::ref_ptr<osg::Geometry> y_axis{getOriginAxis(0,1,0)};
    osg::ref_ptr<osg::Geometry> z_axis{getOriginAxis(0,0,1)};

    osg::ref_ptr<osg::Geode> geode{new osg::Geode};
    geode->addDrawable(x_axis);
    geode->addDrawable(y_axis);
    geode->addDrawable(z_axis);

    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    osg::ref_ptr<osg::PositionAttitudeTransform> transform{new osg::PositionAttitudeTransform};
    transform->setScale(scale_factor);

    transform->addChild(geode);
    return transform;
}

osg::ref_ptr<osg::Node> scaleModel(const osg::ref_ptr<osg::Node> &model, double bounding_radius)
{
    osg::BoundingSphere bb{model->getBound()};

    osg::ref_ptr<osg::PositionAttitudeTransform> scale_trans{new osg::PositionAttitudeTransform};
    double bounding_box_radius{bb.radius()};
    double radius_ratio{bounding_radius/bounding_box_radius};
    scale_trans->setScale(osg::Vec3d{radius_ratio,radius_ratio,radius_ratio});
    scale_trans->addChild(model);

    return scale_trans.release();
}

osg::ref_ptr<osg::Node> rotateModel(const osg::ref_ptr<osg::Node> &model)
{
    double angle{osg::DegreesToRadians(90.0)};
    osg::Vec3d axis1{1,0,0};
    osg::Vec3d axis2{0,0,1};
    osg::Quat q1{angle,axis1};
    osg::Quat q2{angle,axis2};
    osg::ref_ptr<osg::PositionAttitudeTransform> pat{new osg::PositionAttitudeTransform};
    pat->setAttitude(q1*q2);
    pat->addChild(model);
    return pat.release();
}

osg::ref_ptr<osg::Node> translateModel(const osg::ref_ptr<osg::Node> &model)
{
    osg::BoundingSphere bb{model->getBound()};
    osg::ref_ptr<osg::PositionAttitudeTransform> pat{new osg::PositionAttitudeTransform};
    osg::Vec3d pos{bb.center()};
    double cog_offset{-0.065};
    pos = osg::Vec3d{-pos.x(),-pos.y()+cog_offset,-pos.z()};
    pat->setPosition(pos);
    pat->addChild(model);
    return pat.release();
}

osg::ref_ptr<osg::Node> createDroneModel()
{
    osg::ref_ptr<osg::Node> model{osgDB::readNodeFile("../obj/simple_drone.obj")};

    if(model.valid())
    {
        osg::StateSet* state_set{model->getOrCreateStateSet()};
        state_set->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
        state_set->setMode(GL_RESCALE_NORMAL,osg::StateAttribute::ON);
    }
    return model.release();
}

osg::ref_ptr<osg::PositionAttitudeTransform> OSGWidget::createDrone(double bounding_radius)
{
    osg::ref_ptr<osg::Node> model{createDroneModel()};
    osg::ref_ptr<osg::Node> scaled_model{scaleModel(model,bounding_radius)};
    osg::ref_ptr<osg::Node> translated_model{translateModel(scaled_model)};
    osg::ref_ptr<osg::Node> rotated_model{rotateModel(translated_model)};
    osg::ref_ptr<osg::PositionAttitudeTransform> drone_at_origin{new osg::PositionAttitudeTransform};
    drone_at_origin->addChild(rotated_model);

    return drone_at_origin.release();
}
