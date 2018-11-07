#include "osgwidget.hpp"

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

OSGWidget::OSGWidget(QWidget* parent,Qt::WindowFlags flags):
    QOpenGLWidget{parent,flags},
    m_graphics_window{new osgViewer::GraphicsWindowEmbedded{this->x(),this->y(),this->width(),this->height()}},
    m_viewer{new osgViewer::CompositeViewer},
    m_view{new osgViewer::View},
    m_manipulator{new osgGA::TrackballManipulator},
    m_root{new osg::Group}
{
    this->setupCameraAndView();
//    this->create_ironman(5);
    osg::ref_ptr<osg::Node> ironman_pat{this->create_ironman(0.3)};
    m_root->addChild(ironman_pat);
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

    osg::Vec4 color_rgba{0.3f, 0.3f, 0.3f, 1.0f};
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

    osg::Vec3d camera_location{0.0,-20.0,3.0};
    osg::Vec3d camera_center_of_focus{0,0,0};
    osg::Vec3d worlds_up_direction{0,0,1};
    m_manipulator->setHomePosition(camera_location,camera_center_of_focus,worlds_up_direction);

    m_view->setCamera(camera);
    m_view->setSceneData(m_root.get());
    m_view->addEventHandler(new osgViewer::StatsHandler);
    m_view->setCameraManipulator(m_manipulator);
    m_view->home();
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

//The model has it's own scale.  This function creates a node to scale the model to fit inside the bounding sphere defined by the radius.
osg::ref_ptr<osg::Node> create_scaled_model(osg::ref_ptr<osg::Node> model, double boundingRadius)
{
    osg::BoundingSphere bb = model->getBound();

     osg::ref_ptr<osg::PositionAttitudeTransform> scaleTrans = new osg::PositionAttitudeTransform;
     double boundingBoxRadius = bb.radius();
     double radiusRatio{boundingRadius/boundingBoxRadius};
     scaleTrans->setScale(osg::Vec3d(radiusRatio,radiusRatio,radiusRatio));
     scaleTrans->addChild(model);

     return scaleTrans.release();
}

//The model is at it's own position in 3d space.  This funciton creates a node to position the model at the origin.
osg::ref_ptr<osg::Node> create_translated_model(osg::ref_ptr<osg::Node> model)
{
    osg::BoundingSphere bb = model->getBound();
    osg ::ref_ptr<osg::PositionAttitudeTransform> positionTrans = new osg::PositionAttitudeTransform;
    osg::Vec3d pos=bb.center();
    pos=osg::Vec3d(pos.x()*-1,pos.y()*-1,pos.z()*-1);
    positionTrans->setPosition(pos);
    positionTrans->addChild(model);
    return positionTrans.release();
}


osg::ref_ptr<osg::Node> create_model()
{
//    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("IronMan.3DS");
    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("simple_drone.obj");

    if(model.valid())
    {
        osg::StateSet* stateSet = model->getOrCreateStateSet();
        stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
        stateSet->setMode(GL_RESCALE_NORMAL ,osg::StateAttribute::ON);
    }
    return model.release();
}

osg::ref_ptr<osg::Node> OSGWidget::create_ironman(double boundingRadius)
{
    osg::ref_ptr<osg::Node> model = create_model();
    osg::ref_ptr<osg::Node> scaledModel = create_scaled_model(model,boundingRadius);
    osg::ref_ptr<osg::Node> translatedModel = create_translated_model(scaledModel);

    return translatedModel.release();
}
