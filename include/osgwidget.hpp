#ifndef OSGWIDGET_HPP
#define OSGWIDGET_HPP

#include <QOpenGLWidget>

#include <osg/ref_ptr>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>

class OSGWidget : public QOpenGLWidget
{
  Q_OBJECT

public:
  OSGWidget(QWidget* parent = 0,Qt::WindowFlags f = 0);
  virtual ~OSGWidget();

protected:
  virtual void paintEvent(QPaintEvent* paintEvent);
  virtual void paintGL();
  virtual void resizeGL(int width, int height);
  virtual void keyPressEvent(QKeyEvent* event);
  virtual void keyReleaseEvent(QKeyEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void wheelEvent(QWheelEvent* event);
  virtual bool event(QEvent* event);
  void timerEvent(QTimerEvent *);
  void repaintOsgGraphicsAfterInteraction(QEvent* event);

private:
  virtual void on_resize(int width, int height);
  osgGA::EventQueue* getEventQueue() const;
  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> m_graphics_window;
  osg::ref_ptr<osgViewer::CompositeViewer> m_viewer;
  osg::ref_ptr<osgViewer::View> m_view;
  osg::ref_ptr<osgGA::TrackballManipulator> m_manipulator;
  osg::ref_ptr<osg::Group> m_root;
  void setupCamera(osg::Camera* camera);
  void setupView(osg::Camera* camera);
  void setupViewer();
  void setupCameraAndView();
  osg::ref_ptr<osg::Node> create_ironman(double boundingRadius);
};

#endif // OSGWIDGET_HPP
