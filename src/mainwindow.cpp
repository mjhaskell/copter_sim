#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include "osgwidget.hpp"
#include "dronenode.hpp"

MainWindow::MainWindow(int argc,char** argv,QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_drone_node{argc,argv}
{
    m_ui->setupUi(this);
    m_drone_node.init();
//    OSGWidget *osg_widget{OSGWidget*(centralWidget())};
    OSGWidget *osg_widget{new OSGWidget};
    setCentralWidget(osg_widget);
    connect(&m_drone_node,&quad::DroneNode::statesChanged,
            osg_widget, &OSGWidget::updateDroneStates);
}

MainWindow::~MainWindow()
{
    delete m_ui;
}
