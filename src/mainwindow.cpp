#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include "osgwidget.hpp"
#include "dronenode.hpp"
#include <ros/ros.h>
#include <QToolBar>
#include <QProcess>

MainWindow::MainWindow(int argc,char** argv,QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow),
    m_drone_node{argc,argv},
    m_process{new QProcess{this}}
{
    ros::init(argc,argv,"main");
    if (!ros::master::check())
    {
        QString program{"roscore"};
        m_process->start(program);
        m_app_started_roscore = true;
        std::string uri{ros::master::getURI()};
        ROS_WARN("This application started a roscore on %s and will shut it down upon closing.",uri.c_str());
    }

    m_ui->setupUi(this);
    OSGWidget *osg_widget{new OSGWidget};
    setCentralWidget(osg_widget);
    this->createToolbar();

    connect(&m_drone_node,&quad::DroneNode::statesChanged,
            osg_widget, &OSGWidget::updateDroneStates);
}

MainWindow::~MainWindow()
{
    delete m_ui;
    if (m_app_started_roscore)
    {
        m_process->close();
        QProcess kill_roscore;
        kill_roscore.start(QString{"killall"}, QStringList() << "-9" << "roscore");
        kill_roscore.waitForFinished();
        kill_roscore.close();

        QProcess kill_rosmaster;
        kill_rosmaster.start(QString{"killall"}, QStringList() << "-9" << "rosmaster");
        kill_rosmaster.waitForFinished();
        kill_rosmaster.close();
    }
}

void MainWindow::createToolbar()
{
    QToolBar *tool_bar{addToolBar(tr("Main Actions Toolbar"))};
    QAction *start_action{createStartAction()};
    tool_bar->addAction(start_action);

    m_main_toolbar = tool_bar;
}

QAction* MainWindow::createStartAction()
{
    const QIcon start_icon{QIcon(":myicons/start.png")};
    QAction *start_action{new QAction(start_icon, tr("&Run Simulation (Ctrl+R)"), this)};
    start_action->setShortcut(QKeySequence{tr("Ctrl+R")});
    start_action->setStatusTip(tr("Run simulation will either begin or resume simulation"));
    connect(start_action, &QAction::triggered, this, &MainWindow::startSimulation);

    return start_action;
}

void MainWindow::startSimulation()
{
    m_drone_node.init();
}
