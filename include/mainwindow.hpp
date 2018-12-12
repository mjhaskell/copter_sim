#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include "dronenode.hpp"
#include "controllernode.hpp"
#include <QProcess>

namespace Ui
{
class MainWindow;
}

class OSGWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc,char** argv,QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void setupSignalsAndSlots();
    void closeEvent(QCloseEvent *);
    void readSettings();
    void writeSettings();
    void updateRosStatus();
    void startRosCore();
    void createToolbar();
    void setupStatusBar();
    QAction* createStartAction();
    void startSimulation();
    QAction *createRoscoreAction();
    
public slots:
    void closeWithWarning();

private slots:
    void on_start_triggered();
    void on_close_triggered();
    void on_roscore_button_clicked();
    void on_ros_check_box_clicked();
    void on_view_ros_settings_panel_triggered();
    void on_view_ros_connection_status_triggered();
    void on_master_connect_button_clicked();
    void on_use_env_check_box_clicked(bool checked);
    void on_ip_button_clicked();
    void on_ros_dock_visibilityChanged(bool visible);

private:
    Ui::MainWindow *m_ui{nullptr};
    OSGWidget *m_osg_widget{nullptr};
    int m_argc;
    char** m_argv;
    quad::ControllerNode m_controller_node;
    quad::DroneNode m_drone_node;
    QToolBar *m_main_toolbar{nullptr};
    QProcess *m_process{nullptr};
    bool m_app_started_roscore{false};
    QIcon m_check_icon{QIcon{":myicons/check.png"}};
    QIcon m_x_icon{QIcon{":myicons/red_x.jpg"}};
    bool m_use_ros_ip{true};
};

#endif // MAINWINDOW_HPP
