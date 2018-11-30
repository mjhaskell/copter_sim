#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include "dronenode.hpp"
#include <QProcess>

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc,char** argv,QWidget *parent = nullptr);
    ~MainWindow();

protected:
    bool rosIsConnected();
    void updateRosStatus();
    void startRosCore();
    void createToolbar();
    void setupStatusBar();
    QAction* createStartAction();
    void startSimulation();
    QAction *createRoscoreAction();

private:
    Ui::MainWindow *m_ui;
    int m_argc;
    char** m_argv;
    quad::DroneNode m_drone_node;
    QToolBar *m_main_toolbar{nullptr};
    QProcess *m_process{nullptr};
    bool m_app_started_roscore{false};
    QIcon m_check_icon{QIcon{":myicons/check.png"}};
    QIcon m_x_icon{QIcon{":myicons/red_x.jpg"}};
};

#endif // MAINWINDOW_HPP
