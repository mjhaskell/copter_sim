cmake_minimum_required(VERSION 3.4)
project(CopterSim)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(OpenSceneGraph REQUIRED COMPONENTS 
    osgDB 
    osgGA 
    osgUtil 
    osgViewer 
    osgText
    )
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    rosflight_msgs
    )

add_subdirectory(libs/dynamics)

if (NOT TARGET gtest)
    # Prevent GoogleTest from overriding our compiler/linker options
    # when building with Visual Studio
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    add_subdirectory(libs/googletest)
endif()

include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    include
    libs/dynamics/include
    )

set(MYSOURCE
    src/main.cpp
    src/mainwindow.cpp
    include/mainwindow.hpp
    src/mainwindow.ui
    include/osgwidget.hpp
    src/osgwidget.cpp
    include/droneupdatecallback.hpp
    src/droneupdatecallback.cpp
    src/dronenode.cpp
    include/dronenode.hpp
    src/controllernode.cpp
    include/controllernode.hpp
    )

set(MYRESOURCES icons/icons.qrc)

add_executable(${PROJECT_NAME}
    ${MYSOURCE}
    ${MYRESOURCES}
    )

target_link_libraries(${PROJECT_NAME}
    Dynamics
    ${OPENSCENEGRAPH_LIBRARIES}
    ${catkin_LIBRARIES}
    Qt5::Widgets
    Qt5::Gui
    )
