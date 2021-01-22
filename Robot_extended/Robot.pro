QT       += widgets opengl xml serialport

TARGET = Robot
TEMPLATE = lib

DEFINES += ROBOT_LIBRARY

SOURCES += robot.cpp

HEADERS += robot.h\
        robot_global.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

PROJNAME = Robot
INSTTYPE = MOD

LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui
LIBS += -lOpenNI2
LIBS += -lurg_c

INCLUDEPATH += /home/rsys/SDK/RobotSDK/ModuleDev/NiTE-2.0.0/Include
INCLUDEPATH +=/usr/include/openni2
#LIBS += /home/rsys/SDK/RobotSDK/ModuleDev/NiTE-2.0.0/Redist/libNiTE2.so



include(RobotSDK_Main.pri)
