#-------------------------------------------------
#
# Project created by QtCreator 2015-11-23T10:49:22
#
#-------------------------------------------------

QT       += core widgets serialport

QT       -= gui

TARGET = Kinect2SLAM
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

HEADERS += adafruit_uart.h


QMAKE_CFLAGS_RELEASE += -fopenmp
QMAKE_CFLAGS_DEBUG += -fopenmp
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS +=  -fopenmp


################
# Libfreenect2
###############

LIBS += -L/usr/local/lib -lfreenect2

INCLUDEPATH += /usr/local/include
DEPENDPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/libfreenect2/tinythread
DEPENDPATH += /usr/local/include/libfreenect2/tinythread

#################
# LIB PCL
#################
# find lib of PCL 1.8
unix: LIBS += -L/usr/local/lib/ \
-lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_keypoints -lpcl_ml \
-lpcl_octree -lpcl_outofcore -lpcl_people -lpcl_recognition -lpcl_registration -lpcl_sample_consensus \
-lpcl_search -lpcl_segmentation -lpcl_stereo -lpcl_surface -lpcl_tracking -lpcl_visualization \
-lpcl_gpu_containers -lpcl_gpu_features -lpcl_gpu_utils -lpcl_cuda_features -lpcl_gpu_octree -lpcl_gpu_segmentation #\
#-lpcl_gpu_surface -lpcl_gpu_tracking -lpcl_gpu_kinfu -lpcl_gpu_kinfu_large_scale -lpcl_cuda_io

# path dei file header (.h) di PCL
unix:INCLUDEPATH += /usr/local/include/pcl-1.8
unix:INCLUDEPATH += /usr/local/include/pcl-1.8





#################
# LIB Eigen
#################
# Eigen è una libreria "header only" cioè non ci sono .lib, tutto è scritto nel file header
# path dei file header (.h) di Eigen
unix:INCLUDEPATH += /usr/include/eigen3
unix:DEPENDPATH += /usr/include/eigen3



#################
# LIB FLANN
#################
LIBS += -L/usr/lib/ -lflann_cpp
INCLUDEPATH += /usr/include/flann
DEPENDPATH += /usr/include/flann





#################
# LIB Boost
#################
# find lib of Boost
unix: LIBS += -L/usr/lib/ -lboost_thread -lboost_system

# path dei file header (.h) di Boost
unix:INCLUDEPATH += /usr/include/boost
unix:DEPENDPATH += /usr/include/boost








#################
# LIB VTK
#################
# find lib of VTK 6.1 for release AND debug
unix: LIBS += -L/usr/lib/ \
-lvtkCommonDataModel-6.3 \
-lvtkCommonMath-6.3 \
-lvtkCommonCore-6.3 \
-lvtkGUISupportQt-6.3 \
-lvtkRenderingCore-6.3 \
-lvtkRenderingLOD-6.3

# path dei file header (.h) di VTK
unix:INCLUDEPATH += /usr/local/include/vtk-6.3
unix:DEPENDPATH += /usr/local/include/vtk-6.3

