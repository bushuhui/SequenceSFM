TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt


SOURCES += \
    ./src/fasthessian.cpp \
    ./src/surf.cpp \
    ./src/main.cpp \
    ./src/ipoint.cpp \
    ./src/utils.cpp \
    ./src/integral.cpp \

HEADERS += \
    ./src/utils.h \
    ./src/integral.h \
    ./src/kmeans.h \
    ./src/surflib.h \
    ./src/surf.h \
    ./src/responselayer.h \
    ./src/fasthessian.h \
    ./src/ipoint.h \


OPENCV_TOP  = /opt/opencv-2.4.5

LIBS += -L$$OPENCV_TOP/lib \
        -lopencv_calib3d -lopencv_contrib -lopencv_core \
        -lopencv_features2d -lopencv_flann -lopencv_gpu \
        -lopencv_highgui -lopencv_imgproc -lopencv_legacy \
        -lopencv_ml -lopencv_nonfree -lopencv_objdetect \
        -lopencv_photo -lopencv_stitching -lopencv_ts \
        -lopencv_video -lopencv_videostab

INCLUDEPATH +=  $$OPENCV_TOP/include $$OPENCV_TOP/include/opencv

