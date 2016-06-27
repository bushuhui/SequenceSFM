TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt


SOURCES += \
    ../src/selectGoodFeatures.cpp \
    ../src/klt_util.cpp \
    ../src/pnmio.cpp \
    ../src/storeFeatures.cpp \
    ../src/pyramid.cpp \
    ../src/convolve.cpp \
    ../src/trackFeatures.cpp \
    ../src/writeFeatures.cpp \
    ../src/error.cpp \
    ../src/klt.cpp \
    ../example/example1.cpp \
    ../example/example2.cpp \
    ../example/example3.cpp \
    ../example/example4.cpp \
    ../example/example5.cpp \
    ../example/example_ex_1.cpp


HEADERS += \
    ../src/error.h \
    ../src/klt.h \
    ../src/base.h \
    ../src/klt_util.h \
    ../src/pnmio.h \
    ../src/pyramid.h \
    ../src/convolve.h

OPENCV_TOP  = /opt/OpenCV-2.4

LIBS += -L$$OPENCV_TOP/lib \
        -lopencv_calib3d -lopencv_contrib -lopencv_core \
        -lopencv_features2d -lopencv_flann -lopencv_gpu \
        -lopencv_highgui -lopencv_imgproc -lopencv_legacy \
        -lopencv_ml -lopencv_nonfree -lopencv_objdetect \
        -lopencv_photo -lopencv_stitching -lopencv_ts \
        -lopencv_video -lopencv_videostab

INCLUDEPATH +=  $$OPENCV_TOP/include $$OPENCV_TOP/include/opencv

