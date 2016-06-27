TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG += qt

QT      += core gui opengl sql svg network xml declarative
QT      += printsupport widgets

UI_DIR       = ./build
MOC_DIR      = ./build
OBJECTS_DIR  = ./build


################################################################################
################################################################################
SOURCES += \
    ./src/utils.cpp \
    ./src/utils_lnx.cpp \
    ./src/utils_afm.cpp \
    ./src/utils_gui.cpp \
    ./src/utils_event.cpp \
    ./src/utils_math.cpp \
    ./src/dSFMT.cpp \
    ./src/sfm_frame.cpp \
    ./src/sfm_tracker.cpp \
    ./src/sfm_ba_ssba.cpp \
    ./src/sfm_ba_pba.cpp \
    ./src/sfm_triangulation.cpp \
    ./src/sfm_cameraMatrices.cpp \
    ./src/sfm_data.cpp \
    ./src/sfm_utils.cpp \
    ./src/SequenceSFM.cpp \
    ./Thirdparty/SSBA-3.0/Geometry/v3d_metricbundle.cpp \
    ./Thirdparty/SSBA-3.0/Geometry/v3d_stereobundle.cpp \
    ./Thirdparty/SSBA-3.0/SuiteSparse_config/SuiteSparse_config.c \
    ./Thirdparty/SSBA-3.0/SuiteSparse_config/xerbla/xerbla.c \
    ./Thirdparty/SSBA-3.0/Math/v3d_nonlinlsq.cpp \
    ./Thirdparty/SSBA-3.0/Math/v3d_optimization.cpp \
    ./Thirdparty/SSBA-3.0/Math/v3d_optimization_lm.cpp \
    ./Thirdparty/SSBA-3.0/Apps/bundle_varying.cpp \
    ./Thirdparty/SSBA-3.0/Apps/bundle_varying_nonlinlsq.cpp \
    ./Thirdparty/SSBA-3.0/Apps/bundle_common.cpp \
    ./Thirdparty/SSBA-3.0/COLAMD/Demo/colamd_l_example.c \
    ./Thirdparty/SSBA-3.0/COLAMD/Demo/colamd_example.c \
    ./Thirdparty/SSBA-3.0/COLAMD/Source/colamd.c \
    ./Thirdparty/SSBA-3.0/COLAMD/Source/colamd_global.c \
    ./Thirdparty/SSBA-3.0/COLAMD/MATLAB/symamdtestmex.c \
    ./Thirdparty/SSBA-3.0/COLAMD/MATLAB/colamdtestmex.c \
    ./Thirdparty/SSBA-3.0/COLAMD/MATLAB/colamdmex.c \
    ./Thirdparty/SSBA-3.0/COLAMD/MATLAB/symamdmex.c \

HEADERS += \
    ./src/utils.h \
    ./src/utils_gui.h \
    ./src/utils_event.h \
    ./src/utils_afm.h \
    ./src/utils_math.h \
    ./src/dSFMT.h \
    ./src/dSFMT-params.h \
    ./src/dSFMT-params19937.h \
    ./src/sfm_frame.h \
    ./src/sfm_tracker.h \
    ./src/sfm_ba.h \
    ./src/sfm_triangulation.h \
    ./src/sfm_cameraMatrices.h \
    ./src/sfm_data.h \
    ./src/sfm_utils.h \
    ./Thirdparty/SSBA-3.0/Base/v3d_vrmlio.h \
    ./Thirdparty/SSBA-3.0/Base/v3d_exception.h \
    ./Thirdparty/SSBA-3.0/Base/v3d_serialization.h \
    ./Thirdparty/SSBA-3.0/Geometry/v3d_mviewutilities.h \
    ./Thirdparty/SSBA-3.0/Geometry/v3d_distortion.h \
    ./Thirdparty/SSBA-3.0/Geometry/v3d_metricbundle.h \
    ./Thirdparty/SSBA-3.0/Geometry/v3d_cameramatrix.h \
    ./Thirdparty/SSBA-3.0/Geometry/v3d_stereobundle.h \
    ./Thirdparty/SSBA-3.0/SuiteSparse_config/SuiteSparse_config.h \
    ./Thirdparty/SSBA-3.0/SuiteSparse_config/xerbla/xerbla.h \
    ./Thirdparty/SSBA-3.0/Math/win32config.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_optimization.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_nonlinlsq.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_linear_lu.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_linearbase.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_linear.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_mathutilities.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_blockmatrix.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_linear_ldlt.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_ldl_private.h \
    ./Thirdparty/SSBA-3.0/Math/v3d_linear_tnt.h \
    ./Thirdparty/SSBA-3.0/COLAMD/Include/colamd.h \


OTHER_FILES = Makefile


################################################################################
################################################################################
QMAKE_CXXFLAGS += -msse4
DEFINES        += HAVE_SSBA USE_EIGEN V3DLIB_ENABLE_SUITESPARSE

INCLUDEPATH    += ./Thirdparty/eigen3 ./Thirdparty/OpenSURF/src ./Thirdparty/SSBA-3.0 \
                  ./Thirdparty/pba/pba


################################################################################
################################################################################
KLT_TOP         = ./Thirdparty/klt-1.3.4
INCLUDEPATH    += $$KLT_TOP/include
LIBS           += -L$$KLT_TOP/lib


################################################################################
################################################################################
OPENCV_TOP      = /opt/opencv-2.4.9
INCLUDEPATH    += $$OPENCV_TOP/include $$OPENCV_TOP/include/opencv
LIBS           += -L$$OPENCV_TOP/lib \
                  -lopencv_calib3d -lopencv_contrib -lopencv_core \
                  -lopencv_features2d -lopencv_flann -lopencv_gpu \
                  -lopencv_highgui -lopencv_imgproc -lopencv_legacy \
                  -lopencv_ml -lopencv_nonfree -lopencv_objdetect \
                  -lopencv_photo -lopencv_stitching -lopencv_ts \
                  -lopencv_video -lopencv_videostab


################################################################################
################################################################################
SUITESPARSE_TOP = /usr
INCLUDEPATH    += $$SUITESPARSE_TOP/include/suitesparse
LIBS           += -L/usr/lib/x86_64-linux-gnu \
                  -lcxsparse -lcsparse \
                  -lcholmod -lamd -lcolamd -lcamd -lccolamd

