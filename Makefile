

################################################################################
# compiler settings
################################################################################
CC  = gcc
CXX = g++
MOC = moc

################################################################################
# OpenCV settings
# run following command first:
#   export PKG_CONFIG_PATH=/opt/opencv-2.4/lib/pkgconfig
################################################################################
OPENCV_CFLAGS   = $(shell pkg-config --cflags opencv)
OPENCV_LDFLAGS  = $(shell pkg-config --libs   opencv) 


################################################################################
# OpenGL settings
################################################################################
OPENGL_CFLAGS   = 
OPENGL_LDFLAGS  = -lGL -lGLU 

################################################################################
# Qt settings
################################################################################
QT_CFLAGS  = -DQT_NO_DEBUG -DQT_XML_LIB -DQT_OPENGL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED \
                -I/usr/share/qt4/mkspecs/linux-g++ \
                -I/usr/include/qt4/QtCore \
                -I/usr/include/qt4/QtGui \
                -I/usr/include/qt4/QtOpenGL \
                -I/usr/include/qt4/QtXml \
                -I/usr/include/qt4 \
                -L/usr/lib/x86_64-linux-gnu
QT_LDFLAGS = -lQtGui -lQtCore -lQtXml -lQtOpenGL -lQGLViewer-qt4


################################################################################
# V3D V3D
################################################################################
V3D_TOP     = ./Thirdparty/SSBA-3.0
V3D_CFLAGS  = -I$(V3D_TOP)
V3D_LDFLAGS = -L$(V3D_TOP) -lv3d_math -lv3d_geometry

###############################################################################
# PBA
################################################################################
LIBPBA_TOP     = ./Thirdparty/pba
LIBPBA_CFLAGS  = -I$(LIBPBA_TOP)/pba
LIBPBA_LDFLAGS = -L$(LIBPBA_TOP)/lib -lpba_no_gpu


################################################################################
# Eigen3
################################################################################
EIGEN3_TOP      = ./Thirdparty/eigen3
EIGEN3_CFLAGS   = -I$(EIGEN3_TOP)
EIGEN3_LDFLAGS  = 

################################################################################
# KLT
################################################################################
KLT_TOP      	= ./Thirdparty/klt-1.3.4
KLT_CFLAGS   	= -I$(KLT_TOP)/include
KLT_LDFLAGS  	= -L$(KLT_TOP)/lib -lklt

###############################################################################
# OpenSURF
################################################################################
LIBOPENSURF_TOP     = ./Thirdparty/OpenSURF
LIBOPENSURF_CFLAGS  = -I$(LIBOPENSURF_TOP)/src
LIBOPENSURF_LDFLAGS = -L$(LIBOPENSURF_TOP) -lOpenSURF

################################################################################
# other settings
################################################################################
OTHERS_CFLAGS   = 
OTHERS_LDFLAGS  = -lpthread

################################################################################
# Overall settings
################################################################################
SIMP_CFLAGS   	=   $(OPENCV_CFLAGS)  $(OPENGL_CFLAGS)  $(QT_CFLAGS)  \
                	$(EIGEN3_CFLAGS)  $(V3D_CFLAGS) \
                	$(KLT_CFLAGS)     $(LIBOPENSURF_CFLAGS) $(LIBPBA_CFLAGS) \
                	$(OTHERS_CFLAGS)
SIMP_LDFLAGS  	=   $(OPENCV_LDFLAGS) $(OPENGL_LDFLAGS) $(QT_LDFLAGS) \
                	$(EIGEN3_LDFLAGS) $(V3D_LDFLAGS) \
                	$(KLT_LDFLAGS)    $(LIBOPENSURF_LDFLAGS) $(LIBPBA_LDFLAGS) \
                	$(OTHERS_LDFLAGS)

# if in debug mode
#SIMP_CFLAGS  	+= -g -rdynamic

# enable SSE
SIMP_CFLAGS  	+= -msse4 
SIMP_CFLAGS  	+= -DDSFMT_MEXP=19937 -DHAVE_SSE2

# enable OpenMP
#SIMP_CFLAGS += -O3 -openmp -parallel
SIMP_CFLAGS     += -O3

# enable eigen & SSBA
SIMP_CFLAGS   	+= -DUSE_EIGEN -DHAVE_SSBA
SIMP_LDFLAGS  	+= -lcolamd 


ALL_CFLAGS    	= $(SIMP_CFLAGS)  
ALL_LDFLAGS   	= $(SIMP_LDFLAGS) 



################################################################################
# Main body
################################################################################
src-cxx-all := $(wildcard *.cpp)
src-c-all   := $(wildcard *.c)




LIB_utils	   = src/libutils.a
LIB_utils_objs = src/utils.o src/utils_lnx.o src/utils_math.o src/dSFMT.o \
				 src/utils_afm.o \
				 src/utils_gui.o src/moc_utils_gui.o src/utils_event.o

LIB_sfm 	   = src/libsfm.a
LIB_sfm_objs   = src/sfm_frame.o src/sfm_tracker.o src/sfm_ba_ssba.o src/sfm_ba_pba.o \
				 src/sfm_cameraMatrices.o src/sfm_data.o src/sfm_triangulation.o src/sfm_utils.o


bin-all     = 	SequenceSFM.e

LIBS_OBJS   =  	$(LIB_utils) $(LIB_sfm) 



all : $(bin-all) $(LIB_utils) $(LIB_sfm)

build_dir :
	make -C Thirdparty

src/libutils.a : $(LIB_utils_objs)
	ar rcs $@ $(LIB_utils_objs)

src/libsfm.a : $(LIB_sfm_objs)
	ar rcs $@ $(LIB_sfm_objs)



SequenceSFM.e : build_dir src/SequenceSFM.o $(LIBS_OBJS) 
	$(CXX) -o $@ src/SequenceSFM.o $(LIBS_OBJS) $(ALL_CFLAGS) $(ALL_LDFLAGS) 



src/moc_utils_gui.o : src/utils_gui.h
	$(MOC) src/utils_gui.h -o src/moc_utils_gui.cpp
	$(CXX) -c src/moc_utils_gui.cpp -o src/moc_utils_gui.o $(ALL_CFLAGS)


%.e:%.cpp
	$(CXX) $< -o $@ $(ALL_CFLAGS) $(ALL_LDFLAGS)

%.e:%.c
	$(CXX) $< -o $@ $(ALL_CFLAGS) $(ALL_LDFLAGS)

%.o:%.cpp
	$(CXX) -c $< -o $@ $(ALL_CFLAGS)

%.o:%.c
	$(CXX) -c $< -o $@ $(ALL_CFLAGS)


clean :
	rm -f src/*.e src/*.o src/moc_* src/*.a *.e
	make -C Thirdparty clean


