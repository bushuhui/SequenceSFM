
###############################################################################
# compiler settings
###############################################################################
CC=gcc
CXX=g++

###############################################################################
# OpenCV settings
# run EXPORT PKG_CONFIG_PATH=/opt/OpenCV-2.4/lib/pkgconfig
###############################################################################
OPENCV_CFLAGS=$(shell pkg-config --cflags opencv)
OPENCV_LIBS=$(shell pkg-config --libs opencv)


CFLAGS  = $(OPENCV_CFLAGS) -I../src
LDFLAGS = $(OPENCV_LIBS) -L../lib -lklt

###############################################################################
# main body
###############################################################################

src-cxx-all := $(wildcard *.cpp)
src-c-all := $(wildcard *.c)
bin-all := $(patsubst %.cpp,%.e,$(src-cxx-all)) $(patsubst %.c,%.e,$(src-c-all))


all : $(bin-all)

%.e:%.cpp
	$(CXX) $< -o $@ $(CFLAGS) $(LDFLAGS)

%.e:%.c
	$(CC) $< -o $@ $(CFLAGS) $(LDFLAGS)

%.o:%.cpp
	$(CXX) -c $< -o $@ $(CFLAGS)

%.o:%.c
	$(CC) -c $< -o $@ $(CFLAGS)

clean :
	rm -f *.e *.o

