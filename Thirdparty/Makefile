################################################################################
# compiler settings
################################################################################
CC      = gcc
CXX     = g++


all : build_dir


build_dir :
	make -C klt-1.3.4/src
	make -C OpenSURF
	make -C SSBA-3.0/Math
	make -C SSBA-3.0/Geometry
	cd pba; make -f Makefile_no_gpu
	

clean :
	make -C klt-1.3.4/src clean
	make -C OpenSURF clean
	make -C SSBA-3.0/Math clean
	make -C SSBA-3.0/Geometry clean
	cd pba; make -f Makefile_no_gpu clean

