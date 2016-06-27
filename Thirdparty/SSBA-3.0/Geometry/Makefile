include ../make.conf

#src-all := $(wildcard *.cpp)
src-all := v3d_metricbundle.cpp  v3d_stereobundle.cpp 

    
obj-all := $(patsubst %.cpp,%.o,$(src-all))
inc-all := $(wildcard *.h)
bin-all := $(patsubst %.cpp,%.e,$(src-all))


target  := libv3d_geometry.a


all : $(target)

$(target) : $(obj-all) $(inc-all)
	$(AR) rcs $@ $(obj-all) 
	cp $(target) ../

%.e:%.cpp $(inc-y)
	$(CXX) $< -o $@ $(ALL_CLFAGS) $(ALL_LDFLAGS) -lm

%.o:%.cpp
	$(CXX) -c $< -o $@ $(ALL_CLFAGS)

clean :
	rm -f *.e *.o *.a

