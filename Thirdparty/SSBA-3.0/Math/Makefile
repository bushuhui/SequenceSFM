include ../make.conf


ALL_CLFAGS += -I../COLAMD/Include -I../SuiteSparse_config

src-all := $(wildcard *.cpp)
obj-all := $(patsubst %.cpp,%.o,$(src-all))
inc-all := $(wildcard *.h)
bin-all := $(patsubst %.cpp,%.e,$(src-all))

target  := libv3d_math.a


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

