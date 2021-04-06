CXX = $(CROSS_COMPILE)gcc
CXXFLAGS = -c -g -Wall -W -Werror -pedantic
LDFLAGS = 

sensor_readout: sensor_readout.o lsm303dlhc.o
	$(CXX) sensor_readout.o lsm303dlhc.o -o sensor_readout $(LDFLAGS)

sensor_readout.o: sensor_readout.c lsm303dlhc.h
	$(CXX) $(CXXFLAGS) sensor_readout.c

lsm303dlhc.o: lsm303dlhc.c lsm303dlhc.h
	$(CXX) $(CXXFLAGS) lsm303dlhc.c

clean:
	rm -f *o sensor_readout
