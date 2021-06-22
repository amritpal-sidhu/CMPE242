CXX = $(CROSS_COMPILE)gcc
CXXFLAGS = -c -g -Wall -W -Werror -pedantic
LDFLAGS = 

gps_readout: gps_readout.o pa6h.o
	$(CXX) gps_readout.o pa6h.o -o gps_readout $(LDFLAGS)

gps_readout.o: gps_readout.c pa6h.h
	$(CXX) $(CXXFLAGS) gps_readout.c

pa6h.o: pa6h.c pa6h.h
	$(CXX) $(CXXFLAGS) pa6h.c

clean:
	rm -f *o gps_readout
