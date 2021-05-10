CXX = $(CROSS_COMPILE)gcc
CXXFLAGS = -c -g -Wall -W -Werror -pedantic
LDFLAGS = 

adc_readout: adc_readout.o ads1015.o
	$(CXX) adc_readout.o ads1015.o -o adc_readout $(LDFLAGS)

adc_readout.o: adc_readout.c ads1015.h
	$(CXX) $(CXXFLAGS) adc_readout.c

ads1015.o: ads1015.c ads1015.h
	$(CXX) $(CXXFLAGS) ads1015.c

clean:
	rm -f *o adc_readout
