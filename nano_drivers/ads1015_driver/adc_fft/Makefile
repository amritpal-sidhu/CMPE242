CXX = $(CROSS_COMPILE)gcc
CXXFLAGS = -c -g -Wall -W -Werror -pedantic
LDFLAGS = 

adc_fft_log: adc_fft_log.o ads1015.o
	$(CXX) adc_fft_log.o ads1015.o -o adc_fft_log $(LDFLAGS) -lm

adc_fft_log.o: adc_fft_log.c ../ads1015.h
	$(CXX) $(CXXFLAGS) -I../ adc_fft_log.c  -lm

ads1015.o: ../ads1015.c ../ads1015.h
	$(CXX) $(CXXFLAGS) ../ads1015.c

clean:
	rm -f *o adc_fft_log
