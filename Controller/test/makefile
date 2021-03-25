CXX = $(CROSS_COMPILE)gcc
CXXFLAGS = -c -g -Wall -W -Werror -pedantic
LDFLAGS = -lm

INC = ../
INC_PARAMS = $(foreach path, $(INC), -I$(path))

LIB = ../
LIB_PARAMS = $(foreach path, $(LIB), -L$(path))

run_test: test_pid.o pid.o
	$(CXX) test_pid.o pid.o -o run_test $(LDFLAGS)

test_pid.o: test_pid.c ../pid.h
	$(CXX) $(CXXFLAGS) $(INC_PARAMS) test_pid.c

pid.o: ../pid.c ../pid.h
	$(CXX) $(CXXFLAGS) ../pid.c

clean:
	rm -f *o run_test
