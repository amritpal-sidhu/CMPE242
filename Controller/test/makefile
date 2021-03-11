CXX = gcc
CXXFLAGS = -c -g -Wall -W -Werror -pedantic
LDFLAGS =

INC = "../"
INC_PARAMS = foreach path, $(INC), -I$(path)

run_test: test_pid.o pid.o
	$(CXX) $(LDFLAGS) test_pid.o pid.o -o run_test

test_pid.o: test_pid.c pid.h
	$(CXX) $(CXXFLAGS) $(INC_PARAMS) test_pid.c

pid.o: pid.c pid.h
	$(CXX) $(CXXFLAGS) pid.c

clean:
	rm -f *o run_test
