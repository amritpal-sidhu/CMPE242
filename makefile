CXX = gcc
CXXFLAGS = -c -g -Wall -W -Werror -pedantic
LDFLAGS =

run: temp1.o temp0.o
	$(CXX) $(LDFLAGS) temp1.o temp0.o -o run

temp1.o: temp1.c temp1.h temp0.h
	$(CXX) $(CXXFLAGS) temp1.c

temp0.o: temp0.c temp0.h
	$(CXX) $(CXXFLAGS) temp0.c

clean:
	rm -f *o run
