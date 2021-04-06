CXX = $(CROSS_COMPILE)gcc
CXXFLAGS = -c -g -Wall -W -Werror -pedantic
LDFLAGS = 

run_pwm: nano_pwm_user.o
	$(CXX) nano_pwm_user.o -o run_pwm $(LDFLAGS)

nano_pwm_user.o: nano_pwm_user.c
	$(CXX) $(CXXFLAGS) nano_pwm_user.c

clean:
	rm -f *o run_pwm
