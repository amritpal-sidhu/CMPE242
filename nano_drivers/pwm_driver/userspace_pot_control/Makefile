CXX = $(CROSS_COMPILE)gcc
CXXFLAGS = -c -g -Wall -W -Werror -pedantic
LDFLAGS = 

INC = ../../ads1015_driver/
SRC = ../../ads1015_driver/ads1015.c

run_pwm_pot: nano_pwm_pot_user.o ads1015.o
	$(CXX) nano_pwm_pot_user.o ads1015.o -o run_pwm_pot $(LDFLAGS)

nano_pwm_pot_user.o: nano_pwm_pot_user.c
	$(CXX) $(CXXFLAGS) -I$(INC) nano_pwm_pot_user.c

ads1015.o: $(SRC)
	$(CXX) $(CXXFLAGS) $(INC) $(SRC)

clean:
	rm -f *o run_pwm_pot
