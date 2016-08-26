# This Makefile was tested with GNU Make
CXX=g++

# Use pkg-config to lookup the proper compiler and linker flags for LCM
CFLAGS=`pkg-config --cflags lcm`
LDFLAGS=`pkg-config --libs lcm`

msg_types=../lcm/*.lcm

all: exlcm/example_t.hpp exlcm/example_list_t.hpp exlcm/route2_tree_t.hpp \
	listener_list \
	send-message_list \
	read-log

listener-list: listener_list.o 
	$(CXX) -o $@ $^ $(LDFLAGS)

send-message-list: send_message_list.o
	$(CXX) -o $@ $^ $(LDFLAGS)

read-log: read_log.o
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp exlcm/route2_tree_t.hpp
	$(CXX) $(CFLAGS) -I. -o $@ -c $< 

exlcm/%.hpp:
	lcm-gen -x ../types/*.lcm

clean:
	rm -f listener-list send-message-list read-log
	rm -f *.o exlcm/*.o
	rm -f exlcm/*.hpp
