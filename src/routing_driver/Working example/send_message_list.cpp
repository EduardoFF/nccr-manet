// file: send_message.cpp
//
// LCM example program.
//
// compile with:
//  $ g++ -o send_message_list send_message_list.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ g++ -o send_message_list send_message_list.cpp `pkg-config --cflags --libs lcm`

#include <lcm/lcm-cpp.hpp>
#include "route2_tree_t.hpp"

int main(int argc, char ** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    route2_tree_t my_data;
    my_data.timestamp = 0;    
    my_data.n = 15; 
    my_data.rtable.resize(my_data.n);
    // Loop through all tables
    for(int i = 0; i < my_data.n; i++)
    {
        my_data.rtable[i].node = "test";
	my_data.rtable[i].n = 1;
	my_data.rtable[i].entries.resize(my_data.rtable[i].n);
	// Loop through all entries
    	for(int j = 0; j < my_data.rtable[i].n; j++)
	{
	    my_data.rtable[i].entries[j].dest = ("Table %d, entry %d : dest ",i,j);
	    my_data.rtable[i].entries[j].node = ("Table %d, entry %d : node ",i,j);
	    my_data.rtable[i].entries[j].weight = j;
	}
    }

    lcm.publish("ROUTE", &my_data);

    return 0;
}
