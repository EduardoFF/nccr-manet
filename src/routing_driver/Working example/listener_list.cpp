// file: listener.cpp
//
// LCM example program.
//
// compile with:
//  $ gcc -o listener_list listener_list.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o listener_list listener_list.cpp `pkg-config --cflags --libs lcm`

#include <stdio.h>

#include <lcm/lcm-cpp.hpp>
#include "route2_tree_t.hpp"

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const route2_tree_t* msg)
        {
            printf("  Received message on channel \"%s\":\n", chan.c_str());
            printf("  timestamp   = %lld\n", (long long)msg->timestamp);
            printf("  rtable: \n");
            for(int i = 0; i < msg->n; i++)
	    {		
		printf("  node   = %s\n",msg->rtable[i].node.c_str());
		for(int j = 0; j < msg->rtable[i].n; j++)
		{
			printf("  dest   = %s\n", msg->rtable[i].entries[j].dest.c_str());
			printf("  node   = %s\n", msg->rtable[i].entries[j].node.c_str());
			printf("  weight   = %d\n", msg->rtable[i].entries[j].weight);
		}
	    }
            printf("\n");
        }
};

int main(int argc, char** argv)
{
    lcm::LCM lcm;

    if(!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("ROUTE", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
