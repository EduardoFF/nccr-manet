#include <iostream>
#include <stdlib.h>
#include <GetPot.hpp>
#include "gps_driver.h"
#include "xbee_interface.h"
#include "xbee_app_data.h"
#include "timer.h"
#include <csignal>

using namespace std;

// Function to print Help if need be
void print_help(const string Application)
{
    exit(0);
}

XbeeInterface *g_xbee;
char g_outBuf[130];
bool g_abort;

/// mutex for sending one packet at a time
pthread_mutex_t g_sendMutex;

void
signalHandler( int signum )
{
  g_abort = true;
  printf("ending app...\n");
  
  delete g_xbee;
  printf("done.\n");
  exit(signum);
}

void
receiveData(uint16_t addr, 
	    void *data, 
	    char rssi, 
	    timespec timestamp,
	    size_t len)
{
  using namespace xbee_app_data;
  cout << "Got data from " << addr
       << " rssi: " << rssi << ") "
       <<  " len: " << len << endl;
  cout << "-----------------------" << endl;
  if (len > sizeof(Header))
    {
      Header header; 
      memcpy(&header, data, sizeof(Header));
      cout << "Header: " << header << endl;
      if (header.type == XBEEDATA_ENDNODEINFO )
	{
	  if(len == sizeof(Header) + sizeof(EndNodeInfo))
	    {
	      EndNodeInfo eInfo;
	      memcpy(&eInfo,
		     (unsigned char *)data + sizeof(Header),
		     sizeof(EndNodeInfo));
	      cout << "EndNodeInfo: " << eInfo << endl;
	    }
	}
    }
  cout << "-----------------------" << endl;  
}



/////////////// Beginning of Main program //////////////////
int main(int argc, char * argv[])
{

  g_abort = false;
  /// register signal
  signal(SIGINT, signalHandler);
  
  // Simple Command line parser
  GetPot   cl(argc, argv);
  if(cl.search(2, "--help", "-h") ) print_help(cl[0]);
  cl.init_multiple_occurrence();
  const string  xbeeDev  = cl.follow("/dev/ttyUSB1", "--dev");      
  const int     baudrate    = cl.follow(57600, "--baud");
  const int     nodeId    = cl.follow(1, "--nodeid"); 
  cl.enable_loop();

  XbeeInterfaceParam xbeePar;
  xbeePar.SourceAddress = nodeId;

  xbeePar.brate = baudrate;
  xbeePar.mode  = "xbee1";
  xbeePar.Device = xbeeDev;
  xbeePar.writeParams = false;

  /// create mutexes
  if (pthread_mutex_init(&g_sendMutex, NULL) != 0)
    {
      fprintf(stderr, "mutex init failed\n");
      fflush(stderr);
      exit(-1);
    }
  

  printf("Creating xbeeInterface\n");
  fflush(stdout);
  g_xbee = new XbeeInterface(xbeePar);
  g_xbee->registerReceive(&receiveData);

  for(;;)
    {
      sleep(1);
    }

  return 0;
}
