#include "gps_client.h"


int main()
{
  GPSDClient client;
  gps gpsCoordinates;
  while(1)
    {
      gpsCoordinates = client.getGPS();
      if ((gpsCoordinates.x != 0) | (gpsCoordinates.y != 0) | (gpsCoordinates.z != 0 ))
          printf("IN THE MAIN FUNCTION : %f %f %f \n",gpsCoordinates.x, gpsCoordinates.y, gpsCoordinates.z);

      client.getGPS1();
    }
}
