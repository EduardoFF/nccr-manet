#include <string>
#include <libgpsmm.h>
#include <cmath>
#include "gps_manager.h"

int main() {

  GPSManager gpsManager;
  gps1 gpsCoordinates;

  while(1) {

    gpsCoordinates = gpsManager.getGPS1();
    if ((gpsCoordinates.x != 0) | (gpsCoordinates.y != 0) | (gpsCoordinates.z != 0 ))
        printf("IN THE MAIN FUNCTION : %f %f %f \n",gpsCoordinates.x, gpsCoordinates.y, gpsCoordinates.z);

    gpsManager.getGPS2();
  }
}
