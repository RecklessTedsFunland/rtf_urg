#include <cstdio>
#include "urg.hpp"
#include <string>

using namespace std;

int main() {
  printf("hello\n");

  string port = "/dev/tty.usbmodem14501";
  URG lidar;
  bool ok = lidar.open(port);
  if (!ok) {
    printf("*** Cannot open %s\n", port.c_str());
    return 1;
  }

  ok = lidar.set_laser(true);
  if (!ok) {
    printf("*** Cannot turn ON laser\n");
    return 1;
  }

  sleep(1);

  for (int i=0; i<10; ++i) {
    ok = lidar.capture();
    if (!ok) {
      printf("*** Cannot capture data\n");
      return 1;
    }

    for (int i=0; i<682; ++i) {
      if (i%10 == 0) printf("\n");
      printf("%.1f, ", lidar.ranges[i]);
    }
  }

  printf("main return 0\n");
  return 0;
}