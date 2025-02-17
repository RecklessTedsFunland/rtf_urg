#!/usr/bin/env python3
# MIT License Kevin Walchko (c) 2018
#
# this needs: pip install pydar

from lidar_urg import URG04LX
import time
from math import pi
import sys


if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("ERROR: No serial port given")
    print("Usage: {sys.argv[0]} <port>")
    sys.exit(1)

  a = URG04LX()
  port = sys.argv[1]
  ok = a.init(port, baudrate=19200)
  if not ok:
    print("*** Couldn't init lidar ***")
    sys.exit(1)

  print(a.pp_params)
  a.printInfo()

  theta = [i*2*pi/360 for i in range(360)]

  # plt.ion()
  for i in range(2):
    pts = a.capture()
    print('-'*40)
    # print('distance points:', pts)
    # print('timestamp:', tm)
    print('number points:', len(pts.scan))
    print(pts.scan)

  a.close()
  time.sleep(1)
