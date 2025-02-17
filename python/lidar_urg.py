# The MIT License (MIT)
# Copyright (c) 2010 Yota Ichino (original author)
# Copyright (c) 2016 Kevin J. Walchko
import serial
import re
import math
import time
import copy
from collections import namedtuple
from colorama import Fore
# import atexit

Scan = namedtuple('Scan', 'scan timestamp')
ScanRange = namedtuple("ScanRange", "min max")


class ScanUnits:
  """Used to change the units the lidar reports"""
  mm = 1
  cm = 0.1
  m = 0.001




class URG04LX(serial.Serial):
  """ 
  URG SCIP 2.0
  8.3 Laser is off if the LED blinks rapidly and it is ON when LED glows continuously
  """
  angles = ScanRange(-120, 120)
  angle_res = 360/1024
  units = ScanUnits.m

  def __init__(self):
    super(serial.Serial, self).__init__()

  def __del__(self):
    self.cleanup()

  def cleanup(self):
    if self.is_open:
      self.laser(False)
      self.close()
      print("URG closed")

  def init(self, port, baudrate=19200, timeout=0.1):
    '''
    Connect to URG device
    port      : Port or device name. ex:/dev/ttyACM0, COM1, etc...
    baudrate  : Set baudrate. ex: 9600, 38400, etc...
    timeout      : Set timeout[sec]
    '''
    self.port = port
    self.baudrate = baudrate
    self.timeout = timeout
    try:
      self.open()
    except:
      print('Could not open', port, 'at', baudrate)
      return False

    self.set_scip2()

    ret = False
    while not ret:
      ret = self.get_parameter()
      print('get_parameter()')
      time.sleep(0.5)

    if not self.laser(True):
      print('Could not turn on laser')
      return False

    print(self)

    return True

  def setUnits(u):
    if not isinstance(u, ScanUnits):
      print(f"Error: setUnits tales ScanUnits but got {u}")
      return

    self.units = u

  def printInfo(self):
    if self.pp_params is None:
      print('Please connect() first')
      return

    print('========================================')
    print(self.pp_params['PROD'])
    print(self.pp_params['VEND'])
    print('----------------------------------------')
    print('  Port: {} @ {}'.format(self.port, self.baudrate))
    print('  Protocol:', self.pp_params['PROT'])
    print('  Serial Num:', self.pp_params['SERI'])
    print('  Firmware:', self.pp_params['FIRM'])
    print('----------------------------------------')
    print('  Range Min/Max [mm]: {} / {}'.format(
      self.pp_params['DMIN'],
      self.pp_params['DMAX']
    ))
    print('  Index Right/Center/Left [counts]: {} / {} / {}'.format(
      self.pp_params['AMIN'],
      self.pp_params['AFRT'],
      self.pp_params['AMAX']
    ))
    print('  Scan Time [sec]:', 60 / self.rpm)
    print('----------------------------------------')
    print('\n\n')

  def set_scip2(self):
    '''Set SCIP2.0 protcol'''
    self.flushInput()
    self.write(b'SCIP2.0\n')
    return self.readlines()

  def get_version(self):
    '''Get version information.'''
    if not self.isOpen():
      return False

    self.flushInput()
    self.write(b'VV\n')
    get = self.readlines()

    # check expected value
    if not (get[:2] == [b'VV\n', b'00P\n']):
      return False

    for item in get[2:7]:
      tmp = re.split(r':|;', item.decode('utf8'))[:2]
      self.pp_params[tmp[0]] = tmp[1]

    self.range_m = (self.pp_params['DMIN'], self.pp_params['DMAX'],)
    self.firmware = self.pp_params['FIRM']
    self.serial_num = self.pp_params['SERI']
    self.product = self.pp_params['PROD']
    self.vendor = self.pp_params['VEND']
    self.rpm = int(self.pp_params['SCAN'])

    return True

  def get_parameter(self):
    '''
    Get device parameter and set self.pp_params
    return: True/False
    '''
    # ret = self.isOpen()
    if not self.is_open:
      return False

    self.write(b'PP\n')
    time.sleep(0.1)

    get = self.readlines()

    # check expected value
    if not (get[:2] == [b'PP\n', b'00P\n']):
      return False

    # pick received data out of parameters
    self.pp_params = {}
    for item in get[2:10]:
      tmp = re.split(r':|;', item.decode('utf8'))[:2]
      self.pp_params[tmp[0]] = tmp[1]

    return self.get_version()

  def laser_on(self):
    '''Turn on the laser.'''
    if not self.isOpen():
      return False

    self.write(b'BM\n')

    get = self.readlines()

    # print('returned:', get)

    if not(get == [b'BM\n', b'00P\n', b'\n']) and not(get == [b'BM\n', b'02R\n', b'\n']):
      return False
    return True

  def laser_off(self):
    '''Turn off the laser.'''
    if not self.isOpen():
      return False

    self.flushInput()
    self.write(b'QT\n')
    get = self.readlines()

    if not(get == [b'QT\n', b'00P\n', b'\n']):
      return False
    return True

  def laser(self, val=False):
    ok = "ON" if val else "OFF"
    if val:
      print(f"{Fore.RED}")
    else:
      print(f"{Fore.GREEN}")
    print("///////////////")
    print(f"// LASER {ok:3} //")
    print("///////////////")
    print(f"{Fore.RESET}")
    if val:
      return self.laser_on()
    else:
      return self.laser_off()

  def __decode(self, encode_str):
    '''Return a numeric which converted encoded string from numeric'''
    decode = 0

    for c in encode_str:
      decode <<= 6
      decode &= ~0x3f
      decode |= c - 0x30

    return decode * self.units

  def __decode_length(self, encode_str, byte):
    '''Return leght data as list'''
    data = []
    index = 0
    # start = -135 + 44*self.angle_res
    for i in range(0, len(encode_str), byte):
      split_str = encode_str[i:i+byte]
      # data.append((start + index*self.angle_res, self.__decode(split_str),))
      data.append(self.__decode(split_str))
      index += 1

    # print("\n\nindex: {}\n\n".format(index))

    return data

  # def index2rad(self, index):
  #     '''Convert index to radian and reurun.'''
  #     rad = (2.0 * math.pi) * (index - int(self.pp_params['AFRT'])) / int(self.pp_params['ARES'])
  #     return rad

  def capture(self):
    # Receive lenght data
    # cmd = self.create_capture_command()
    cmd = 'GD' + self.pp_params['AMIN'].zfill(4) + self.pp_params['AMAX'].zfill(4) + '01\n'
    cmd = cmd.encode('utf8')
    self.flushInput()
    self.write(cmd)
    time.sleep(0.1)
    get = self.readlines()

    # checking the answer
    if not (get[:2] == [cmd, b'00P\n']):
        return [], -1

    # show packets in message
    # nbytes = 0
    # print(f"number of lines: {len(get)}")
    # for i,a in enumerate(get):
    #   print(f"  {i}[{len(a)}]: {a}")
    #   nbytes += len(a)
    # print(f"Total bytes in message: {nbytes}")

    # decode the timestamp
    tm_str = get[2][:-1]  # timestamp
    timestamp = self.__decode(tm_str)

    # decode length data
    length_byte = 0
    line_decode_str = b''
    if cmd[:2] == (b'GS' or b'MS'):
      length_byte = 2
    elif cmd[:2] == (b'GD' or b'MD'):
      length_byte = 3
    # Combine different lines which mean length data
    NUM_OF_CHECKSUM = -2
    for line in get[3:]:
      line_decode_str += line[:NUM_OF_CHECKSUM]

    # Set dummy data by begin index.
    # self.length_data = [-1 for i in range(int(self.pp_params['AMIN']))]
    # self.length_data += self.__decode_length(line_decode_str, length_byte)

    data = self.__decode_length(line_decode_str, length_byte)
    # return (self.length_data, timestamp)
    # return self.length_data
    return Scan(tuple(data), time.time())
