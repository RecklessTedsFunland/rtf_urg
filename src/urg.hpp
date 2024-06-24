// URG-04LX-UG01
// | Model No. | URG-04LX-UG01 |
// |-----------|---------------|
// | Power source	| 5VDC±5%(USB Bus power)
// | Light source | Semiconductor laser diode(λ=785nm), Laser safety class 1
// | Measuring area | 20 to 5600mm(white paper with 70mm×70mm), 240°
// Accuracy	60 to 1,000mm : ±30mm,
// 1,000 to 4,095mm : ±3% of measurement
// Angular resolution	Step angle : approx. 0.36°(360°/1,024 steps)
// Scanning time	100ms/scan
// Noise	25dB or less
// Interface	USB2.0/1.1[Mini B](Full Speed)
// Command System	SCIP Ver.2.0
// Ambient illuminance*1	Halogen/mercury lamp: 10,000Lux or less, Florescent: 6000Lux(Max)
// Ambient temperature/humidity	-10 to +50 degrees C, 85% or less(No condensation, no icing)
// Vibration resistance	10 to 55Hz, double amplitude 1.5mm each 2 hour in X, Y and Z directions
// Impact resistance	196m/s2, Each 10 time in X, Y and Z directions
// Weight	Approx. 160g

/*
./urg_test.py /dev/tty.usbmodem14501
get_parameter()
URG04LX<id=0x1036d0df0, open=True>(port='/dev/tty.usbmodem14501', baudrate=19200, bytesize=8, parity='N', stopbits=1, timeout=0.1, xonxoff=False, rtscts=False, dsrdtr=False)
{'MODL': 'URG-04LX-UG01(Simple-URG)(Hokuyo Automatic Co.,Ltd.)', 'DMIN': '20', 'DMAX': '5600', 'ARES': '1024', 'AMIN': '44', 'AMAX': '725', 'AFRT': '384', 'SCAN': '600', 'VEND': 'Hokuyo Automatic Co.,Ltd.', 'PROD': 'SOKUIKI Sensor URG-04LX-UG01(Simple-URG)', 'FIRM': '3.4.03(17/Dec./2012)', 'PROT': 'SCIP 2.0', 'SERI': 'H1620274'}
========================================
SOKUIKI Sensor URG-04LX-UG01(Simple-URG)
Hokuyo Automatic Co.,Ltd.
----------------------------------------
  Port: /dev/tty.usbmodem14501 @ 19200
  Protocol: SCIP 2.0
  Serial Num: H1620274
  Firmware: 3.4.03(17/Dec./2012)
----------------------------------------
  Range Min/Max [mm]: 20 / 5600
  Index Right/Center/Left [counts]: 44 / 384 / 725
  Scan Time [sec]: 0.1
----------------------------------------
number points: 682
----------------------------------------
number points: 682
number of lines: 36
  0[13]: b'GD0044072501\n'
  1[4]: b'00P\n'
  2[6]: b'ZR3h7\n'
  3[66]: b'00600600611N0060070060060070060060060060060070060060070fa0dG0fa0S\n'
  4[66]: b'fa0fa0dG0@U0@G0@B0@B0@:0@70@40@10?`0?Y0?Y0?U0?E0>k0>k0>k0>m0>m0?F\n'
  5[66]: b'10?20?50?>0?X0?Y0?]0?d0?g0?l0?m0@10@K0C=0D<000000000000000000000:\n'
  6[66]: b'0000FS0FQ0FQ0FS0FS0FW0Fn0G90GW0GX0G[0G[0G[0GX0GM08707m07m07m07m0O\n'
  7[66]: b'7l07l07l08C08J08K08h09M09P09T09W09_09f0:L000007000007000000000000\n'
  8[66]: b'00070IH0HW0HW0Gf0G70FH0FH0FH00000000000000006c06c06c06k06k08I0?PX\n'
  9[66]: b'0?P0?P0?90>n0>\\0>P0=m0=c0=a0=J0=A0=40<[0<S0<F0<=0<00;f0;^0;M0;K0m\n'
  10[66]: b';:0;50;40:a0:W0:U0:A0:>0:<08W08W08W09R09R09R09C09?09?09:09708m08R\n'
  11[66]: b'g08^08Y08V08L08H08D08>08607o07j07`06N06J06J06J07S07S07S07S07H07Cc\n'
  12[66]: b'07@07?07707407207207206h06g06X06X06X06Z06Z06Z06Y06W06S06N06D06D0C\n'
  13[66]: b'6C06>06>06<06706706506505l05k05k05k05g05b05b05^05]05X05V05V05X058\n'
  14[66]: b'V05V05R05R05Q05J05G05G05<05<05G05G05=05@05=05905905@05805805;05:I\n'
  15[66]: b'05905905905405405405405404o04o04o05304o04n04f04n04n04f04f04m04m0Q\n'
  16[66]: b'4m04m04m04d04a04a04`04`04\\04X04U04U04U04R04W04X04\\04X04X04W04W043\n'
  17[66]: b'W04[04[04\\04[04W04V04R04R04O04R04T04T04Q04T04X04Q04X04X04Y04T04UB\n'
  18[66]: b'04U04U04T04T04R04Q04P04R04R04U04U04U04R04R04Q04R04U04U04U04U04U0l\n'
  19[66]: b'4U04W04X04W04Q04Q04Q04Q04Q04T04T04N04N04T04T04U04U04U04Q04Q04Q04f\n'
  20[66]: b'X04\\04\\04\\04\\04[04Z04Z04X04W04W04W04W04\\04]04a04a04^04a04a04^04cK\n'
  21[66]: b'04c04b04`04a04a04b04d04d04e04g04g04g04g04n04k04n04k04k04k04m04m0H\n'
  22[66]: b'4n04o04n05305405505605605905905:05:05>05?05@05@05@05E05E05J05K05@\n'
  23[66]: b'K05J05J05K05L05M05O05O05O05W05X05Y05]05b05]05b05f05f05f05f05k05n`\n'
  24[66]: b'06106106106=06=06=06>06@06D06F06F06E06F06M06N06T06Y06]06]06^06^0_\n'
  25[66]: b'6X06X06^06c06i07507907907;07=07I07K07O07O07Q07U07Y07Z07b07e07f07F\n'
  26[66]: b'l07o08708>08A08E08I08I08U08Y08a08k08l08n09109>09>09>09<09;09;098P\n'
  27[66]: b'09809609509309309009109108j08i08g08g08d08d06U05?05104m04g04e04d0>\n'
  28[66]: b'4N03@02?02>02;02;02;02;02=02=02=02I02K03=06h08708C08C08E08E08E08Q\n'
  29[66]: b'>08?08?08?08?08?08B08?08?08=08?08=08=08A08A08;08?08;08;08:088087B\n'
  30[66]: b'08608608508508508:08>08>08>08=08608608608708708;08;08;0880860810D\n'
  31[66]: b'7i07i07_07[07[05g02S01[01T01P01P01O01O01J01J01J01Q01Q01Q01N01N014\n'
  32[66]: b'N01N01R01U0000000000000001DL1DD1D51D51D51D71D<1DB1DJ1DY1D_1Dj1DmT\n'
  33[66]: b'1EE0000070000060060060060060060060000000000000000000000000000000L\n'
  34[64]: b'00000000000000000000000000000000000006000006006006000000000000h\n'
  35[1]: b'\n'
Total bytes in message: 2134
----------------------------------------
these 64B rows need to be combined into one long one
(31*64+62) / 3 = 682.0
*/

#pragma once

#include <string>
#include <vector>
#include <array>
#include <cstring>
#include "serial_port.hpp"

constexpr int URG_BUFFER_SIZE = 2200; // 2134
constexpr int URG_MSG_LEN = 682;
constexpr int URG_TMP_LEN = 682*3; // 3 char range
constexpr int URG_MIN_CNT = 44;
constexpr int URG_MID_CNT = 384;
constexpr int URG_MAX_CNT = 725;
constexpr float URG_ANGLE_INC = 360.0/1024.0;
constexpr float URG_ANGLE_MIN = (URG_MIN_CNT-URG_MID_CNT)*URG_ANGLE_INC; // neg
constexpr float URG_ANGLE_MAX = (URG_MAX_CNT-URG_MID_CNT)*URG_ANGLE_INC; // pos
constexpr float URG_RANGE_MIN = 0.2f;
constexpr float URG_RANGE_MAX = 5.6f;

using std::string;
using std::vector;
// using std::array;

// typedef array<array<uint8_t,66>,36> ScanMsg;

// sensor_msgs/LaserScan Message
// ------------------------------
// std_msgs/Header header
// float32 angle_min
// float32 angle_max
// float32 angle_increment
// float32 time_increment
// float32 scan_time
// float32 range_min
// float32 range_max
// float32[] ranges
// float32[] intensities

class URG {
  public:
  URG() {}
  ~URG() {
    // printf("~URG START\n");
    close();
    // printf("~URG FINISH\n");
  }

  bool open(const string& port, int baud=19200) {
    // printf(">> open(%s,%d) START\n", port.c_str(), baud);
    bool ok = serial.open(port, baud);
    if (!ok) return false;

    // scip2 ----------------------------------------------
    // WARNING: should just get 0\n\n but my lidar returns
    // 0Ee\n\n ... why? Since command() flushes input, maybe
    // just look for "SCIP2.0\n0" and anything after that will
    // get flushed?
    ok = command("SCIP2.0\n","SCIP2.0\n0Ee\n\n");
    if (!ok) {
      // printf("*** Failed to enter scip 2.0 mode\n");
      return false;
    }

    // get_parameter()

    // printf(">> open SUCCESS\n");

    return true;
  }

  void close() {
    if (laser_on) set_laser(false);
    serial.close();
  }

  bool set_laser(bool status) {
    if (laser_on == status) return true;

    bool ok = false;
    if (status == true) {
      ok = command("BM\n","BM\n00P\n\n","BM\n02R\n\n");
      if (ok) {
        laser_on = true;
        printf(">> Laser ON\n");
      }
      else laser_on = false;
    }
    else {
      ok = command("QT\n","QT\n00P\n\n");
      if (ok) {
        laser_on = false;
        printf(">> Laser OFF\n");
      }
      else laser_on = true;
    }

    return ok;
  }

  // NOTE: for capture(), allow to read in MORE bytes than answer expected
  bool command(const string& cmd, const string& ans, size_t len=0) {
    // printf(">> command START %s -> %s\n",cmd.c_str(), ans.c_str());
    memset(buffer,0,URG_BUFFER_SIZE);
    serial.flush_input();
    serial.write(cmd);
    // sleep(1);

    if (len == 0) len = ans.size();
    if (serial.readBytes(buffer, len) == false) {
      return false;
    }

    // buffer[len+1] = '\0';
    // printf(">> recv msg: %s\n", (char*)buffer);

    if (strncmp((char*)buffer,ans.c_str(),ans.size()) == 0) return true;
    // printf(">> command FAIL\n");
    return false;
  }

  bool command(const string& cmd, const string& ans1, const string& ans2) {
    // printf(">> command START %s -> %s\n",cmd.c_str(), ans1.c_str());
    memset(buffer,0,URG_BUFFER_SIZE);
    serial.flush_input();
    serial.write(cmd);
    if (serial.readBytes(buffer, ans1.size()) == false) return false;

    // buffer[ans1.size()+5] = '\0';
    // printf(">> recv msg: %s\n", (char*)buffer);

    if (strncmp((char*)buffer,ans1.c_str(),ans1.size()) == 0) return true;
    if (strncmp((char*)buffer,ans2.c_str(),ans2.size()) == 0) return true;

    // printf(">> command FAIL\n");

    return false;
  }

  /*
  | G | D | Starting Step | End Step | Cluster Count | String Characters | LF |
  | 0 | 0 | P | LF | Time Stamp | Sum | LF |
  | Data Block 1 (64 bytes) | Sum | LF |
  | ----------------------- | Sum | LF |
  | Data Block N (64 bytes) | Sum | LF | LF |

  */
  bool capture() {
    // char msg[25];
    // char ans[25];
    // snprintf(msg,10,"GD%04d%04d%02d\n",0,1024,1);

    // start: 44
    // end: 725
    if (command("GD0044072501\n","GD0044072501\n00P\n", 2134) == false) return false;

    if (process_buffer() == false) return false;

    return true;
  }

  // bool process_buffer() {
  //   // first pass, consilidates 64B chuncks into one
  //   // array for processing
  //   int p=23;
  //   int n=0;
  //   for (int pkts=0; pkts<32; pkts++) {
  //     for (int i=0; i<64; i++) {
  //       uint8_t c = buffer[p+i];
  //       if (c == (uint8_t)'\n') break;
  //       tmp_buf[n++] = c;
  //     }
  //     p += 66;
  //   }

  //   // now process the 682*3B array into intensities
  //   // printf("--------------------------\n");
  //   p = 0;
  //   // int line = 0;
  //   for (int i=0; i<682; ++i) {
  //     // if (i%10 == 0) printf("\n[%d]: ", line++);
  //     ranges[i] = decode_3(&tmp_buf[p]);
  //     p += 3;
  //     // printf("%.1f,",intensities[i]);
  //   }
  //   // printf("\n--------------------------\n");

  //   return true;
  // }

  bool process_buffer() {
    // first pass, consilidates 64B chuncks into one
    // array for processing
    int p=23;
    // int n=0;
    // for (int pkts=0; pkts<32; pkts++) {
    //   for (int i=0; i<64; i++) {
    //     uint8_t c = buffer[p+i];
    //     if (c == (uint8_t)'\n') break;
    //     tmp_buf[n++] = c;
    //   }
    //   p += 66;
    // }
    int tp = 0;
    int ti = 0;
    uint8_t tmp_buf[3];
    for (int pkts=0; pkts<32; pkts++) {
      for (int i=0; i<64; i++) {
        uint8_t c = buffer[p+i];
        if (c == (uint8_t)'\n') break;
        tmp_buf[tp++] = c;
        if (tp == 3) {
          tp = 0;
          ranges[ti++] = decode_3(tmp_buf);
        }
      }
      p += 66;
    }

    // // now process the 682*3B array into intensities
    // // printf("--------------------------\n");
    // p = 0;
    // // int line = 0;
    // for (int i=0; i<682; ++i) {
    //   // if (i%10 == 0) printf("\n[%d]: ", line++);
    //   ranges[i] = decode_3(&tmp_buf[p]);
    //   p += 3;
    //   // printf("%.1f,",intensities[i]);
    // }
    // // printf("\n--------------------------\n");

    return true;
  }

  // float ranges[URG_MSG_LEN]{0.0f};
  std::array<float,URG_MSG_LEN> ranges;
  // float intensities[URG_MSG_LEN]{0.0f};
  bool laser_on{false};

protected:
  SerialPort serial;
  uint8_t buffer[URG_BUFFER_SIZE]{0};
  // uint8_t tmp_buf[URG_TMP_LEN]{0};

  // 3 char code -> distance [m]
  // URG returns range in mm, so divide by 1000
  // to get meters.
  float decode_3(uint8_t *encode_str) {
    int decode{0};

    for (size_t i=0; i<3; i++) {
      uint8_t c = encode_str[i];
      decode <<= 6;
      decode &= ~0x3f;
      decode |= c - 0x30;
    }

    return static_cast<float>(decode) / 1000.0;
  }

};

