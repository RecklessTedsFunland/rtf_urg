/*******************************************************************************\
MIT License

Copyright (c) 2019 Kevin J. Walchko

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
\********************************************************************************/
#pragma once

#include <cstdint> // uint8_t
#include <exception>
#include <string>
// #include <errno.h>     // errno
#include <fcntl.h>     // open
#include <string.h>    // memset
#include <sys/ioctl.h> // ioctl, dtr or rts
#include <termios.h>   // serial
#include <unistd.h>    // write(), read(), close()

// macos is broken!!!
// define some higher, missing data rates
#ifndef B1000000
  #define B1000000 0010010
  #define B2000000 0010013
  #define B2500000 0010014
  #define B3000000 0010015
  #define B3500000 0010016
  #define B4000000 0010017
#endif

/**
 * Serial port
 */
class SerialPort {
public:
  SerialPort() : fd(0) {}
  ~SerialPort() { this->close(); }

  // void begin(int i) {} // does nothing

  bool open(const std::string &port, int speed, int vtime=0) {

    // O_RDWR means that the port is opened for both reading and writing.
    // O_NOCTTY means that no terminal will control the process opening the
    // serial port.
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      fd = 0;
      return false;
    }

    struct termios t;
    memset(&t, 0, sizeof(t)); // clear struct for new port settings

    t.c_cflag = speed | CS8 | CLOCAL | CREAD;
    t.c_iflag = IGNPAR;
    t.c_oflag = 0;
    t.c_lflag = 0;

    // VTIME: 10th of second, 1 = 0.1 sec, time to block
    // before return
    t.c_cc[VTIME] = vtime;
    // VMIN: min number of characters before return
    t.c_cc[VMIN] = 0;

    // activate the settings for the port
    if (tcsetattr(fd, TCSANOW, &t) < 0) return false;

    return true;
  }

  void close() {
    if (fd > 0) ::close(fd);
  }

  int write(const uint8_t byte) {
    return ::write(fd, (void *)&byte, 1);
  }
  int write(const std::string &s) {
    return ::write(fd, (void *)s.c_str(), s.size());
  }
  int write(const void *buffer, int size) {
    // int ret = guard(::write(fd, buffer, size), "write(buffer): ");
    return ::write(fd, buffer, size);
  }

  int read() {
    int c;
    int num = ::read(fd, (void *)&c, 1);
    if (num <= 0) c = -1; // timeout if non-blocking
    return c;
  }

  int readBytes(uint8_t *buf, int size) {
    int num = 0;
    memset(buf, 0, size);

    while (num < size) {
      int err = ::read(fd, (void *)&buf[num], size - num);
      if (err < 0) return err;
      num += err;
    }

    return num;
  }

  bool flush_input() {
    if (tcflush(fd, TCIFLUSH) < 0) return false;
    return true;
  }

  bool flush_output() {
    if (tcflush(fd, TCOFLUSH) < 0) return false;
    return true;
  }

  bool flush() {
    if (tcflush(fd, TCIOFLUSH) < 0) return false;
    return true;
  }

  bool set_dtr(bool enabled) {
    int pin   = TIOCM_DTR;
    int value = enabled ? TIOCMBIS : TIOCMBIC;
    if (ioctl(fd, value, &pin) < 0) return false;
    return true;
  }

  bool set_rts(bool enabled) {
    int pin   = TIOCM_RTS;
    int value = enabled ? TIOCMBIS : TIOCMBIC;
    if (ioctl(fd, value, &pin) < 0) return false;
    return true;
  }

  // bool setTimeout(int time) { /* FIXME */
  //   return false;
  // }

  int available() {
    int bytes;
    // guard(ioctl(fd, FIONREAD, &bytes), "available(): ");
    if (ioctl(fd, FIONREAD, &bytes) < 0) return -1;
    return bytes;
  }

  // inline int peek() { return -1; } // FIXME: can I do thiis?
  // // int peek() {
  // //   int c = fgetc(fd);
  // //   ungetc(c, fd);
  // //   return c;
  // // }

  inline explicit operator bool() const noexcept { return bool(fd); }

  inline bool is_open() const { return (fd > 0) ? true : false; }

protected:
  int fd;
};

