#include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
// #include "sensor_msgs/msg/magnetic_field.hpp"
// #include "sensor_msgs/msg/fluid_pressure.hpp"
// #include "sensor_msgs/msg/temperature.hpp"
#include <string>

#include "urg.hpp"
// #include <squaternion.hpp>
// #include <quaternion_filters.hpp>

using std::string;
using namespace std::chrono_literals;  // cpp_std s,ms,etc

constexpr string FRAME_ID = "lidar";


class rtfUrg : public rclcpp::Node {
public:
  rtfUrg(): Node("rtf_urg") {
    RCLCPP_INFO(this->get_logger(), "rtf_urg: CTRL+C to exit!");

    // Setup sensor ---------------------------------------
    string port = "/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00";
    int baud = 19200;

    bool ok = lidar.open(port, baud);
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(),"*** Cannot open %s ***\n", port.c_str());
      // return 1;
    }
    else RCLCPP_INFO(this->get_logger(), ">> open(%s,%d) START\n", port.c_str(), baud);

    ok = lidar.set_laser(true);
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(),"*** Cannot turn ON laser ***\n");
      // return 1;
    }
    else RCLCPP_INFO(this->get_logger(),">> Laser is ON\n");

    // Setup publishers -----------------------------------
    pub_urg = this->create_publisher<sensor_msgs::msg::LaserScan>("lidar", 10);

    // Setup timers for publishers ------------------------
    auto interval = std::chrono::milliseconds(1000 / 10); // 10Hz
    timer = this->create_wall_timer(interval, std::bind(&rtfUrg::cb, this));

  }

  ~rtfUrg() {
    RCLCPP_INFO(this->get_logger(), "Bye rtf_urg!");
  }

  void cb() {
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = this->now();
    msg.header.frame_id = FRAME_ID;
    msg.angle_min = URG_ANGLE_MIN;
    msg.angle_max = URG_ANGLE_MAX;
    msg.angle_increment = URG_ANGLE_INC;
    msg.time_increment = 0.1;
    msg.range_min = URG_RANGE_MIN;
    msg.range_max = URG_RANGE_MAX;

    bool ok = lidar.capture();
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(),"*** Cannot capture data\n");
      return;
    }

    // memcpy((void*)msg.ranges, lidar.ranges, 682*sizeof(float));
    // for (int i=0; i<682; ++i) msg.ranges.push_back(lidar.ranges[i]);
    msg.ranges.resize(682);
    memcpy(&msg.ranges[0], lidar.ranges, 682*sizeof(float));

    pub_urg->publish(msg);
  }

  URG lidar;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_urg;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rtfUrg>());
  rclcpp::shutdown();
  return 0;
}