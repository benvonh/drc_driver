#ifndef __DRIVER_HPP__
#define __DRIVER_HPP__

#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include <unistd.h>
#include <termios.h>

#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/Int32.hpp"

#define INFO(...)  RCLCPP_INFO(get_logger(), __VA_ARGS__)
#define WARN(...)  RCLCPP_WARN(get_logger(), __VA_ARGS__)
#define ERROR(...) RCLCPP_ERROR(get_logger(), __VA_ARGS__)

using std::placeholders::_1;
using Int32 = std_msgs::msg::Int32;
using namespace std::chrono_literals;

#define SLEEP_3s rclcpp::sleep_for(3000000000ns)

template<typename T>
inline const unsigned pct_to_pwm(T x)
{
  return static_cast<unsigned>((PULSE_MAX - PULSE_MIN) * x / 100 + PULSE_MIN);
}

template<typename T>
inline const int clamp(T x)
{
  return static_cast<int>(x < 0 ? 0 : (x > 100 ? 100 : x));
}

class Driver : public rclcpp::Node
{
public:
  Driver();
  ~Driver();

private:
  void speed_cb(const Int32::SharedPtr msg);
  void steer_cb(const Int32::SharedPtr msg);
  void set_speed(unsigned pw);
  void set_steer(unsigned pw);
  void calibrate();
  void manual();

private:
  rclcpp::Subscription<Int32>::SharedPtr m_SpeedSub;
  rclcpp::Subscription<Int32>::SharedPtr m_SteerSub;
  int m_Pi;
}

#endif/*__DRIVER_HPP__*/
