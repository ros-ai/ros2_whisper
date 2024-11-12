#ifndef WHISPER_UTILS__CHRONO_UTILS_HPP_
#define WHISPER_UTILS__CHRONO_UTILS_HPP_

#include <chrono>
#include <ctime>    // std::put_time
#include <iomanip>  // std::setfill and std::setw
#include <sstream> // std::stringstream
#include "rclcpp/rclcpp.hpp" // rclcpp::Time
#include <builtin_interfaces/msg/time.hpp> // builtin_interfaces::msg::Time

namespace whisper {

// Get the nanoseconds from the current ROS clock and convert it to a chrono timestamp
inline std::chrono::system_clock::time_point ros_time_to_chrono(const rclcpp::Time &now) {
  std::chrono::nanoseconds nanoseconds(now.nanoseconds());
  return std::chrono::system_clock::time_point(nanoseconds);
};

// Helper function 
inline std::pair<int64_t, uint64_t> chrono_time_to_ros(
                const std::chrono::system_clock::time_point &timestamp) {
  std::chrono::system_clock::duration duration_since_epoch = timestamp.time_since_epoch();
  std::chrono::seconds sec = 
    std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
  std::chrono::nanoseconds nano = 
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch - sec);
  return {sec.count(), nano.count()};
};

// Convert ROS timestamp (seconds and nanoseconds) to a std::chrono::duration
inline std::chrono::system_clock::time_point ros_msg_to_chrono(
                      const builtin_interfaces::msg::Time& ros_time) {
  std::chrono::seconds sec(ros_time.sec);
  std::chrono::nanoseconds nsec(ros_time.nanosec);
  std::chrono::system_clock::duration total_duration = sec + nsec;

  std::chrono::system_clock::time_point time_point(total_duration);
  return time_point;
};

// Convert std::chrono::system_clock::time_point to a ROS timestamp (seconds and nanoseconds)
inline builtin_interfaces::msg::Time chrono_to_ros_msg(
                      const std::chrono::system_clock::time_point& timestamp) {
  auto [sec, nsec] = chrono_time_to_ros(timestamp);
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = sec;
  ros_time.nanosec = nsec;
  return ros_time;
};

// Convert the timestamp to a human-readable string:  "YY-MM-DD HH:MM:SS.mmm"
inline std::string timestamp_as_str(const std::chrono::system_clock::time_point& timestamp) {
  std::stringstream ss;
  // Convert to time_t for easier formating
  std::time_t segment_start_time = std::chrono::system_clock::to_time_t(timestamp);
  std::chrono::milliseconds ms = 
                        std::chrono::duration_cast<std::chrono::milliseconds>
                        (timestamp.time_since_epoch()) % 1000;
  ss << std::put_time(std::localtime(&segment_start_time), "%Y-%m-%d %H:%M:%S");
  ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
  return ss.str();
}

} // end of namespace whisper
#endif // WHISPER_UTILS__CHRONO_UTILS_HPP_
