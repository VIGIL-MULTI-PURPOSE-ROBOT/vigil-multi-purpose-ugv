#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto sender = std::make_shared<rclcpp::Node>("agri_setup_sender");
  auto receiver = std::make_shared<rclcpp::Node>("agri_setup_receiver");
  auto publisher = sender->create_publisher<std_msgs::msg::UInt32>("/agri_setup/probe", 10);
  bool received = false;
  auto subscription = receiver->create_subscription<std_msgs::msg::UInt32>(
    "/agri_setup/probe", 10, [&received](std_msgs::msg::UInt32::ConstSharedPtr msg) {
      received = msg->data == 280;
    });
  auto timer = sender->create_wall_timer(std::chrono::milliseconds(100), [&publisher]() {
    std_msgs::msg::UInt32 msg;
    msg.data = 280;
    publisher->publish(msg);
  });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sender);
  executor.add_node(receiver);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (rclcpp::ok() && !received && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(100));
  }
  if (received) {
    RCLCPP_INFO(receiver->get_logger(), "PASS: ROS 2 publisher/subscriber transport received value 280");
  } else {
    RCLCPP_ERROR(receiver->get_logger(), "FAIL: no matching ROS 2 message within 10 seconds");
  }
  rclcpp::shutdown();
  return received ? 0 : 1;
}
