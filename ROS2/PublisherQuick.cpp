#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("talker_node");
  auto marker_pub = node->create_publisher<std_msgs::msg::String>("talker", 1);
  std_msgs::msg::String st;
  st.data = "test";
  while (rclcpp::ok())
  {
    rclcpp::Rate loop_rate(1);
    marker_pub->publish(st);
    loop_rate.sleep();
  }
}


// You can do it via command line as well:
// ros2 topic pub -r 1 /test std_msgs/msg/String "{data: 'test'}"
