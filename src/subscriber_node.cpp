#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"  // Using Float64MultiArray for data transfer
#include "InverseKinematics.h"   
#include "RRRManipulator.h"      
#include <memory>

using std::placeholders::_1;

class subscriber_node : public rclcpp::Node
{
public:
  subscriber_node() : Node("subscriber_node")
  {
    subscription_ = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
      "ee_pose", 10, std::bind(&subscriber_node::poseCallback, this, _1));
    RCLCPP_INFO(this->get_logger(), "IK Subscriber Node started");
  }

private:
  void poseCallback(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_ERROR(this->get_logger(), "Received data array size (%zu) is less than expected", msg->data.size());
      return;
    }
    double x   = msg->data[0];
    double y   = msg->data[1];
    double phi = msg->data[2];
    
    RCLCPP_INFO(this->get_logger(), "Received FK: x=%.3f, y=%.3f, phi=%.3f", x, y, phi);

    // Create an instance of the manipulator
    RRRManipulator robot(0.3, 0.3, 0.1);
    double t1, t2, t3;
    // Call the free function from InverseKinematics to compute joint angles.
    if (IK::inverseKinematicsAlgebraic(robot, x, y, phi, t1, t2, t3)) {
      RCLCPP_INFO(this->get_logger(), "IK solution: theta1=%.3f, theta2=%.3f, theta3=%.3f", t1, t2, t3);
    } else {
      RCLCPP_ERROR(this->get_logger(), "No IK solution found for the given pose.");
    }
  }

  rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<subscriber_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
