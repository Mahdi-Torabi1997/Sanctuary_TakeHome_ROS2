#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"  // Using Float64MultiArray for data transfer
#include "ForwardKinematics.h"  // Provides getEndEffectorPose()
#include "RRRManipulator.h"     // Contains the class definition for RRRManipulator
#include <memory>

using namespace std::chrono_literals;

class publisher_node : public rclcpp::Node
{
public:
  publisher_node() : Node("publisher_node")
  {
    // Declare parameters with default values.
    this->declare_parameter<double>("L1", 0.3);
    this->declare_parameter<double>("L2", 0.3);
    this->declare_parameter<double>("L3", 0.1);
    this->declare_parameter<double>("theta1", 0.5);
    this->declare_parameter<double>("theta2", 0.3);
    this->declare_parameter<double>("theta3", -0.2);

    // Create publisher and timer.
    publisher_ = this->create_publisher<example_interfaces::msg::Float64MultiArray>("ee_pose", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&publisher_node::publishFK, this));
    RCLCPP_INFO(this->get_logger(), "FK Publisher Node started");
  }

private:
  void publishFK()
  {
    // Retrieve parameters.
    double L1, L2, L3, theta1, theta2, theta3;
    this->get_parameter("L1", L1);
    this->get_parameter("L2", L2);
    this->get_parameter("L3", L3);
    this->get_parameter("theta1", theta1);
    this->get_parameter("theta2", theta2);
    this->get_parameter("theta3", theta3);

    // Create the manipulator using the parameter values.
    RRRManipulator robot(L1, L2, L3);
    robot.setJointAngles(theta1, theta2, theta3);

    double x, y, orientation;
    // Use the helper function to compute end-effector pose.
    getEndEffectorPose(robot, x, y, orientation);

    // Pack the results into a multiarray message.
    auto msg = example_interfaces::msg::Float64MultiArray();
    msg.data.push_back(x);
    msg.data.push_back(y);
    msg.data.push_back(orientation);

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published FK: x=%.3f, y=%.3f, orientation=%.3f", x, y, orientation);
  }

  rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<publisher_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
