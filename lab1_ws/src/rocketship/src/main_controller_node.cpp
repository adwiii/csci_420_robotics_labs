#include <math.h>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class MainControllerNode : public rclcpp::Node
{
public:
    MainControllerNode() : Node("MainControllerNode") {
      countdown_sub = this->create_subscription<std_msgs::msg::Int64>(
      "countdown", 1000, [this](const std_msgs::msg::Int64::SharedPtr msg){ countdown_callback(msg); });
      abort_sub = this->create_subscription<std_msgs::msg::Bool>(
      "launch_abort", 1000, [this](const std_msgs::msg::Bool::SharedPtr msg){ abort_callback(msg); });
    }

    bool launch;
    bool abort_mission;

  private:
    void countdown_callback(const std_msgs::msg::Int64::SharedPtr msg) {
          // Cancel the countdown if the launch is aborted
          if (abort_mission == false) {
              RCLCPP_INFO(this->get_logger(), "Countdown: %lds", msg->data);
              // If the message is less or equal to 0
              if (msg->data <= 0) {
                  // Launch the rocket
                  this->launch = true;
              }
          }
    }
    void abort_callback(const std_msgs::msg::Bool::SharedPtr msg) {
          // If we get a true abort message
          if (msg->data == true) {
              // If we have not launched yet
              if (launch == false) {
                  // Abort the mission
                  RCLCPP_INFO(this->get_logger(), "Abort Abort Abort!!!");
                  this->abort_mission = true;
              } else {
                  // Otherwise we can not abort
                  RCLCPP_INFO(this->get_logger(), "Abort Failed");
                  this->abort_mission = false;
              }
          }
    }
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int64>> countdown_sub;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> abort_sub;

};

int main(int argc, char **argv) {
    // Initialize the ROS node and register it
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainControllerNode>();
    // Create a publisher to publish the velocity
    auto velocity_pub = node->create_publisher<std_msgs::msg::Float64>("cmd_vel", 1000);

    // Set the rate of this node to 10Hz
    rclcpp::Rate loop_rate(10);

    // Launch the rocket
    RCLCPP_INFO(node->get_logger(), "Rocket Ready for Countdown");

    // Create the velocity variable
    double velocity = 0;

    // Create a variable which will control the velocity
    float vel_counter = -5;

    // While ROS is still running
    while (rclcpp::ok()) {
        // If we have launched and havent aborted
        if ((node->launch == true) && (node->abort_mission == false)) {
            // Create the ros message
            std_msgs::msg::Float64 msg;
            // We want the velocity to follow a Hyperbolic curve (1685m/s is the typical top speed of a falcon 9 rocket)
            velocity = (tanh(vel_counter) + 1) * 842;
            msg.data = velocity;

            // Every time the robot reachs an interval of 10mph display it to terminal
            RCLCPP_INFO(node->get_logger(), "Requested Velocity: %lfm/s", msg.data);

            // Publish the velocity
            velocity_pub->publish(msg);

            // increment the velocity counter
            vel_counter += 0.01 ;
        }

        // Wait the amount of time required to maintain 10Hz
        rclcpp::spin_all(node, 0s);
        loop_rate.sleep();
    }
    return 0;
}