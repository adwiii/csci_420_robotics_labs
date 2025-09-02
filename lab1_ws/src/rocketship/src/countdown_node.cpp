#include <stdio.h>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class CountDownNode : public rclcpp::Node
{
public:
    CountDownNode() : Node("CountDownNode")
    {
    }
};


int main(int argc, char **argv) {
    // Initialize the ROS node and register it
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountDownNode>();

    // Create a publisher to publish the velocity
    auto countdown_pub = node->create_publisher<std_msgs::msg::Int64>("countdown", 1000);

    // Create a variable which will control the countdown starting at 10
    int count = 10;

    //Wait for 2 seconds before we start the countdown
    sleep(5);

    // Set the rate of this node to 1Hz
    rclcpp::Rate loop_rate(1);

    while (rclcpp::ok()) {
        // Create the ros message
        std_msgs::msg::Int64 msg;
        msg.data = count;

        // If the count is greater than 0 subtract 1
        if (count >= 0) {
            // Publish the velocity
            countdown_pub->publish(msg);
            count--;
        }

        // Wait the amount of time required to maintain 10Hz
        rclcpp::spin_all(node, 0s);
        loop_rate.sleep();
    }
}