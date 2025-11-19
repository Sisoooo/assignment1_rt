#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("text_interface");
    auto publisher1_ = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    auto publisher2_ = node->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
    auto message = geometry_msgs::msg::Twist();
    while(rclcpp::ok()){
        std::cout << "Interface to control turtles, press q at any prompt to quit" << std::endl;
        std::string lx_input, ly_input, a_input, turtle_choice;
        std::cout << "Enter linear velocity (x-axis) as float number (e.g. 1.0): " << std::endl;
        std::getline(std::cin, lx_input);
        std::cout << "Enter linear velocity (y-axis) as float number (e.g. 1.0): " << std::endl;
        std::getline(std::cin, ly_input);
        std::cout << "Enter angular velocity (z-axis) as float number (e.g. 1.0): " << std::endl;
        std::getline(std::cin, a_input);
        std::cout << "Enter turtle choice (1 or 2): " << std::endl;
        std::getline(std::cin, turtle_choice);
        if(lx_input == "q" || ly_input == "q" || a_input == "q" || turtle_choice == "q"){
            rclcpp::shutdown();
            break;
        }
        message.linear.x = std::stof(lx_input);
        message.linear.y = std::stof(ly_input);
        message.angular.z = std::stof(a_input);

        rclcpp::Rate rate(10);
        auto start = std::chrono::steady_clock::now();
        const auto duration = std::chrono::seconds(1);

        if(turtle_choice == "1" || turtle_choice == "2"){
            while (std::chrono::steady_clock::now() - start < duration){
                if(turtle_choice == "1"){
                    publisher1_->publish(message);
                }
                else if(turtle_choice == "2"){
                    publisher2_->publish(message);
                }
                rclcpp::spin_some(node);
                rate.sleep();
            }
        }
        else{
            std::cout << "Invalid turtle choice. Please enter 1 or 2." << std::endl;
            continue;
        }
    }
}