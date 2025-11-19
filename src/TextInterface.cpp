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
        std::string x_input, z_input, turtle_choice;
        std::cout << "Enter linear velocity (x-axis) as float number (e.g. 1.0): " << std::endl;
        std::getline(std::cin, x_input);
        std::cout << "Enter angular velocity (z-axis) as float number (e.g. 1.0): " << std::endl;
        std::getline(std::cin, z_input);
        std::cout << "Enter turtle choice (1 or 2): " << std::endl;
        std::getline(std::cin, turtle_choice);
        if(x_input == "q" || z_input == "q" || turtle_choice == "q"){
            rclcpp::shutdown();
            break;
        }
        message.linear.x = std::stof(x_input);
        message.angular.z = std::stof(z_input);

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
                
            }
        }
        else{
            std::cout << "Invalid turtle choice. Please enter 1 or 2." << std::endl;
            continue;
        }
        rate.sleep();
    }
}