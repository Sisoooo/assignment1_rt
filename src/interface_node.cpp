#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

class TurtlesimController: public rclcpp::Node{
    public: TurtlesimController(): Node("turtlesim_controller"){
        subscription_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
            std::bind(&TurtlesimController::topic_callback, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&TurtlesimController::timer_callback, this));
    }
    private: void topic_callback(const turtlesim::msg::Pose::SharedPtr msg){ 
        RCLCPP_INFO(this->get_logger(), "Turtle Pose (x,y): x=%f, y=%f", msg->x, msg->y);
    }
    private: void timer_callback(){
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 1.0;
        message.angular.z = 0.5;
        publisher_->publish(message);
    }
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist message_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlesimController>());
    rclcpp::shutdown();
    return 0;
}