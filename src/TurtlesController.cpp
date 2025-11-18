#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

class TurtlesController: public rclcpp::Node{
    public: TurtlesController(): Node("turtles_controller"){
        subscription1_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
            std::bind(&TurtlesController::topic_callback1, this, _1));
        subscription2_ = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10,
            std::bind(&TurtlesController::topic_callback2, this, _1));
        
        publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10); 
        publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
        publisher_distance_ = this->create_publisher<std_msgs::msg::Float32>("turtles/distance", 10);

        timer1_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&TurtlesController::timer_callback1, this));
        timer2_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&TurtlesController::timer_callback2, this));
        timer_distance_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&TurtlesController::timer_distance_callback, this));
    }
    private: void topic_callback1(const turtlesim::msg::Pose::SharedPtr msg1){
        turtle1_x_ = msg1->x;
        turtle1_y_ = msg1->y;
    }
    private: void topic_callback2(const turtlesim::msg::Pose::SharedPtr msg2){
        turtle2_x_ = msg2->x;
        turtle2_y_ = msg2->y;
    }

    private: void timer_callback1(){
        // Implement movement logic for turtle1 here
    } 

    private: void timer_callback2(){
        // Implement movement logic for turtle2 here
    }
    
    private: void timer_distance_callback(){
        // Calculate and publish the distance between turtle1 and turtle2
    } 

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist message_;
    float turtle1_x_;
    float turtle1_y_;
    float turtle2_x_;
    float turtle2_y_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlesController>());
    rclcpp::shutdown();
    return 0;
}