#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

// TurtlesController node initialized with class layout
class TurtlesController: public rclcpp::Node{
    public: TurtlesController(): Node("turtles_controller"){
        
        // Subscribers to turtle poses to obtain their positions and use them in buonadry control and distance calculation
        subscription1_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
            std::bind(&TurtlesController::topic_callback1, this, _1));
        subscription2_ = this->create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10,
            std::bind(&TurtlesController::topic_callback2, this, _1));
        
        // Three publishers, two to control turtle velocities, one to publish distance between them
        publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10); 
        publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
        publisher_distance_ = this->create_publisher<std_msgs::msg::Float32>("turtles/distance", 10);

        // Timers bound to publishers, the distance publishing timer has a longer duration to avoid flooding the terminal 
        timer1_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&TurtlesController::timer_callback1, this));
        timer2_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&TurtlesController::timer_callback2, this));
        timer_distance_ = this->create_wall_timer(std::chrono::milliseconds(3000),
        std::bind(&TurtlesController::timer_distance_callback, this));
    }
    
    // Topic callbacks retrieve turtle positions without publishing them
    private: void topic_callback1(const turtlesim::msg::Pose::SharedPtr msg1){
        turtle1_x_ = msg1->x;
        turtle1_y_ = msg1->y;
    }
    private: void topic_callback2(const turtlesim::msg::Pose::SharedPtr msg2){
        turtle2_x_ = msg2->x;
        turtle2_y_ = msg2->y;
    }

    // Timer callback 1 and 2 block the respective turtle's movement if it is too close to the other turtle or to the bounds 
    private: void timer_callback1(){
        float distance = std::sqrt(std::pow(turtle2_x_ - turtle1_x_, 2) + std::pow(turtle2_y_ - turtle1_y_, 2));
        if(distance < 1.5 || (turtle1_x_ < 1.0 || turtle1_x_ > 10.0 || turtle1_y_ < 1.0 || turtle1_y_ > 10.0)){
            message_.linear.x = 0.0;
            publisher1_->publish(message_);
        }
    } 

    private: void timer_callback2(){
        float distance = std::sqrt(std::pow(turtle2_x_ - turtle1_x_, 2) + std::pow(turtle2_y_ - turtle1_y_, 2));
        if(distance < 1.5 || (turtle2_x_ < 1.0 || turtle2_x_ > 10.0 || turtle2_y_ < 1.0 || turtle2_y_ > 10.0)){
            message_.linear.x = 0.0;
            publisher2_->publish(message_);
        }
    }
    
    private: void timer_distance_callback(){
        auto message = std_msgs::msg::Float32();
        float distance = std::sqrt(std::pow(turtle2_x_ - turtle1_x_, 2) + std::pow(turtle2_y_ - turtle1_y_, 2));
        message.data = distance;
        RCLCPP_INFO(this->get_logger(), "Distance between turtles: %.2f", distance);
        publisher_distance_ -> publish(message);
    }
    
    // Initialized variables 
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription1_, subscription2_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_, publisher2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_distance_;
    rclcpp::TimerBase::SharedPtr timer1_, timer2_, timer_distance_;
    geometry_msgs::msg::Twist message_;
    float turtle1_x_, turtle1_y_, turtle2_x_, turtle2_y_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlesController>());
    rclcpp::shutdown();
    return 0;
}