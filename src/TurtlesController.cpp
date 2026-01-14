#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <limits>
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
        publisher_obstacles_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("obstacles", 10);

        // Timers bound to publishers, the distance publishing timer has a longer duration to avoid flooding the terminal 
        timer1_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&TurtlesController::timer_callback1, this));
        timer2_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&TurtlesController::timer_callback2, this));
        timer_distance_ = this->create_wall_timer(std::chrono::milliseconds(3000),
        std::bind(&TurtlesController::timer_distance_callback, this));
        timer_obstacles_ = this->create_wall_timer(std::chrono::milliseconds(1000),
        std::bind(&TurtlesController::publish_obstacle_distances, this));
    }

    // Obstacle positions hardcoded for simplicity
    float obst1_x_ = 2.0;
    float obst1_y_ = 2.0;
    float obst2_x_ = 8.0;
    float obst2_y_ = 8.0;
    float obst3_x_ = 5.0;
    float obst3_y_ = 5.0;

    // Sensor configuration: 16 sensors distributed in a circle around each turtle
    static constexpr int NUM_SENSORS = 16;
    float sensor_distance_ = 0.5;  // Distance from turtle center to sensor
    
    // Arrays to store sensor positions for both turtles
    std::array<geometry_msgs::msg::Point, NUM_SENSORS> turtle1_sensors_;
    std::array<geometry_msgs::msg::Point, NUM_SENSORS> turtle2_sensors_;

    // Function to compute 16 sensor positions around a turtle
    void compute_sensor_positions(float turtle_x, float turtle_y, 
                                   std::array<geometry_msgs::msg::Point, NUM_SENSORS>& sensors){
        for(int i = 0; i < NUM_SENSORS; ++i){
            // Angle for each sensor: 360/16 = 22.5 degrees apart
            float angle = i * (2.0 * M_PI / NUM_SENSORS);
            sensors[i].x = turtle_x + sensor_distance_ * std::cos(angle);
            sensors[i].y = turtle_y + sensor_distance_ * std::sin(angle);
            sensors[i].z = 0.0;
        }
    }

    // Function to compute minimum distance from each sensor to all obstacles
    // Returns array of 16 distances (one per sensor) - each is the minimum distance to any obstacle
    std::array<float, NUM_SENSORS> compute_sensor_obstacle_distances(
            const std::array<geometry_msgs::msg::Point, NUM_SENSORS>& sensors){
        std::array<float, NUM_SENSORS> distances;
        
        // Obstacle positions in array for easier iteration
        std::vector<std::pair<float, float>> obstacles = {
            {obst1_x_, obst1_y_}, {obst2_x_, obst2_y_}, {obst3_x_, obst3_y_}
        };
        
        for(int i = 0; i < NUM_SENSORS; ++i){
            float min_dist = std::numeric_limits<float>::max();
            for(const auto& obs : obstacles){
                float dist = std::sqrt(std::pow(sensors[i].x - obs.first, 2) + 
                                       std::pow(sensors[i].y - obs.second, 2));
                if(dist < min_dist){
                    min_dist = dist;
                }
            }
            distances[i] = min_dist;
        }
        return distances;
    }

    // Publish sensor-obstacle distances for both turtles
    void publish_obstacle_distances(){
        auto msg = std_msgs::msg::Float64MultiArray();
        
        // Compute distances for turtle1 (16 values) and turtle2 (16 values)
        auto t1_distances = compute_sensor_obstacle_distances(turtle1_sensors_);
        auto t2_distances = compute_sensor_obstacle_distances(turtle2_sensors_);
        
        // Format: [t1_s0, t1_s1, ..., t1_s15, t2_s0, t2_s1, ..., t2_s15]
        for(int i = 0; i < NUM_SENSORS; ++i){
            msg.data.push_back(t1_distances[i]);
        }
        for(int i = 0; i < NUM_SENSORS; ++i){
            msg.data.push_back(t2_distances[i]);
        }
        
        publisher_obstacles_->publish(msg);
    }

    
    // Topic callbacks retrieve turtle positions and compute sensor positions
    private: void topic_callback1(const turtlesim::msg::Pose::SharedPtr msg1){
        turtle1_x_ = msg1->x;
        turtle1_y_ = msg1->y;
        compute_sensor_positions(turtle1_x_, turtle1_y_, turtle1_sensors_);
    }
    private: void topic_callback2(const turtlesim::msg::Pose::SharedPtr msg2){
        turtle2_x_ = msg2->x;
        turtle2_y_ = msg2->y;
        compute_sensor_positions(turtle2_x_, turtle2_y_, turtle2_sensors_);
    }

    // Callback to extract obstacle positions as array of points
    private: void obstacle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        obstacles_.clear();
        // Data format: [x1, y1, x2, y2, x3, y3, ...]
        for(size_t i = 0; i + 1 < msg->data.size(); i += 2){
            geometry_msgs::msg::Point p;
            p.x = msg->data[i];
            p.y = msg->data[i + 1];
            p.z = 0.0;
            obstacles_.push_back(p);
        }
    }

    // Timer callback 1 and 2 block the respective turtle's movement if it is too close to the other turtle or to the bounds 
    private: void timer_callback1(){
        float distance = std::sqrt(std::pow(turtle2_x_ - turtle1_x_, 2) + std::pow(turtle2_y_ - turtle1_y_, 2));
        if(distance < 1.5 || (turtle1_x_ < 1.0 || turtle1_x_ > 10.0 || turtle1_y_ < 1.0 || turtle1_y_ > 10.0)){
            message_.linear.x = 0.0;
            publisher1_->publish(message_);
        }
        for (const auto& obs : obstacles_){
            float dist_to_obstacle = std::sqrt(std::pow(obs.x - turtle1_x_, 2) + std::pow(obs.y - turtle1_y_, 2));
            if(dist_to_obstacle < 1.0){
                message_.linear.x = 0.0;
                publisher1_->publish(message_);
                break;
            }
        }
    } 

    private: void timer_callback2(){
        float distance = std::sqrt(std::pow(turtle2_x_ - turtle1_x_, 2) + std::pow(turtle2_y_ - turtle1_y_, 2));
        if(distance < 1.5 || (turtle2_x_ < 1.0 || turtle2_x_ > 10.0 || turtle2_y_ < 1.0 || turtle2_y_ > 10.0)){
            message_.linear.x = 0.0;
            publisher2_->publish(message_);
        }
        for (const auto& obs : obstacles_){
            float dist_to_obstacle = std::sqrt(std::pow(obs.x - turtle2_x_, 2) + std::pow(obs.y - turtle2_y_, 2));
            if(dist_to_obstacle < 1.0){
                message_.linear.x = 0.0;
                publisher2_->publish(message_);
                break;
            }
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_obstacles_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_, publisher2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_distance_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_obstacles_;
    rclcpp::TimerBase::SharedPtr timer1_, timer2_, timer_distance_, timer_obstacles_;
    geometry_msgs::msg::Twist message_;
    float turtle1_x_, turtle1_y_, turtle2_x_, turtle2_y_;
    std::vector<geometry_msgs::msg::Point> obstacles_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlesController>());
    rclcpp::shutdown();
    return 0;
}