#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node
{
    public:
        WheelVelocitiesPublisher() : Node("wheel_velocities_publisher")
        {
            wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
            timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_ = this->create_wall_timer(50ms,std::bind(&WheelVelocitiesPublisher::controlLoop, this), timer_callback_group_);
        }

    


    private:
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr timer_callback_group_;


        void controlLoop()
        {
            this->timer_->cancel();
            moveRobot();
        }

        void moveRobot()
        {
            RCLCPP_INFO(this->get_logger(), "Initialized wheel velocities publisher node");
            RCLCPP_INFO(this->get_logger(), "Waiting 10 seconds. Open other terminal and type 'ros2 topic echo /wheel_speed'");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            std_msgs::msg::Float32MultiArray velocities;
            RCLCPP_INFO(this->get_logger(), "Move forward");
            std::vector<float> velocitiesVector = {1.0, 1.0, 1.0, 1.0};
            velocities.data = velocitiesVector;
            wheel_speed_pub_->publish(velocities);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            RCLCPP_INFO(this->get_logger(), "Move backward");
            velocitiesVector = {-1.0, -1.0, -1.0, -1.0};
            velocities.data = velocitiesVector;
            wheel_speed_pub_->publish(velocities);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            RCLCPP_INFO(this->get_logger(), "Move left");
            velocitiesVector = {-1.0, 1.0, -1.0, 1.0};
            velocities.data = velocitiesVector;
            wheel_speed_pub_->publish(velocities);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            RCLCPP_INFO(this->get_logger(), "Move right");
            velocitiesVector = {1.0, -1.0, 1.0, -1.0};
            velocities.data = velocitiesVector;
            wheel_speed_pub_->publish(velocities);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            RCLCPP_INFO(this->get_logger(), "Turn clockwise");
            velocitiesVector = {1.0, -1.0, -1.0, 1.0};
            velocities.data = velocitiesVector;
            wheel_speed_pub_->publish(velocities);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            RCLCPP_INFO(this->get_logger(), "Turn counter-clockwise");
            velocitiesVector = {-1.0, 1.0, 1.0, -1.0};
            velocities.data = velocitiesVector;
            wheel_speed_pub_->publish(velocities);
            std::this_thread::sleep_for(std::chrono::seconds(3));

            RCLCPP_INFO(this->get_logger(), "Stop");
            velocitiesVector = {0.0, 0.0, 0.0, 0.0};
            velocities.data = velocitiesVector;
            wheel_speed_pub_->publish(velocities);
            std::this_thread::sleep_for(std::chrono::seconds(3));

        }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<WheelVelocitiesPublisher>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    rclcpp::shutdown();
}