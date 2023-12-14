#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/QR>    
#include <Eigen/SVD>
#include <cmath>


using namespace std::chrono_literals;

class EightTrajectory : public rclcpp::Node
{
    public:
        EightTrajectory() : Node("eight_trajectory")
        {   

            odom_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options_odom;
            options_odom.callback_group = odom_callback_group_;
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&EightTrajectory::odomCallback, this, std::placeholders::_1), options_odom);
            wheel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
            timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_ = this->create_wall_timer(50ms,std::bind(&EightTrajectory::controlLoop, this), timer_callback_group_);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_pub_;
        geometry_msgs::msg::Twist velocity;
         //half of the wheel base distance
        double l = 0.170/2;
        //the radius of the wheels
        double r = 0.1/2;
        //half of track width
        double w = 0.26969/2;
        Eigen::Matrix<double, 3, 1>  twist;
        Eigen::Matrix<double, 4, 1>  wheel_velocity;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
        rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
        double roll;
        double pitch;
        double yaw;


        void odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)            
        {

            tf2::Quaternion q;
            tf2::fromMsg(msg->pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            m.getRPY(this->roll, this->pitch, this->yaw);
        }

        void controlLoop()
        {
            this->timer_->cancel();
            move_robot();

        }

        void move_robot()
        {
            Eigen::Matrix<double, 3, 1> w1;
            w1 << 0.0, 1, -1;
            Eigen::Matrix<double, 3, 1> w2;
            w2 << 0.0, 1, 1;
            Eigen::Matrix<double, 3, 1> w3;
            w3 << 0.0, 1, 1;
            Eigen::Matrix<double, 3, 1> w4;
            w4 << 1.5708, 1, -1;
            Eigen::Matrix<double, 3, 1> w5;
            w5 << -3.1415, -1, -1;
            Eigen::Matrix<double, 3, 1> w6;
            w6 << 0.0, -1, 1;
            Eigen::Matrix<double, 3, 1> w7;
            w7 << 0.0, -1, 1;
            Eigen::Matrix<double, 3, 1> w8;
            w8 << 0.0, -1, -1;
            Eigen::Matrix<double, 3, 1> w9;
            w9 << 0.0, 0.0, 0.0; 

            std::vector<Eigen::Matrix<double, 3, 1>> waypoints = {w1, w2, w3, w4, w5, w6, w7, w8, w9};
            std_msgs::msg::Float32MultiArray wheel_msg;
            int x = 1;
            for (auto waypoint : waypoints) 
            {   
                
                if (x == 9) {
                RCLCPP_INFO(get_logger(), "Going back to waypoint: W%d", 0);
                } else {
                    RCLCPP_INFO(get_logger(), "Going to waypoint: W%d", x);
                }
                // RCLCPP_INFO(this->get_logger(), "Going to waypoint: W%d", x);
                float speed_reducer = 3.0;
                waypoint /= speed_reducer;

                for (size_t i = 0; i < 120; i++) 
                {
                    this->transformVelocitytoTwist(waypoint);
                    this->transformTwisttoWheels(this->twist);

                    std::vector<float> vector_data(this->wheel_velocity.data(), this->wheel_velocity.data() + this->wheel_velocity.size());
                    
                    wheel_msg.data = vector_data;

                    this->wheel_pub_->publish(wheel_msg);
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }
                wheel_msg.data = {0.0, 0.0, 0.0, 0.0};
                this->wheel_pub_->publish(wheel_msg);
                rclcpp::sleep_for(std::chrono::milliseconds(250));
                x++;
            }
                RCLCPP_INFO(this->get_logger(), "Robot completed eight trajectory");
                rclcpp::shutdown();
        }
        

        void transformVelocitytoTwist(Eigen::Matrix<double, 3, 1> waypoint)
        {
            Eigen::Matrix<double, 3, 3> R;

            R(0, 0) = 1.0;
            R(0, 1) = 0.0;
            R(0, 2) = 0.0;
            R(1, 0) = 0.0;
            R(1, 1) = cos(this->yaw);
            R(1, 2) = sin(this->yaw);
            R(2, 0) = 0.0;
            R(2, 1) = -sin(this->yaw);
            R(2, 2) = cos(this->yaw);

            this->twist = R * waypoint;
            
        }

        void transformTwisttoWheels(Eigen::Matrix<double, 3, 1> t)
        {
            Eigen::Matrix<double, 4, 3> H;

            H(0, 0) = (-this->l - this->w) / this->r;
            H(0, 1) = 1 / this->r;
            H(0, 2) = -1 / this->r;

            H(1, 0) = (this->l + this->w) / this->r;
            H(1, 1) = 1 / this->r;
            H(1, 2) = 1 / this->r;

            H(2, 0) = (this->l + this->w) / this->r;
            H(2, 1) = 1 / this->r;
            H(2, 2) = -1 / this->r;

            H(3, 0) = (-this->l - this->w) / this->r;
            H(3, 1) = 1 / this->r;
            H(3, 2) = 1 / this->r;

            this->wheel_velocity = H * t;
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<EightTrajectory>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    rclcpp::shutdown();
}

