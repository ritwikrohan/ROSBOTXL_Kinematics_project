#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/detail/float32_multi_array__struct.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <memory>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/QR>    
#include <Eigen/SVD>


using namespace std::chrono_literals;

class KinematicModel : public rclcpp::Node
{
    public:
        KinematicModel() : Node("kinematic_model")
        {   

            sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions options_sub;
            options_sub.callback_group = sub_callback_group_;
            wheel_speed_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10, std::bind(&KinematicModel::transformWheeltoTwist, this, std::placeholders::_1), options_sub);
            twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_ = this->create_wall_timer(50ms,std::bind(&KinematicModel::controlLoop, this), timer_callback_group_);
        }

    private:
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        geometry_msgs::msg::Twist velocity;
         //half of the wheel base distance
        double l = 0.170/2;
        //the radius of the wheels
        double r = 0.1/2;
        //half of track width
        double w = 0.26969/2;
        Eigen::Matrix<double, 3, 1>  result;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
        rclcpp::CallbackGroup::SharedPtr sub_callback_group_;


        void controlLoop()
        {
            this->velocity.angular.z = this->result[0];
            this->velocity.linear.x = this->result[1];
            this->velocity.linear.y = this->result[2];
            twist_pub_->publish(this->velocity);

        }
        

        void transformWheeltoTwist(const std::shared_ptr<std_msgs::msg::Float32MultiArray> msg)
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

            Eigen::Matrix<float, 4, 1> wheelVelocities;
            for (int i = 0; i < 4; ++i)
            {
                wheelVelocities(i, 0) = msg->data[i];
            }

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Matrix<double, 3, 4> H_pseudo_inverse = svd.solve(Eigen::MatrixXd::Identity(4, 4));
            this->result = H_pseudo_inverse * wheelVelocities.cast<double>();

            
            
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<KinematicModel>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);

    rclcpp::shutdown();
}

