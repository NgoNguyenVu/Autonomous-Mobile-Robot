#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/timer.hpp" // <-- Thêm thư viện Timer
#include <chrono> // <-- Thêm thư viện chrono

using std::placeholders::_1;
using namespace std::chrono_literals; // <-- Thêm để dùng 'ms'

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("odom_publisher"),
        total_ticks1_(0.0), total_ticks2_(0.0), total_ticks3_(0.0), total_ticks4_(0.0),
        x_(0.0), y_(0.0), th_(0.0),
        vx_(0.0), vy_(0.0), vth_(0.0) // <-- THAY ĐỔI 1: Khởi tạo vận tốc = 0
    {
        // Tham số (Giữ nguyên)
        this->declare_parameter<double>("ticks_per_rev", 1320.0);
        this->declare_parameter<double>("wheel_radius", 0.034);
        this->declare_parameter<double>("Lx", 0.12);
        this->declare_parameter<double>("Ly", 0.105);
        this->declare_parameter<int>("encoder_max_count", 65536);
        this->declare_parameter<double>("odom_angular_scale_correction", 1.3);
        this->declare_parameter<double>("pid_sampling_time", 0.02);
        
        this->get_parameter("ticks_per_rev", ticks_per_rev_);
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("Lx", Lx_);
        this->get_parameter("Ly", Ly_);
        this->get_parameter("encoder_max_count", encoder_max_count_);
        this->get_parameter("odom_angular_scale_correction", odom_angular_scale_correction_);
        this->get_parameter("pid_sampling_time", dt_);
        
        // Subscriber (Giữ nguyên)
        enc_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "encoder_deltas", 10, std::bind(&OdomPublisher::delta_callback, this, _1));

        // Publisher (Giữ nguyên)
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // <-- THAY ĐỔI 2: Khởi tạo thời gian cuối cùng nhận delta
        last_delta_time_ = this->now();

        // <-- THAY ĐỔI 3: Tạo Timer chính 50Hz (20ms)
        // Timer này sẽ publish MỌI THỨ
        timer_ = this->create_wall_timer(
            20ms, // 50Hz
            std::bind(&OdomPublisher::publish_loop_callback, this));

        RCLCPP_INFO(this->get_logger(), "Odometry Publisher node started (Timer-based).");
    }

private:
    // <-- THAY ĐỔI 4: HÀM TIMER CHÍNH
    // Publish mọi thứ (Odom, TF, JointStates) 50 lần/giây
    void publish_loop_callback()
    {
        auto current_time = this->now();

        // 1. Kiểm tra timeout: Nếu không nhận delta 
        //    trong 0.1s (100ms), set vận tốc về 0 (cho an toàn)
        if ((current_time - last_delta_time_).seconds() > 0.1)
        {
            vx_ = 0.0;
            vy_ = 0.0;
            vth_ = 0.0;
        }

        // 2. Publish Odometry message
        // Sẽ publish ngay cả khi robot đứng yên (x,y,th = 0)
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.twist.twist.linear.x = vx_;  // Dùng biến thành viên
        odom_msg.twist.twist.linear.y = vy_;  // Dùng biến thành viên
        odom_msg.twist.twist.angular.z = vth_; // Dùng biến thành viên
        odom_pub_->publish(odom_msg);

        // 3. Publish TF transform (odom -> base_footprint)
        // Sẽ publish ngay cả khi robot đứng yên
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_footprint";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(odom_tf);

        // 4. Publish JointState
        // Sẽ publish ngay cả khi robot đứng yên
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = current_time;
        joint_msg.name = {"wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"};
        joint_msg.position = {
            (total_ticks1_ / ticks_per_rev_) * 2.0 * M_PI,
            (total_ticks2_ / ticks_per_rev_) * 2.0 * M_PI,
            (total_ticks3_ / ticks_per_rev_) * 2.0 * M_PI,
            (total_ticks4_ / ticks_per_rev_) * 2.0 * M_PI
        };
        joint_pub_->publish(joint_msg);
    }

    // <-- THAY ĐỔI 5: HÀM DELTA_CALLBACK (Đã đơn giản hóa)
    // Hàm này CHỈ cập nhật trạng thái, không publish bất cứ thứ gì
    void delta_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 4) return;

        // Cập nhật thời gian
        last_delta_time_ = this->now();
        double dt = dt_; // Dùng dt cố định

        int delta1 = msg->data[0]; // FL
        int delta2 = msg->data[1]; // FR
        int delta3 = msg->data[2]; // BL
        int delta4 = msg->data[3]; // BR

        // Cập nhật tổng ticks
        total_ticks1_ += delta1;
        total_ticks2_ += delta2;
        total_ticks3_ += delta3;
        total_ticks4_ += delta4;

        // Tính vận tốc góc (rad/s)
        double w1 = (2.0 * M_PI * delta1) / (ticks_per_rev_ * dt); // FL
        double w2 = (2.0 * M_PI * delta2) / (ticks_per_rev_ * dt); // FR
        double w3 = (2.0 * M_PI * delta3) / (ticks_per_rev_ * dt); // RL
        double w4 = (2.0 * M_PI * delta4) / (ticks_per_rev_ * dt); // RR
        
        // --- Forward Kinematics ---
        // Cập nhật các biến thành viên vx_, vy_, vth_
        vx_  = (wheel_radius_ / 4.0) * ( w1 + w2 + w3 + w4);
        vy_  = (wheel_radius_ / 4.0) * (-w1 + w2 + w3 - w4);
        vth_ = odom_angular_scale_correction_ * (wheel_radius_ / (4.0 * (Lx_ + Ly_))) * (-w1 + w2 - w3 + w4);

        // Tích hợp vị trí (cập nhật x_, y_, th_)
        x_ += (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
        y_ += (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
        th_ += vth_ * dt;

        // --- KHÔNG CÓ LỆNH PUBLISH NÀO Ở ĐÂY ---
    }

    // Các biến thành viên
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr enc_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    
    double total_ticks1_, total_ticks2_, total_ticks3_, total_ticks4_;
    double x_, y_, th_;
    double vx_, vy_, vth_; // <-- THAY ĐỔI 6: Vận tốc lưu trữ
    
    double ticks_per_rev_, wheel_radius_, Lx_, Ly_;
    int encoder_max_count_;
    double dt_;
    double odom_angular_scale_correction_;

    rclcpp::TimerBase::SharedPtr timer_; // Biến cho Timer
    rclcpp::Time last_delta_time_; // Thời gian cuối cùng nhận delta
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}