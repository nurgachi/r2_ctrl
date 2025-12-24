#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "robomas_package_2/msg/motor_cmd_array.hpp"
#include "robomas_package_2/msg/motor_cmd.hpp"
#include "robomas_package_2/msg/ems.hpp"

class R2Ctrl : public rclcpp::Node
{
public:
    R2Ctrl() : Node("r2_ctrl"){
        sub_joy_   = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&R2Ctrl::joy_callback, this, std::placeholders::_1));
        pub_motor_ = this->create_publisher<robomas_package_2::msg::MotorCmdArray>("motor_cmd_array", 10);
        pub_ems_   = this->create_publisher<robomas_package_2::msg::Ems>("ems_tx", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        if(msg->buttons[6]){
            // 非常停止モードへ
            robomas_package_2::msg::Ems EMS;
            EMS.ems_stop = true;
            pub_ems_->publish(EMS);
        }
        else if(msg->buttons[7]){
            // 駆動モードへ
            robomas_package_2::msg::Ems EMS;
            EMS.ems_stop = false;
            pub_ems_->publish(EMS);
        }
        
        robomas_package_2::msg::MotorCmdArray out;

        robomas_package_2::msg::MotorCmd cmd;

        // --- motor1: 右スティックX（axes[3]） ---
        cmd.id = 1;
        cmd.mode = 1;  // SPEED mode
        cmd.value = msg->axes[3] * 1000.0f;
        out.cmds.push_back(cmd);

        pub_motor_->publish(out);

        RCLCPP_INFO(
            this->get_logger(),
            "m1=%.1f  m2=%.1f  m3=%.1f",
            out.cmds[0].value
        );
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    rclcpp::Publisher<robomas_package_2::msg::MotorCmdArray>::SharedPtr pub_motor_;
    rclcpp::Publisher<robomas_package_2::msg::Ems>::SharedPtr pub_ems_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<R2Ctrl>());
    rclcpp::shutdown();
    return 0;
}