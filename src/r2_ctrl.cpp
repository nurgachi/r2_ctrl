#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "robomas_package_2/msg/motor_cmd_array.hpp"
#include "robomas_package_2/msg/motor_cmd.hpp"
#include "robomas_package_2/msg/ems.hpp"

constexpr double parallel_velocity = 0.1;
constexpr double rotation_velocity = 0.1;
constexpr double rotation_leg_rotation_velocity = 0.1;
constexpr double rotation_leg_linear_velocity = 0.1;
constexpr double linear_leg_linear_velocity = 0.1;

int LF = 1;
int RF = 2;
int LR = 3;
int RR = 4;

int RLF = 5;
int RRF = 6;
int RLR = 7;
int RRR = 8;

int LLF = 9;
int LRF = 10;

int LL = 11;

double linear_leg_location = 0;

double sin45 = sqrt(2) / 2;

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

        //ベースの4輪
        if(std::abs(msg->axes[2])<0.1 && std::abs(msg->buttons[5])<0.1){
            //平行移動

            float x = msg->axes[3] * parallel_velocity;
            float y = msg->axes[4] * parallel_velocity;

            cmd.id = LF;
            cmd.mode = 1;
            cmd.value = x * 1 + y * 1;
            out.cmds.push_back(cmd);

            cmd.id = RR;
            cmd.mode = 1;
            cmd.value = x * -1 + y * -1;
            out.cmds.push_back(cmd);

            cmd.id = RF;
            cmd.mode = 1;
            cmd.value = x * 1 + y * -1;
            out.cmds.push_back(cmd);

            cmd.id = LR;
            cmd.mode = 1;
            cmd.value = x * -1 + y * 1;
            out.cmds.push_back(cmd);
        } else {
            //回転移動
            double rotation_direction = rotation_velocity * (msg->axes[2] - msg->axes[5]);

            cmd.id = LF;
            cmd.mode = 1;
            cmd.value = rotation_direction;
            out.cmds.push_back(cmd);

            cmd.id = RR;
            cmd.mode = 1;
            cmd.value = rotation_direction;
            out.cmds.push_back(cmd);

            cmd.id = RF;
            cmd.mode = 1;
            cmd.value = rotation_direction;
            out.cmds.push_back(cmd);

            cmd.id = LR;
            cmd.mode = 1;
            cmd.value = rotation_direction;
            out.cmds.push_back(cmd);
        }

        //前方の回転足の回転
        double forward_rotation_leg_rotation_direction = rotation_leg_rotation_velocity * (msg->buttons[1] - msg->buttons[0]);

        cmd.id = RLF;
        cmd.mode = 1;
        cmd.value = forward_rotation_leg_rotation_direction;
        out.cmds.push_back(cmd);

        cmd.id = RRF;
        cmd.mode = 1;
        cmd.value = forward_rotation_leg_rotation_direction * -1;
        out.cmds.push_back(cmd);

        //後方の回転足の回転
        double reverse_rotation_leg_rotation_direction = rotation_leg_rotation_velocity * (msg->buttons[3] - msg->buttons[2]);

        cmd.id = RLR;
        cmd.mode = 1;
        cmd.value = reverse_rotation_leg_rotation_direction;
        out.cmds.push_back(cmd);

        cmd.id = RRR;
        cmd.mode = 1;
        cmd.value = reverse_rotation_leg_rotation_direction * -1;
        out.cmds.push_back(cmd);

        //回転足の直進
        double rotation_leg_linear_direction = rotation_leg_linear_velocity * (msg->buttons[5] - msg->buttons[4]);

        cmd.id = LLF;
        cmd.mode = 1;
        cmd.value = rotation_leg_linear_direction;
        out.cmds.push_back(cmd);

        cmd.id = LRF;
        cmd.mode = 1;
        cmd.value = rotation_leg_linear_direction * -1;
        out.cmds.push_back(cmd);

        //直動足
        linear_leg_location += (msg->buttons[9] - msg->buttons[8]) * linear_leg_linear_velocity;
        if(linear_leg_location < 0) linear_leg_location = 0;

        cmd.id = LL;
        cmd.mode = 2;
        cmd.value = linear_leg_location;
        out.cmds.push_back(cmd);
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