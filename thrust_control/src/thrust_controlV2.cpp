#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>

#include "shared_msgs/msg/final_thrust_msg.hpp"
#include "shared_msgs/msg/thrust_command_msg.hpp"
#include "shared_msgs/msg/tools_motor_msg.hpp"
#include "shared_msgs/msg/tools_command_msg.hpp"

#include "thrust_mapping.hpp"

constexpr int MAX_CHANGE = 5;

// x trans ability is capped at 18.33 kgf-m physically
// y trans ability is capped at 10.59 kgf-m physically
// z trans ability is capped at 9.87 kgf-m physically
// x rot ability is capped at 0.59 kgf-m physically
// y rot ability is capped at 4.0 kgf-m physically
// z rot ability is capped at 5.39 kgf-m physically

constexpr int FINE = 0;
constexpr int STANDARD = 1;
constexpr int YEET = 2;
constexpr int MEGAYEET = 3;

// [x trans, y trans, z trans, x rot, y rot, z rot]
using Effort = ThrustMapper::Effort;
using Pwm = ThrustMapper::Pwm;

const Effort BASE =
    (Effort() << 1.6, 1.6, 1.6, 10.0, 0.3, 0.6).finished();

const std::array<Effort, 4> MULT = {
    1.0 * BASE,
    1.5 * BASE,
    3.0 * BASE,
    4.5 * BASE
};

class ThrustControlNode : public rclcpp::Node {
public:
    ThrustControlNode()
        : Node("thrust_control"),
          tm_(),
          power_mode_(MEGAYEET) {

        // Setup heartbeat
        // HeartbeatHelper omitted for now

        // initialize publishers
        thrust_pub_ =
            this->create_publisher<shared_msgs::msg::FinalThrustMsg>(
                "final_thrust", 10);

        tools_pub_ =
            this->create_publisher<shared_msgs::msg::ToolsMotorMsg>(
                "tools_motor", 10);

        // initialize subscribers
        command_sub_ =
            this->create_subscription<shared_msgs::msg::ThrustCommandMsg>(
                "thrust_command",
                10,
                std::bind(
                    &ThrustControlNode::PilotCommand,
                    this,
                    std::placeholders::_1));

        tools_sub_ =
            this->create_subscription<shared_msgs::msg::ToolsCommandMsg>(
                "tools_command",
                10,
                std::bind(
                    &ThrustControlNode::ToolsCommand,
                    this,
                    std::placeholders::_1));

        // initialize thrust arrays
        desired_effort_.setZero();

        desired_thrusters_.fill(127);
        desired_thrusters_unramped_.fill(127);
    }

private:
    ThrustMapper tm_;

    Effort desired_effort_;
    Pwm desired_thrusters_;
    Pwm desired_thrusters_unramped_;

    int power_mode_;

    rclcpp::Publisher<
        shared_msgs::msg::FinalThrustMsg>::SharedPtr thrust_pub_;

    rclcpp::Publisher<
        shared_msgs::msg::ToolsMotorMsg>::SharedPtr tools_pub_;

    rclcpp::Subscription<
        shared_msgs::msg::ThrustCommandMsg>::SharedPtr command_sub_;

    rclcpp::Subscription<
        shared_msgs::msg::ToolsCommandMsg>::SharedPtr tools_sub_;

    void PilotCommand(
        const shared_msgs::msg::ThrustCommandMsg::SharedPtr msg);

    void ToolsCommand(
        const shared_msgs::msg::ToolsCommandMsg::SharedPtr msg);

    void OnLoop();

    void Ramp(const Pwm& unramped_thrusters);
};

void ThrustControlNode::PilotCommand(
    const shared_msgs::msg::ThrustCommandMsg::SharedPtr msg) {

    for (int i = 0; i < 6; ++i) {
        desired_effort_(i) = msg->desired_thrust[i];
    }

    power_mode_ = msg->is_fine;

    OnLoop();
}

void ThrustControlNode::ToolsCommand(
    const shared_msgs::msg::ToolsCommandMsg::SharedPtr msg) {

    auto tools_command = msg->tools;

    for (std::size_t i = 0; i < tools_command.size(); ++i) {
        int value = static_cast<int>(tools_command[i]);
        value = std::clamp(value, 0, 255);
        tools_command[i] = value;
    }

    shared_msgs::msg::ToolsMotorMsg output_msg;
    output_msg.tools = tools_command;

    tools_pub_->publish(output_msg);
}

void ThrustControlNode::OnLoop() {

    // scale effort by multplier value
    desired_effort_ =
        desired_effort_.cwiseProduct(
            MULT[power_mode_] * 5.0);

    // calculate thrust
    desired_thrusters_unramped_ =
        tm_.GetPwm(desired_effort_);

    Ramp(desired_thrusters_unramped_);

    Pwm pwm_values = desired_thrusters_;

    // assign values to publisher messages for thurst control and status
    shared_msgs::msg::FinalThrustMsg tcm;

    tcm.thrusters = pwm_values;

    // publish data
    thrust_pub_->publish(tcm);
}

// pwm cannot change by more than MAX_CHANGE per command
void ThrustControlNode::Ramp(
    const Pwm& unramped_thrusters) {

    for (std::size_t i = 0; i < unramped_thrusters.size(); ++i) {

        // calculate the difference between the new and old thruster values
        int diff =
            static_cast<int>(unramped_thrusters[i]) -
            static_cast<int>(desired_thrusters_[i]);

        // clip the difference to +- MAX_CHANGE
        diff = std::clamp(
            diff,
            -MAX_CHANGE,
            MAX_CHANGE);

        // add the difference back into the old thruster values
        int new_value =
            static_cast<int>(desired_thrusters_[i]) + diff;

        new_value =
            std::clamp(
                new_value,
                0,
                255);

        desired_thrusters_[i] =
            static_cast<std::uint8_t>(new_value);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto thrust_control =
        std::make_shared<ThrustControlNode>();

    rclcpp::spin(thrust_control);

    rclcpp::shutdown();

    return 0;
}

