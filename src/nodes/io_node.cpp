#include "nodes/io_node.hpp"

#include <nodes/io_node.hpp>
#include <rclcpp/node.hpp>

namespace nodes{
    IoNode::IoNode() : Node("io_node") {
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons", 10, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));
        encoder_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "/bpc_prp_robot/encoders", 10, [this](const std_msgs::msg::UInt32MultiArray::SharedPtr msg) { on_encoder_callback(msg); });
        rgb_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/rgb_leds", 10);
        RCLCPP_INFO(this->get_logger(), "Node setup complete");
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
        RCLCPP_INFO(this->get_logger(), "Node setup complete");
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    std::array<uint32_t, 2> IoNode::get_encoder_values() const {
        std::lock_guard<std::mutex> lock(encoder_mutex_);
        return encoder_values_;
    }

    void IoNode::turn_on_leds(const std::vector<uint8_t>& values) {
        if (values.size() != 12) {
            RCLCPP_WARN(this->get_logger(), "Wrong number of values provided");
            return;
        }
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = values;
        rgb_publisher_->publish(msg);
       // RCLCPP_INFO(this->get_logger(), "Turned on");

    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        button_pressed_ = msg->data;
        std::cout << "Button "<<button_pressed_<<" pressed" << std::endl;
        RCLCPP_INFO(this->get_logger(),"Zmacknute tlacitko: %d", button_pressed_);
    }

    void IoNode::on_encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            std::lock_guard<std::mutex> lock(encoder_mutex_);
            encoder_values_ = {msg->data[0], msg->data[1]};
            // RCLCPP_INFO(this->get_logger(), "Encoder Values: [%d, %d]", encoder_values_[0], encoder_values_[1]);
            //std::cout << "Hodnoty enkoderu "<<encoder_values_[0]<<", " <<encoder_values_[1]<< std::endl;
        } else {
            RCLCPP_WARN(this->get_logger(), "Received an invalid encoder message.");
        }
    }

    void IoNode::set_motor_speeds(const std::vector<uint8_t>& values) {
        if (values.size() != 2) {
            RCLCPP_WARN(this->get_logger(), "Wrong number of values provided");
            return;
        }
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = values;
        motor_publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Turned on");

    }
}



