#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        // Constructor
        IoNode();
        // Destructor (default)
        ~IoNode() override = default;


        // Function to retireve the last pressed button value
        int get_button_pressed() const;

    private:
        // Variable to store the last received button press value
        int button_pressed_ = -1;

        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr led_publisher_;

        // Callback - preprocess received message
        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg)
        {
            button_pressed_= msg->data;
            std::cout << "button_pressed_: " << button_pressed_ << std::endl;
            //RCLCPP_INFO("Received: %f", msg->data);
        }

        void publish_message(float value_to_publish) {
                auto msg = std_msgs::msg::UInt8();
                msg.data = value_to_publish;
                led_publisher_->publish(msg);
                std::cout << "LED: " << value_to_publish << std::endl;
                //RCLCPP_INFO(node_->get_logger(), "Published: %f", msg.data);
            }
    };

}
