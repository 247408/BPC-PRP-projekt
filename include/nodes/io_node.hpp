#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <mutex>

namespace nodes {
     class IoNode : public rclcpp::Node {
     public:
         // Constructor
         IoNode();


         // Destructor (default)
         ~IoNode() override = default;

         // Function to retrieve the last pressed button value
         int get_button_pressed() const;

         void turn_on_leds(const std::vector<uint8_t>& values);

         void set_motor_speeds(const std::vector<uint8_t>& values);

         std::array<uint32_t, 2> get_encoder_values() const ;

     private:
         // Variable to store the last received button press value
         int button_pressed_ = -1;

         std::array<uint32_t, 2> encoder_values_ = {0, 0};

         mutable std::mutex encoder_mutex_;

         std::vector<std_msgs::msg::UInt8::SharedPtr> rgb_values_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

         // Subscriber for button press messages
         rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;

         rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr encoder_subscriber_;

         rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rgb_publisher_;

         rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_publisher_;

         // Callback - preprocess received message
         void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

         void on_encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);


     };
 }
