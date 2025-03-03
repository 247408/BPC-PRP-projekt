#include "nodes/io_node.hpp"

#include "helper.hpp"


namespace nodes {
    IoNode::IoNode():Node("io_node"){

        // Initialize the subscriber
        button_subscriber_ = create_subscription<std_msgs::msg::UInt8>(Topic::buttons, 1,
            std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));

        led_publisher_ = create_publisher<std_msgs::msg::UInt8>(Topic::set_rgb_leds,1);

    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;

    }

}


