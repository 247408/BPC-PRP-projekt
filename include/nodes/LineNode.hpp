#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

namespace nodes {
    class LineNode : public rclcpp::Node {
    public:

        LineNode();

        ~LineNode() override = default;

        // relative pose to line in meters
        float get_continuous_line_pose() const;

        DiscreteLinePose get_discrete_line_pose() const;

    private:

        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

        void on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg);

        float estimate_continuous_line_pose(float left_value, float right_value);

        DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm);
    };

}


