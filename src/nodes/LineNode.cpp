#include <nodes/LineNode.hpp>
#include <std_msgs/msg/detail/u_int8__struct.hpp>

namespace nodes {
    LineNode::LineNode() : Node("line_node") {
        line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>("/bpc_prp_robot/line_sensors",10,std::bind(&LineNode::on_line_sensors_msg,this,std::placeholders::_1));
    }

    void LineNode::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg) {
        RCLCPP_INFO(this->get_logger(), "Received sensor data: %f    %f",msg->data[0],msg->data[1]);
        float left_value=static_cast<float>(msg->data[0]);
        float right_value=static_cast<float>(msg->data[1]);

        float continuos_pose=estimate_continuous_line_pose(left_value, right_value);
        DiscreteLinePose discrete_pose=estimate_discrete_line_pose(left_value, right_value);

        RCLCPP_INFO(this->get_logger(),"Continuous line pose: %f",continuos_pose);
        RCLCPP_INFO(this->get_logger(),"Discrete line pose: %d",discrete_pose);
    }
    float LineNode::estimate_continuous_line_pose(float left_value, float right_value) {
        float total=left_value+right_value;
        if (total == 0) return 0.0f;
        return(right_value-left_value)/total;
    }
    DiscreteLinePose LineNode::estimate_discrete_line_pose(float l_norm, float r_norm) {
        if (l_norm>0 && r_norm>0) return DiscreteLinePose::LineBoth;
        if (l_norm>0) return DiscreteLinePose::LineOnLeft;
        if (r_norm>0) return DiscreteLinePose::LineOnRight;
        return DiscreteLinePose::LineNone;
    }
}
