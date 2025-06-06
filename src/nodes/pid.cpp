#include "nodes/pid.hpp"
#include <algorithm>
#include <cmath>
#include "std_msgs/msg/int32.hpp"

namespace nodes {

    constexpr float PI = 3.14159265f;

    PidNode::PidNode(std::shared_ptr<ImuNode> imu_node)
        : Node("pid_node"),
          Kp(6.0f), Kd(4.0f), Ki(0), // Ki se nepouziva
          base_speed(140.0f),
          last_error(0.0f), integral_(0.0f),
          imu_node_(imu_node),
          state_(DriveState::DRIVE_FORWARD),
          turn_direction_(0),
          turn_start_yaw_(0.0f)
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 10,
            std::bind(&PidNode::scan_callback, this, std::placeholders::_1)
        );

        aruco_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/bpc_prp_robot/tag_detected", 10,
        std::bind(&PidNode::aruco_callback, this, std::placeholders::_1)
    );

        motor_controller_ = std::make_shared<MotorController>();
    }
    rclcpp::Time drive_forward_start_time_;
    float drive_forward_duration_threshold_ = 1.5f; // čas v sekundách po kterém resetujeme just_turned

    float round_to_rad90(float angle_rad) {
        int quadrant = static_cast<int>(std::round(angle_rad / M_PI_2));
        return quadrant * M_PI_2;
    }

void PidNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    LidarFiltr filter;
    auto result = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max, msg->range_min, msg->range_max);

    float front = result.front;
    float back = result.back;

    float front_left = result.front_left;
    float front_right = result.front_right;
    float back_left = result.back_left;
    float back_right = result.back_right;
    float right_side = result.right_side;
    float left_side = result.left_side;

    float left_side_follow_front = result.left_side_follow_front;
    float left_side_follow_back = result.left_side_follow_back;
    float right_side_follow_front = result.right_side_follow_front;
    float right_side_follow_back = result.right_side_follow_back;
    float ffront_left = result.ffront_left;
        float ffront_right = result.ffront_right;

    float side_threshold=0.20f;
    float front_side_threshold = 0.35f;
    float front_side_RT_threshold = 0.18f;
    float left_motor_speed=0, right_motor_speed=0;
    float front_treshold = 0.25f;
    bool full_turn = false;
    static bool waiting_before_turn = false;

    if (last_aruco_id_ != -1) {
        RCLCPP_INFO(this->get_logger(), "Detected ArUco ID: %d", last_aruco_id_);
    }

    if (state_ == DriveState::DRIVE_FORWARD)
    {
        // přešel jsem do stavu drive_forward začínám zaznamenávat čas
        static bool first_time_in_drive = true;
        if (first_time_in_drive) {
            drive_forward_start_time_ = this->now();
            first_time_in_drive = false;
        }

        // zkontroluju jestli uplynul čas na reset stavových proměnných
        auto current_time = this->now();
        auto time_in_drive_forward = (current_time - drive_forward_start_time_).seconds();

        if (time_in_drive_forward > drive_forward_duration_threshold_) {
            just_turned = false;
            just_turned_left = false;
            just_turned_right = false;
            go_straight = false;
            RCLCPP_INFO(this->get_logger(), "Reset just_turned after %.2f seconds", time_in_drive_forward);
        }

        static rclcpp::Time wait_start_time;

        if (front < front_treshold) {
            // Překážka vpředu zastavím
            motor_controller_->set_motor_speeds({128.0f, 128.0f});

            // Změním stav na turning, vezmu data z imu
            state_ = DriveState::TURNING;
            turn_start_yaw_ = imu_node_->getIntegratedResults();
            first_time_in_drive = true; // Reset pro následující DRIVE_FORWARD
            waiting_before_turn = false;
            go_straight = false;

            if (back_right > front_side_threshold && (back_left < front_side_threshold)) {
                turn_direction_ = 1;
                just_turned_left=false;
                just_turned = true;
                just_turned_right = true;
                RCLCPP_INFO(this->get_logger(), "Překážka vpředu – otáčím o 90° doprava");
            }
            else if (back_right < front_side_threshold && (back_left > front_side_threshold)) {
                turn_direction_ = -1;
                just_turned_left=true;
                just_turned = true;
                just_turned_right = false;
                RCLCPP_INFO(this->get_logger(), "Překážka vpředu – otáčím o 90° doleva");
            }
            else if ((back_left < front_side_threshold) && (back_right < front_side_threshold)) {
                turn_direction_ = 2;
                full_turn = true;
                RCLCPP_INFO(this->get_logger(), "Otáčím o 180°");
            }
            else if ((back_right > front_side_threshold) && (back_left > front_side_threshold) && (last_aruco_id_ == 1 || last_aruco_id_ == 11)) {
                turn_direction_ = -1;
                just_turned_left=true;
                just_turned = true;
                just_turned_right = false;
                last_aruco_id_ = -1;
                RCLCPP_INFO(this->get_logger(), "Prekážka vpredu – začínam otáčať o 90° do lavej strany");
            }
            else if ((back_right > front_side_threshold) && (back_left > front_side_threshold) && (last_aruco_id_ == 2 ||last_aruco_id_ == 12)) {
                turn_direction_ = 1;
                just_turned_left=false;
                just_turned = true;
                just_turned_right = true;
                last_aruco_id_ = -1;
                RCLCPP_INFO(this->get_logger(), "Prekážka vpredu – začínam otáčať o 90° do prave strany");
            }
            else {
                turn_direction_ = 1;
                just_turned_right=false;
                just_turned = true;
                just_turned_left = true;
                RCLCPP_INFO(this->get_logger(), "Prekážka vpredu – začínam otáčať o 90° do leve strany");
            }
            return;
        }
        else
            {
            //točení podle aruco tagů
            if ((left_side > front_side_threshold) && (front> front_side_threshold+0.2f) &&!just_turned && !waiting_before_turn) {
                if (last_aruco_id_ == 0 || last_aruco_id_ == 10) {
                    RCLCPP_INFO(this->get_logger(),
                        "Ignoruju levou odbočku – poslední ArUco ID %d říká jeď rovně.", last_aruco_id_);
                    last_aruco_id_=-1;
                    return;
                }
                if ((last_aruco_id_ == 1 || last_aruco_id_ == 11) && back>front_side_threshold+0.1f) {
                    wait_start_time = this->now();
                    waiting_before_turn = true;
                    turn_direction_ = -1;
                    last_aruco_id_=-1;
                    RCLCPP_INFO(this->get_logger(), "Mezera vlevo a poslední ArUco ID %d – připravuju otočku doľeva", last_aruco_id_);
                    return;
                }

            }
            else if ((right_side > front_side_threshold) && (front > front_side_threshold+0.2f) && !just_turned && !waiting_before_turn) {
                // jestli posledni aruco říká jeď rovně neodbočuj doprava
                //točení podle aruco
                if (last_aruco_id_ == 0 || last_aruco_id_ == 10) {
                    RCLCPP_INFO(this->get_logger(),
                        "Ignoruju pravou odbočku – poslední ArUco ID %d říká jeď rovně.", last_aruco_id_);
                    last_aruco_id_=-1;
                    return;
                }
                if ((last_aruco_id_ == 2 || last_aruco_id_ == 12) && back>front_side_threshold+0.1f)
                {
                    wait_start_time = this->now();
                    waiting_before_turn = true;
                    turn_direction_ = 1;
                    last_aruco_id_=-1;
                    RCLCPP_INFO(this->get_logger(), "Mezera vpravo – připravuju otočku doprava");
                    return;
                }
            }

            if (waiting_before_turn) {
                auto time_waiting = (this->now() - wait_start_time).seconds();
                if (time_waiting < 1.31f) {
                    // ešte čakáme pred otočkou
                    return;
                } else {
                    waiting_before_turn = false;
                    motor_controller_->set_motor_speeds({128.0f, 128.0f});
                    state_ = DriveState::TURNING;
                    turn_start_yaw_ = imu_node_->getIntegratedResults();
                    first_time_in_drive = true;

                    just_turned = true;
                    just_turned_left = (turn_direction_ == -1 && front < front_side_threshold);
                    just_turned_right = (turn_direction_ == 1 && front < front_side_threshold);
                    if (just_turned_left == false && just_turned_right == false) {
                        go_straight = true;
                    }

                    RCLCPP_INFO(this->get_logger(), "Otočka %s, ArUco ID %d.",
                        (turn_direction_ == -1 ? "doleva" : "doprava"), last_aruco_id_);
                    return;
                }
            }

            float error;// = back_left - back_right;
            float error_avg;// = (error + last_error) / 2.0f;
            float d_error;// = error_avg - last_error;
            float speed_diff;

            float error_side_correction=1.5f;
            float error_action_threshold=0.03f;
            float side_too_far_threshhold=0.23f;


            //řídím v koridoru
            if (just_turned_left)
            {
                RCLCPP_INFO(this->get_logger(), "Držim se pravé stěny po otočce");
                error=right_side_follow_front-right_side_follow_back;
                if (std::isnan(error)){error=last_error;}
                if (error<-error_action_threshold){speed_diff=1.0f;}
                if (error>error_action_threshold){speed_diff=-1.0f;}
                if (right_side>side_too_far_threshhold){speed_diff=-1.0f;}
            }
            else if (just_turned_right)
            {
                RCLCPP_INFO(this->get_logger(), "Držím se levé stěny po otočce");
                error=left_side_follow_back-left_side_follow_front;
                if (std::isnan(error)){error=last_error;}
                if (error<=-error_action_threshold){speed_diff=1.0f;}
                if (error>=error_action_threshold){speed_diff=-1.0f;}
                if (left_side>side_too_far_threshhold){speed_diff=1.0f;}
            }
            else if (go_straight == true) {
                speed_diff=0;
                RCLCPP_INFO(this->get_logger(), "Vidím křižovatku +, jedu rovně");
            }
            // řízení pomocí obou stěn
            else if (left_side_follow_back <= front_side_threshold && left_side_follow_front <= front_side_threshold &&
                right_side_follow_back <= front_side_threshold && right_side_follow_front <= front_side_threshold)
            {
                error = back_left - back_right;
                error_avg = (error + last_error) / 2.0f;
                d_error = error_avg - last_error;
                speed_diff = Kp * error_avg + Kd * d_error + Ki * integral_;
                RCLCPP_INFO(this->get_logger(), "Řídím se oběma regulátory");
            }
            // řízení pomoce levé stěny
            else if (left_side_follow_back <= front_side_threshold+0.05f && left_side_follow_front <= front_side_threshold+0.05f && back_left<back_right)
            {
                RCLCPP_INFO(this->get_logger(), "Držim se levé stěny");
                error=left_side_follow_back-left_side_follow_front;
                if (std::isnan(error)){error=last_error;}
                if (error<=-error_action_threshold){speed_diff=1.0f;}
                if (error>=error_action_threshold){speed_diff=-1.0f;}
                if (left_side>side_too_far_threshhold){speed_diff=1.0f;}
            }
            // řízení pomocí levé stěny
            else if (right_side_follow_back <= front_side_threshold+0.05f && right_side_follow_front <= front_side_threshold+0.05f)
            {
                error=right_side_follow_front-right_side_follow_back;
                if (std::isnan(error)){error=last_error;}
                if (error<-error_action_threshold){speed_diff=1.0f;}
                if (error>error_action_threshold){speed_diff=-1.0f;}
                if (right_side>side_too_far_threshhold){speed_diff=-1.0f;}
                RCLCPP_INFO(this->get_logger(), "Držím se pravé stěny");
            }
            // neřídím
            else if (((front > front_side_threshold && left_side > front_side_threshold) && ((front > front_side_threshold && right_side > front_side_threshold))
                || (back_left>front_side_threshold && back_right>front_side_threshold)))
            {
                speed_diff=0;
                RCLCPP_INFO(this->get_logger(), "Vidim krizovatku +, jedu rovne");
            }

            if (front_left<side_threshold && front_left<front_right){speed_diff=-1.0f;}
            else if (front_right<side_threshold){speed_diff=1.0f;}

            speed_diff = std::clamp(speed_diff,-2.0f,2.0f);
            left_motor_speed = base_speed - speed_diff;
            right_motor_speed = base_speed + speed_diff;

            motor_controller_->set_motor_speeds({
                std::clamp(left_motor_speed, 128.0f, 2 * (base_speed - 128.0f)+128.0f),
                std::clamp(right_motor_speed, 128.0f, 2 *(base_speed - 128.0f)+128.0f)
            });

            last_error = error;
            }
    }
        else if (state_ == DriveState::TURNING) {
            float current_yaw = imu_node_->getIntegratedResults();
            float target_yaw = round_to_rad90(ImuNode::normalize_angle(turn_start_yaw_ - turn_direction_ * M_PI_2));
            float delta_yaw =ImuNode::normalize_angle(target_yaw-current_yaw);//std::abs(
            if (std::abs(delta_yaw) <= 0.09f) {
            // Otočené
                motor_controller_->set_motor_speeds({128, 128});
                state_ = DriveState::DRIVE_FORWARD;
                last_error = 0;
                integral_ = 0;
                RCLCPP_INFO(this->get_logger(), "Otočka dokončená. Pokračuju dopředu.");
                return;
            }
            // Otáčej podle turn_direction, pokud je uhel větší než 0.3 zrychli otočku, pokud ne nastav rychlost 3
            float turn_speed;
            if (std::abs(delta_yaw)>0.3f){turn_speed=9.0f;}
            else{turn_speed=3.0f;}

            if (turn_direction_ == 1) {
                motor_controller_->set_motor_speeds({128 + turn_speed, 128 - turn_speed});
            } else {
                motor_controller_->set_motor_speeds({128 - turn_speed, 128 + turn_speed});
            }
            RCLCPP_INFO(this->get_logger(), "[TURNING] Yaw: %.2f ΔYaw: %.2f ΔTargetYaw: %.2f", current_yaw, delta_yaw,target_yaw);
        }
    }

    void PidNode::aruco_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    int new_id = msg->data;

    bool collect_single_digit = true;
    bool collect_double_digit = false;

    auto now = this->now();


    if (collect_single_digit && new_id >= 0 && new_id <= 9) {
        last_aruco_id_ = new_id;
        RCLCPP_INFO(this->get_logger(), "Ukladám 1-ciferné ArUco ID: %d", new_id);
    } else if (collect_double_digit && new_id >= 10 && new_id <= 12) {
        last_aruco_id_ = new_id;
        RCLCPP_INFO(this->get_logger(), "Ukladám 2-ciferné ArUco ID: %d", new_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "ID %d nesplňuje aktuální režim.", new_id);
    }
}

}