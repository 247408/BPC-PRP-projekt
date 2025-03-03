#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.hpp"
#include "nodes/io_node.hpp"


int main(int argc, char* argv[]) {
    // rclcpp::init(argc, argv);
    //
    // // Create an executor (for handling multiple nodes)
    // auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    //
    // // Create multiple nodes
    // auto node1 = std::make_shared<rclcpp::Node>("node1");
    // auto node2 = std::make_shared<rclcpp::Node>("node2");
    //
    // // Create instances of RosExampleClass using the existing nodes
    // auto example_class1 = std::make_shared<RosExampleClass>(node1, "topic1", 0.001);
    // auto example_class2 = std::make_shared<RosExampleClass>(node2, "topic2", 2.0);
    //
    // // Add nodes to the executor
    // executor->add_node(node1);
    // executor->add_node(node2);
    //
    // // Run the executor (handles callbacks for both nodes)
    // executor->spin();
    //
    // // Shutdown ROS 2
    // rclcpp::shutdown();
    // return 0;
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto io_node = std::make_shared<nodes::IoNode>();

    executor->add_node(io_node);

    auto executor_thread = std::thread([&executor]() { executor->spin(); });

    while (rclcpp::ok()) {
        switch (io_node->get_button_pressed()) {
            case 0:

                break;
            case 1:
                break;
            case 2:
                break;
        }
    }

    //executor->spin();

    rclcpp::shutdown();
    return 0;
}
