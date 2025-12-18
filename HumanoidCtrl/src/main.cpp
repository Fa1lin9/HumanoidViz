#include "HumanoidCtrl.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto humanoid_ctrl_node = std::make_shared<HumanoidCtrl>(options);

    rclcpp::spin(humanoid_ctrl_node);
    rclcpp::shutdown();

    return 0;
}
