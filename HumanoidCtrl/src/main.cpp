#include <HumanoidCtrl.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto ctrlNode = std::make_shared<HumanoidCtrl>(options);

    rclcpp::spin(ctrlNode);
    rclcpp::shutdown();

    return 0;
}
