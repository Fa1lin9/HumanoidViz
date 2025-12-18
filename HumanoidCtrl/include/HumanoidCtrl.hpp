#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <string>
#include <humanoid_msgs/msg/joint_state.hpp>
#include <mutex>
#include <urdf/model.h>
#include <fstream>
#include <sstream>

const std::string ProjectName = "HumanoidCtrl";

class HumanoidCtrl : public rclcpp::Node
{
public:
    explicit HumanoidCtrl(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~HumanoidCtrl();

private:
    void InputDataCallBack(const humanoid_msgs::msg::JointState& msg);
    void PublishJointState();

    // void TraverseJoints(const urdf::LinkConstSharedPtr &link,
    //                     std::vector<std::string> &jointNames);

    // Parameter
    std::string inputTopicName;
    std::string outputTopicName;
    double publishRate;
    std::string robotModelPath;
    urdf::Model robotModel;
    std::vector<std::string> jointNames;
    int jointNum;

    // Pub and Sub
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr outputTopicPub;
    rclcpp::Subscription<humanoid_msgs::msg::JointState>::SharedPtr inputTopicSub;
    rclcpp::TimerBase::SharedPtr timer;

    // Data
    std::vector<double> lastPosition;
    std::mutex mtx;
};


