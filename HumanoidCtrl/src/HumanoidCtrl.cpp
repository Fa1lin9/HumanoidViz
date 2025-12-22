#include "HumanoidCtrl.hpp"

using std::placeholders::_1;

HumanoidCtrl::HumanoidCtrl(const rclcpp::NodeOptions& options)
: Node(ProjectName, options)
{
    // Declare Parameters
    this->declare_parameter<std::string>("InputTopicName", "/HumanoidCtrl/JointStateWithoutStamp");
    this->declare_parameter<std::string>("JointTopic", "/HumanoidCtrl/JointState");
    this->declare_parameter<double>("PublishRate", 25.0);
    this->declare_parameter<std::vector<std::string>>("JointNames", std::vector<std::string>());
    this->declare_parameter<int>("JointNum", 0);
    this->declare_parameter<std::string>("RobotType", "Ti5Robot");
    // this->declare_parameter<std::string>("RobotModelPath", "/HumanoidCtrl/RobotModelPath");

    // Read Parameters
    this->inputTopicName = this->get_parameter("InputTopicName").as_string();
    this->jointTopic = this->get_parameter("JointTopic").as_string();
    this->publishRate = this->get_parameter("PublishRate").as_double();
    this->jointNames = this->get_parameter("JointNames").as_string_array();
    this->jointNum = this->get_parameter("JointNum").as_int();
    this->robotType = this->get_parameter("RobotType").as_string();
    // this->robotModelPath = this->get_parameter("RobotModelPath").as_string();

    // Check Parameter
    // Check TopicName
    if (this->inputTopicName.empty() || this->jointTopic.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[HumanoidCtrl] The TopicName cannot be empty! ");
    }

    // Check PublishRate
    if (this->publishRate <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "[HumanoidCtrl] The PublishRate(%.2f) is invalid! ", this->publishRate);
        this->publishRate = 25.0;
    }

    // Check JointNum
    if (this->jointNum <= 0) {
        RCLCPP_FATAL(this->get_logger(), "[HumanoidCtrl] The JointNum(%.u) is invalid! ", this->jointNum);
        throw std::runtime_error("[HumanoidCtrl] Invalid JointNum");
    }

    // Check JointNames
    if (this->jointNames.empty()) {
        RCLCPP_FATAL(this->get_logger(), "[HumanoidCtrl] The JointNames is empty!");
        throw std::runtime_error("[HumanoidCtrl] Empty JointNames");
    }

    if (static_cast<int>(this->jointNames.size()) != this->jointNum) {
        RCLCPP_ERROR(this->get_logger(), 
            "[HumanoidCtrl] The size of the JointNames(%lu) is not equal to JointNum(%u)", 
            this->jointNames.size(), this->jointNum);
            throw std::runtime_error("[HumanoidCtrl] The size of the JointNames is not equal to JointNum");
    }

    for (const auto& name : this->jointNames) {
        RCLCPP_DEBUG(
            this->get_logger(), 
            "[HumanoidCtrl] Load joint: %s", 
            name.c_str());
    }

    // Publisher
    this->outputTopicPub = 
        this->create_publisher<sensor_msgs::msg::JointState>(
            "/" + this->robotType + this->jointTopic, 10
        );

    // Subscriber
    this->inputTopicSub = 
        this->create_subscription<humanoid_msgs::msg::JointState>(
            this->inputTopicName, 10,
            std::bind(&HumanoidCtrl::InputDataCallBack, this, _1)
        );
    
    // Timer
    auto period = std::chrono::duration<double>(1.0 / this->publishRate);
    this->timer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&HumanoidCtrl::PublishJointState, this)
    );

    // Init
    this->lastPosition.resize(this->jointNum);

    RCLCPP_INFO(
        get_logger(), 
        "[HumanoidCtrl] HumanoidCtrl started, Publishing to [%s] at %.1f Hz! ",
        this->jointTopic.c_str(),
        this->publishRate
    );
}

HumanoidCtrl::~HumanoidCtrl()
{   
    
}

void HumanoidCtrl::InputDataCallBack(const humanoid_msgs::msg::JointState &msg)
{
    if (msg.name.size() != msg.position.size()) {
        RCLCPP_ERROR(
            get_logger(),
            "[HumanoidCtrl::InputDataCallBack] JointState name size (%zu) != position size (%zu)",
            msg.name.size(), msg.position.size()
        );
        return;
    }
    
    if(msg.position.empty()){
        RCLCPP_ERROR(
            this->get_logger(),
            "[HumanoidCtrl::InputDataCallBack] Received msg, but the position of the msg is empty! "
        );
        return;
    }

    if(msg.name.empty()){
        RCLCPP_ERROR(
            this->get_logger(),
            "[HumanoidCtrl::InputDataCallBack] Received msg, but the name of the msg is empty! "
        );
        return;
    }

    RCLCPP_INFO(
        get_logger(), 
        "[HumanoidCtrl] Received msg, the size of the joints is %zu! ",
        msg.name.size()
    );

    if(0){
        for(size_t i=0;i<msg.name.size();i++){
            RCLCPP_INFO(
                get_logger(), 
                "[HumanoidCtrl] Joint %zu %s: Position %f. ",
                i,
                msg.name[i].c_str(),
                msg.position[i]
            );
        }
    }

    std::lock_guard<std::mutex> lock(this->mtx);
    this->lastPosition = msg.position;
    this->jointNames = msg.name;
}

void HumanoidCtrl::PublishJointState(){
    std::vector<double> position;
    std::vector<std::string> name;

    {
        std::lock_guard<std::mutex> lock(this->mtx);
        position = this->lastPosition;
        name = this->jointNames;
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = this->now();
    out.name = name;
    out.position = position;

    this->outputTopicPub->publish(out);
}

// void HumanoidCtrl::TraverseJoints(
//     const urdf::LinkConstSharedPtr &link,
//     std::vector<std::string> &jointNames)
// {
//     for (const auto &childJoint : link->child_joints)
//     {
//         if (childJoint->type != urdf::Joint::FIXED)
//             jointNames.push_back(childJoint->name);

//         // map 查名字得到 LinkSharedPtr，然后转成 LinkConstSharedPtr
//         auto it = this->robotModel.links_.find(childJoint->child_link_name);
//         if (it != this->robotModel.links_.end())
//         {
//             auto childLink = std::const_pointer_cast<const urdf::Link>(it->second);
//             TraverseJoints(childLink, jointNames);
//         }
//     }
// }

