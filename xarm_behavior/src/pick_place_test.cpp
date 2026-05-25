/**
 * @file pick_place_test.cpp
 * @brief ROS 2 MoveIt 2 MoveGroup node for robotic arm control
 * @author Diego Carvajal
 * @date 2026
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <variant>

class MoveGroupNode : public rclcpp::Node
{
public:
    MoveGroupNode()
    : Node("pick_place_test")
    {
        // Declare parameters
        this->declare_parameter<std::string>("planning_group", "xarm");
        this->declare_parameter<std::string>("planning_pipeline", "ompl");
        this->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");
        this->declare_parameter<double>("planning_time", 1.0);
        this->declare_parameter<double>("max_velocity_scaling", 1.0);
        this->declare_parameter<double>("max_acceleration_scaling", 1.0);

        // Get parameters
        planning_group_ = this->get_parameter("planning_group").as_string();
        planning_pipeline_ = this->get_parameter("planning_pipeline").as_string();
        planner_id_ = this->get_parameter("planner_id").as_string();
        planning_time_ = this->get_parameter("planning_time").as_double();
        max_velocity_scaling_ = this->get_parameter("max_velocity_scaling").as_double();
        max_acceleration_scaling_ = this->get_parameter("max_acceleration_scaling").as_double();

        RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface for group: %s", planning_group_.c_str());

        // Create MoveGroupInterface
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_);

        // Configure planning
        move_group_interface_->setPlanningPipelineId(planning_pipeline_);
        move_group_interface_->setPlannerId(planner_id_);
        move_group_interface_->setPlanningTime(planning_time_);
        move_group_interface_->setMaxVelocityScalingFactor(max_velocity_scaling_);
        move_group_interface_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

        // Get joint model group
        const moveit::core::JointModelGroup* joint_model_group =
            move_group_interface_->getCurrentState()->getJointModelGroup(planning_group_);

        // Print info
        RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
        RCLCPP_INFO(get_logger(), "End effector link: %s", move_group_interface_->getEndEffectorLink().c_str());
        RCLCPP_INFO(get_logger(), "Planner ID: %s", move_group_interface_->getPlannerId().c_str());

        // Create PlanningSceneInterface for collision objects
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Create services
        plan_service_ = this->create_service<moveit_msgs::srv::GetPlanningScene>(
            "plan_trajectory",
            std::bind(&MoveGroupNode::planCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Create subscriber for goal poses (optional)
        goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&MoveGroupNode::goalPoseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "MoveGroup node initialized successfully");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    std::string planning_group_;
    std::string planning_pipeline_;
    std::string planner_id_;
    double planning_time_;
    double max_velocity_scaling_;
    double max_acceleration_scaling_;

    rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr plan_service_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;

    void planCallback(
        const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request> request,
        std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Received planning request");

        // Plan to current state or set target
        auto current_state = move_group_interface_->getCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto success = move_group_interface_->plan(plan);

        if (success) {
            RCLCPP_INFO(get_logger(), "Planning succeeded");
            response->success = true;
        } else {
            RCLCPP_ERROR(get_logger(), "Planning failed");
            response->success = false;
        }
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received goal pose");

        move_group_interface_->setPoseTarget(msg);

        auto [success, plan] = [this]() {
            moveit::planning_interface::MoveGroupInterface::Plan msg_plan;
            auto ok = static_cast<bool>(move_group_interface_->plan(msg_plan));
            return std::make_pair(ok, msg_plan);
        }();

        if (success) {
            RCLCPP_INFO(get_logger(), "Planning to pose succeeded");
            // Uncomment for real robot execution:
            // move_group_interface_->execute(plan);
        } else {
            RCLCPP_ERROR(get_logger(), "Planning to pose failed");
        }
    }

    // Helper: Move to joint positions
    bool moveToJointValues(const std::vector<double>& joint_values)
    {
        move_group_interface_->setJointValueTarget(joint_values);
        auto [success, plan] = [this]() {
            moveit::planning_interface::MoveGroupInterface::Plan msg_plan;
            auto ok = static_cast<bool>(move_group_interface_->plan(msg_plan));
            return std::make_pair(ok, msg_plan);
        }();

        if (success) {
            RCLCPP_INFO(get_logger(), "Joint plan succeeded");
            // move_group_interface_->execute(plan);
            return true;
        }
        RCLCPP_ERROR(get_logger(), "Joint plan failed");
        return false;
    }

    // Helper: Move to pose
    bool moveToPose(const geometry_msgs::msg::Pose& pose)
    {
        move_group_interface_->setPoseTarget(pose);
        auto [success, plan] = [this]() {
            moveit::planning_interface::MoveGroupInterface::Plan msg_plan;
            auto ok = static_cast<bool>(move_group_interface_->plan(msg_plan));
            return std::make_pair(ok, msg_plan);
        }();

        if (success) {
            RCLCPP_INFO(get_logger(), "Pose plan succeeded");
            // move_group_interface_->execute(plan);
            return true;
        }
        RCLCPP_ERROR(get_logger(), "Pose plan failed");
        return false;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveGroupNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}