#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using Float64MultiArray = example_interfaces::msg::Float64MultiArray;
using PoseCommand = my_robot_interfaces::msg::PoseCommand;
using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
        :isAbsolute_(true)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        set_abs_sub_ = node_->create_subscription<Bool>(
            "set_abs", 10, std::bind(&Commander::SetAbsoluteCallback, this, _1));

        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper", 10, std::bind(&Commander::OpenGripperCallback, this, _1));

        move_arm_by_joint_sub_ = node_->create_subscription<Float64MultiArray>(
            "move_arm_by_joint", 10, std::bind(&Commander::MoveArmByJointCallback, this, _1));

        move_arm_by_pose_sub_ = node_->create_subscription<PoseCommand>(
            "move_arm_by_pose", 10, std::bind(&Commander::MoveArmByPoseCallback, this, _1));
    }

    void MoveByNamedGoal(const std::string& name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        PlanAndExecute(arm_);
    }

    void MoveByJointGoal(const std::vector<double>& joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        PlanAndExecute(arm_);
    }

    void MoveByPoseGoal(double x, double y, double z,
                        double roll, double pitch, double yaw,
                        bool isCartesian)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        if(isCartesian) {
            MoveL(target_pose.pose);
        }
        else{
            MoveJ(target_pose);
        }
    }

    void OpenGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        PlanAndExecute(gripper_);
    }

    void CloseGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        PlanAndExecute(gripper_);
    }

private:
    void MoveJ(const geometry_msgs::msg::PoseStamped& target)
    {
        arm_->setStartStateToCurrentState();
        arm_->setPoseTarget(target);
        PlanAndExecute(arm_);
    }

    void MoveL(const geometry_msgs::msg::Pose& pose)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        if(fraction == 1) {
            arm_->execute(trajectory);
        }
    }

    void PlanAndExecute(const std::shared_ptr<MoveGroupInterface>& interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
            interface->execute(plan);
        }
    }

    void SetAbsoluteCallback(const Bool &msg)
    {
        isAbsolute_ = msg.data;
        if(msg.data){
            RCLCPP_INFO_ONCE(node_->get_logger(), "绝对运动模式设置成功");
        }
        else{
            RCLCPP_INFO_ONCE(node_->get_logger(), "相对运动模式设置成功");
        }
    }

    void OpenGripperCallback(const Bool &msg)
    {
        if(msg.data){
            OpenGripper();
        }
        else{
            CloseGripper();
        }
    }

    void MoveArmByJointCallback(const Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> joints = msg->data;
        if(joints.size() != 6) {
            RCLCPP_WARN_ONCE(node_->get_logger(), "参数个数应该是6个");
            return;
        }

        if(!isAbsolute_){
            std::vector<double> curJoints = arm_->getCurrentJointValues();
            if(curJoints.size() != joints.size()) {
                RCLCPP_WARN_ONCE(node_->get_logger(), "关节角个数错误");
                return;
            }

            for(std::size_t i = 0; i < joints.size(); ++i){
                joints[i] += curJoints[i];
            }
        }

        MoveByJointGoal(joints);
    }

    void MoveArmByPoseCallback(const PoseCommand::SharedPtr msg)
    {
        double x = msg->x;
        double y = msg->y;
        double z = msg->z;
        double roll = msg->roll;
        double pitch = msg->pitch;
        double yaw = msg->yaw;

        bool isCartesian = msg->cartesian_path;

        if(!isAbsolute_){
            geometry_msgs::msg::Pose curPose = arm_->getCurrentPose().pose;
            x += curPose.position.x;
            y += curPose.position.y;
            z += curPose.position.z;

            tf2::Quaternion q;
            geometry_msgs::msg::Quaternion& ori = curPose.orientation;
            q.setValue(ori.x, ori.y, ori.z, ori.w);
            q.normalize();

            tf2::Matrix3x3 m(q);
            double curRoll, curPitch, curYaw;
            m.getRPY(curRoll, curPitch, curYaw);
            roll += curRoll;
            pitch += curPitch;
            yaw += curYaw;
        }

        MoveByPoseGoal(x, y, z, roll, pitch, yaw, isCartesian);
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    bool isAbsolute_;
    rclcpp::Subscription<Bool>::SharedPtr set_abs_sub_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<Float64MultiArray>::SharedPtr move_arm_by_joint_sub_;
    rclcpp::Subscription<PoseCommand>::SharedPtr move_arm_by_pose_sub_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
