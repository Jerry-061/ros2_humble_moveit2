//STL Headers
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

void MoveByNamedGoal(moveit::planning_interface::MoveGroupInterface& arm, const std::string& name);
void MoveByJointGoal(moveit::planning_interface::MoveGroupInterface& arm, const std::vector<double>& joints);
void MoveByPoseGoal(moveit::planning_interface::MoveGroupInterface& arm, 
    double x, double y, double z, double roll, double pitch, double yaw, bool isCartesian);
void MoveJ(moveit::planning_interface::MoveGroupInterface& arm, const geometry_msgs::msg::PoseStamped& target);
void MoveL(moveit::planning_interface::MoveGroupInterface& arm, const geometry_msgs::msg::Pose& pose);

void PlanAndExecute(moveit::planning_interface::MoveGroupInterface& interface);


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){ executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    
    // Named Goal
    MoveByNamedGoal(arm, "pose_1");
    MoveByNamedGoal(arm, "home");

    // Joint Goal
    std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };
    MoveByJointGoal(arm, joints);

    // Pose Goal
    MoveByPoseGoal(arm, 0.0, -0.7, 0.4, 3.14, 0.0, 0.0, false);

    // Cartesian Path
    MoveByPoseGoal(arm, 0.0, -0.7, 0.6, 3.14, 0.0, 0.0, true);

    MoveByNamedGoal(arm, "home");

    rclcpp::shutdown();
    spinner.join();
    return 0;
}


void MoveByNamedGoal(moveit::planning_interface::MoveGroupInterface& arm, const std::string& name)
{
    arm.setStartStateToCurrentState();
    arm.setNamedTarget(name);
    PlanAndExecute(arm);
}

void MoveByJointGoal(moveit::planning_interface::MoveGroupInterface& arm, const std::vector<double>& joints)
{
    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);
    PlanAndExecute(arm);
}

void MoveByPoseGoal(moveit::planning_interface::MoveGroupInterface& arm, 
                    double x, double y, double z,
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
        MoveL(arm, target_pose.pose);
    }
    else{
        MoveJ(arm, target_pose);
    }
}

void MoveJ(moveit::planning_interface::MoveGroupInterface& arm, const geometry_msgs::msg::PoseStamped& target)
{
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target);
    PlanAndExecute(arm);
}

void MoveL(moveit::planning_interface::MoveGroupInterface& arm, const geometry_msgs::msg::Pose& pose)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pose);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if(fraction == 1) {
        arm.execute(trajectory);
    }
}

void PlanAndExecute(moveit::planning_interface::MoveGroupInterface& interface)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success){
        interface.execute(plan);
    }
}
