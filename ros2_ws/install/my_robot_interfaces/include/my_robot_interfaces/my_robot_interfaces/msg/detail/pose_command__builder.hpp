// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_interfaces:msg/PoseCommand.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__POSE_COMMAND__BUILDER_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__POSE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_interfaces/msg/detail/pose_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_PoseCommand_cartesian_path
{
public:
  explicit Init_PoseCommand_cartesian_path(::my_robot_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::msg::PoseCommand cartesian_path(::my_robot_interfaces::msg::PoseCommand::_cartesian_path_type arg)
  {
    msg_.cartesian_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_yaw
{
public:
  explicit Init_PoseCommand_yaw(::my_robot_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_cartesian_path yaw(::my_robot_interfaces::msg::PoseCommand::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PoseCommand_cartesian_path(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_pitch
{
public:
  explicit Init_PoseCommand_pitch(::my_robot_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_yaw pitch(::my_robot_interfaces::msg::PoseCommand::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_PoseCommand_yaw(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_roll
{
public:
  explicit Init_PoseCommand_roll(::my_robot_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_pitch roll(::my_robot_interfaces::msg::PoseCommand::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_PoseCommand_pitch(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_z
{
public:
  explicit Init_PoseCommand_z(::my_robot_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_roll z(::my_robot_interfaces::msg::PoseCommand::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_PoseCommand_roll(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_y
{
public:
  explicit Init_PoseCommand_y(::my_robot_interfaces::msg::PoseCommand & msg)
  : msg_(msg)
  {}
  Init_PoseCommand_z y(::my_robot_interfaces::msg::PoseCommand::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_PoseCommand_z(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCommand msg_;
};

class Init_PoseCommand_x
{
public:
  Init_PoseCommand_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseCommand_y x(::my_robot_interfaces::msg::PoseCommand::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PoseCommand_y(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::msg::PoseCommand>()
{
  return my_robot_interfaces::msg::builder::Init_PoseCommand_x();
}

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__POSE_COMMAND__BUILDER_HPP_
