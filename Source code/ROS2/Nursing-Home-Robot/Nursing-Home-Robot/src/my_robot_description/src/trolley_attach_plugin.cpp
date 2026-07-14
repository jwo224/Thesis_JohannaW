#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace my_robot_description
{

class TrolleyAttachPlugin : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);

    robot_model_name_ = sdf->Get<std::string>("robot_model", "mecanum_bot").first;
    robot_link_name_ = sdf->Get<std::string>("robot_link", "base_link").first;
    trolley_model_name_ = sdf->Get<std::string>("trolley_model", "Trolley").first;
    trolley_link_name_ = sdf->Get<std::string>("trolley_link", "link").first;

    attach_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "attach_trolley",
      std::bind(
        &TrolleyAttachPlugin::Attach,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    detach_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "detach_trolley",
      std::bind(
        &TrolleyAttachPlugin::Detach,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      node_->get_logger(),
      "Trolley attach plugin ready. Services: /attach_trolley and /detach_trolley");
  }

private:
  void Attach(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (joint_)
    {
      response->success = true;
      response->message = "Trolley is already attached.";
      return;
    }

    auto robot_model = world_->ModelByName(robot_model_name_);
    auto trolley_model = world_->ModelByName(trolley_model_name_);
    if (!robot_model || !trolley_model)
    {
      response->success = false;
      response->message = "Robot or trolley model not found.";
      return;
    }

    auto robot_link = robot_model->GetLink(robot_link_name_);
    auto trolley_link = trolley_model->GetLink(trolley_link_name_);
    if (!robot_link || !trolley_link)
    {
      response->success = false;
      response->message = "Robot or trolley link not found.";
      return;
    }

    joint_ = world_->Physics()->CreateJoint("fixed", robot_model);
    if (!joint_)
    {
      response->success = false;
      response->message = "Failed to create fixed joint.";
      return;
    }

    joint_->SetName("robot_trolley_pin_joint");
    joint_->Load(robot_link, trolley_link, ignition::math::Pose3d::Zero);
    joint_->Init();

    response->success = true;
    response->message = "Trolley physically attached with fixed joint.";
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
  }

  void Detach(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!joint_)
    {
      response->success = true;
      response->message = "Trolley is already detached.";
      return;
    }

    joint_->Detach();
    joint_.reset();

    response->success = true;
    response->message = "Trolley detached.";
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::physics::JointPtr joint_;
  std::mutex mutex_;

  std::string robot_model_name_;
  std::string robot_link_name_;
  std::string trolley_model_name_;
  std::string trolley_link_name_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_service_;
};

}  // namespace my_robot_description

GZ_REGISTER_WORLD_PLUGIN(my_robot_description::TrolleyAttachPlugin)
