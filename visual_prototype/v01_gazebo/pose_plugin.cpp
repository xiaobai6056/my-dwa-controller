#include <memory>
#include <mutex>
#include <optional>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

namespace
{
using SetState = gazebo_msgs::srv::SetEntityState;

class V01Pose final : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);
    service_ = node_->create_service<SetState>(
      "v01_set_pose",
      [this](std::shared_ptr<rmw_request_id_t> header, SetState::Request::SharedPtr request) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (pending_) {
          SetState::Response response;
          response.success = false;
          service_->send_response(*header, response);
          return;
        }
        pending_.emplace(Pending{*header, request->state});
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &) {ApplyPose();});
  }

private:
  struct Pending
  {
    rmw_request_id_t header;
    gazebo_msgs::msg::EntityState state;
  };

  void ApplyPose()
  {
    if (world_->IsPaused()) {
      return;
    }
    std::optional<Pending> command;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      pending_.swap(command);
    }
    if (!command) {
      return;
    }

    // Apply on Gazebo's update thread without changing the user's pause state.
    auto & state = command->state;
    SetState::Response response;
    auto model = world_->ModelByName(state.name);
    response.success = model && state.reference_frame == "world";
    if (response.success) {
      model->SetWorldPose(gazebo_ros::Convert<ignition::math::Pose3d>(state.pose));
      model->SetLinearVel(gazebo_ros::Convert<ignition::math::Vector3d>(state.twist.linear));
      model->SetAngularVel(gazebo_ros::Convert<ignition::math::Vector3d>(state.twist.angular));
    }
    service_->send_response(command->header, response);
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  std::mutex mutex_;
  std::optional<Pending> pending_;
  rclcpp::Service<SetState>::SharedPtr service_;
  gazebo::event::ConnectionPtr update_;
};

GZ_REGISTER_WORLD_PLUGIN(V01Pose)
}  // namespace
