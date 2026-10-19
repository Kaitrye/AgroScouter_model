#include <cmath>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace robot_sim {

// Model a small, persistent speed difference between the two track motors.
// TrackedVehicle sends ideal track speeds; TrackController receives scaled speeds.
class TrackMotorBias final : public gz::sim::System,
                             public gz::sim::ISystemConfigure {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    leftScale_ = sdf->Get<double>("left_speed_scale", 1.0).first;
    rightScale_ = sdf->Get<double>("right_speed_scale", 1.0).first;
    if (!std::isfinite(leftScale_) || !std::isfinite(rightScale_) ||
        leftScale_ <= 0 || rightScale_ <= 0) {
      gzerr << "Track motor speed scales must be positive and finite" << std::endl;
      return;
    }

    const std::string prefix = "/model/" + gz::sim::Model(entity).Name(ecm) + "/link/";
    const std::string left = prefix + "left_track/";
    const std::string right = prefix + "right_track/";
    leftPub_ = node_.Advertise<gz::msgs::Double>(left + "track_cmd_vel");
    rightPub_ = node_.Advertise<gz::msgs::Double>(right + "track_cmd_vel");
    const bool leftSubscribed =
        node_.Subscribe(left + "ideal_track_cmd_vel", &TrackMotorBias::OnLeft, this);
    const bool rightSubscribed =
        node_.Subscribe(right + "ideal_track_cmd_vel", &TrackMotorBias::OnRight, this);
    if (!leftSubscribed || !rightSubscribed) {
      gzerr << "Cannot subscribe to ideal track motor commands" << std::endl;
    }
  }

 private:
  void OnLeft(const gz::msgs::Double &msg) {
    gz::msgs::Double output;
    output.set_data(msg.data() * leftScale_);
    leftPub_.Publish(output);
  }

  void OnRight(const gz::msgs::Double &msg) {
    gz::msgs::Double output;
    output.set_data(msg.data() * rightScale_);
    rightPub_.Publish(output);
  }

  gz::transport::Node node_;
  gz::transport::Node::Publisher leftPub_;
  gz::transport::Node::Publisher rightPub_;
  double leftScale_{1.0};
  double rightScale_{1.0};
};

}  // namespace robot_sim

GZ_ADD_PLUGIN(robot_sim::TrackMotorBias, gz::sim::System,
              gz::sim::ISystemConfigure)
