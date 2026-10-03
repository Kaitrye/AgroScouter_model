#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/transport/Node.hh>

namespace robot_sim {

class TrackAnimation final : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    const gz::sim::Model model(entity);
    auto config = sdf->Clone();
    count_ = sdf->Get<unsigned int>("tread_count", 48u).first;
    height_ = sdf->Get<double>("track_height", 0.325).first;
    if (count_ < 8 || !std::isfinite(height_) || height_ <= 0) {
      gzerr << "Invalid track animation dimensions" << std::endl;
      return;
    }

    for (unsigned int side = 0; side < 2; ++side) {
      const std::string contourName = side == 0 ? "left_contour" : "right_contour";
      if (!sdf->HasElement(contourName)) {
        gzerr << "Track animation missing " << contourName << std::endl;
        return;
      }
      auto point = config->GetElement(contourName)->GetElement("point");
      while (point) {
        const double x = point->Get<double>("x");
        const double z = point->Get<double>("z");
        if (!std::isfinite(x) || !std::isfinite(z)) {
          gzerr << "Nonfinite point in " << contourName << std::endl;
          return;
        }
        contour_[side].emplace_back(x, z);
        point = point->GetNextElement("point");
      }
      if (contour_[side].size() < 3) {
        gzerr << "Track animation needs at least three contour points" << std::endl;
        return;
      }
      const auto &path = contour_[side];
      for (size_t i = 0; i < path.size(); ++i) {
        const double length = (path[(i + 1) % path.size()] - path[i]).Length();
        if (length <= 0) {
          gzerr << "Track animation has duplicate contour points" << std::endl;
          return;
        }
        segmentLength_[side].push_back(length);
        perimeter_[side] += length;
      }
      const std::string name = side == 0 ? "left_track" : "right_track";
      const auto link = model.LinkByName(ecm, name);
      if (link == gz::sim::kNullEntity) {
        gzerr << "Track animation cannot find " << name << std::endl;
        return;
      }
      for (unsigned int i = 0; i < count_; ++i) {
        const auto visual = ecm.EntityByComponents(
            gz::sim::components::Name("tread_" + name + "_" + std::to_string(i)),
            gz::sim::components::ParentEntity(link),
            gz::sim::components::Visual());
        if (visual == gz::sim::kNullEntity) {
          gzerr << "Track animation missing tread " << i << " on " << name << std::endl;
          return;
        }
        treads_[side].push_back(visual);
      }
      const auto topic = "/model/robot/link/" + name + "/track_cmd_vel";
      bool subscribed = side == 0
          ? transport_.Subscribe(topic, &TrackAnimation::OnLeft, this)
          : transport_.Subscribe(topic, &TrackAnimation::OnRight, this);
      if (!subscribed) gzerr << "Cannot subscribe to " << topic << std::endl;
    }
    gzmsg << "Visual track animation loaded with " << count_
          << " treads per side" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || treads_[0].empty() || treads_[1].empty()) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0) return;
    const auto now = std::chrono::steady_clock::now();
    const std::array<double, 2> velocity = {
        SecondsSince(now, leftReceived_) < 0.8 ? leftVelocity_.load() : 0.0,
        SecondsSince(now, rightReceived_) < 0.8 ? rightVelocity_.load() : 0.0};
    for (unsigned int side = 0; side < 2; ++side)
      phase_[side] = std::fmod(phase_[side] + velocity[side] * dt, perimeter_[side]);

    elapsed_ += dt;
    if (elapsed_ < 1.0 / 30.0) return;
    elapsed_ = 0;
    for (unsigned int side = 0; side < 2; ++side) {
      for (unsigned int i = 0; i < count_; ++i) {
        auto *pose = ecm.Component<gz::sim::components::Pose>(treads_[side][i]);
        if (!pose) continue;
        pose->Data() = TreadPose(side, i * perimeter_[side] / count_ + phase_[side]);
        ecm.SetChanged(treads_[side][i], gz::sim::components::Pose::typeId,
                       gz::sim::ComponentState::OneTimeChange);
      }
    }
  }

 private:
  static double SecondsSince(std::chrono::steady_clock::time_point now,
                             const std::atomic<long long> &stamp) {
    const auto ticks = stamp.load();
    if (ticks == 0) return 1e9;
    return std::chrono::duration<double>(now.time_since_epoch()).count() - ticks * 1e-9;
  }

  static long long ClockTicks() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void OnLeft(const gz::msgs::Double &msg) {
    leftVelocity_.store(msg.data());
    leftReceived_.store(ClockTicks());
  }
  void OnRight(const gz::msgs::Double &msg) {
    rightVelocity_.store(msg.data());
    rightReceived_.store(ClockTicks());
  }

  gz::math::Pose3d TreadPose(unsigned int side, double distance) const {
    double s = std::fmod(distance, perimeter_[side]);
    if (s < 0) s += perimeter_[side];
    const auto &path = contour_[side];
    for (size_t i = 0; i < path.size(); ++i) {
      const double length = segmentLength_[side][i];
      if (s <= length || i + 1 == path.size()) {
        const auto direction = (path[(i + 1) % path.size()] - path[i]) / length;
        const double dx = direction.X(), dz = direction.Y();
        const auto center = path[i] + direction * s;
        constexpr double outwardOffset = 0.004;
        const double x = center.X() - dz * outwardOffset;
        const double z = center.Y() + dx * outwardOffset;
        // The box's X axis is tangent to the CAD belt silhouette.
        return {{x, 0, z - height_ / 2.0}, {0, std::atan2(-dz, dx), 0}};
      }
      s -= length;
    }
    return {};
  }

  gz::transport::Node transport_;
  std::array<std::vector<gz::sim::Entity>, 2> treads_;
  std::array<std::vector<gz::math::Vector2d>, 2> contour_;
  std::array<std::vector<double>, 2> segmentLength_;
  std::array<double, 2> perimeter_{{0, 0}};
  std::array<double, 2> phase_{{0, 0}};
  std::atomic<double> leftVelocity_{0}, rightVelocity_{0};
  std::atomic<long long> leftReceived_{0}, rightReceived_{0};
  unsigned int count_{0};
  double height_{0};
  double elapsed_{0};
};

}  // namespace robot_sim

GZ_ADD_PLUGIN(robot_sim::TrackAnimation, gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
