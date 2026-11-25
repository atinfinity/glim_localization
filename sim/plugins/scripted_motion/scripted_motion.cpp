// Scripted, deterministic planar motion for the Gazebo evaluation (scripts/sim_eval.sh).
//
// The sensor body hangs from the world on a chain of joints: prismatic x, prismatic y (world axes), then
// revolute yaw. Each step, the plugin commands joint velocities from a profile of forward speed v(t) and
// yaw rate w(t) defined in the SDF, as functions of simulation time only, so the motion does not depend
// on wall time or on the rendering speed. The world-frame velocity is v * (cos yaw, sin yaw) with the yaw
// read back from the joint. The physics engine resolves the joint velocities as actuated joints, so the
// body has real accelerations (including centripetal ones) and the IMU sensor measures them.
//
// The plugin also publishes the world pose of one link (the IMU link) as gz.msgs.Pose, stamped with
// simulation time, at a fixed rate in simulation time. This is the ground truth T_world_imu.
//
// SDF:
//   <plugin filename="ScriptedMotion" name="sim_eval::ScriptedMotion">
//     <joint_x>..</joint_x> <joint_y>..</joint_y> <joint_yaw>..</joint_yaw>
//     <pose_link>..</pose_link> <pose_topic>/sim_eval/imu_pose</pose_topic> <pose_rate>100</pose_rate>
//     <segment duration="2" v0="0" v1="1" w="none|bump|sine" w_amp="0.5" w_cycles="1"/>  (repeated)
//   </plugin>
// Within a segment of duration T and s = (t - t_start) / T in [0, 1]:
//   v = v0 + (v1 - v0) * (1 - cos(pi s)) / 2                  smooth ramp, constant when v0 == v1
//   w = 0 (none), w_amp * (1 - cos(2 pi s)) / 2 (bump; integral w_amp * T / 2),
//       or w_amp * sin(2 pi w_cycles s) (sine; zero net yaw for whole cycles)
// Before the first and after the last segment, v = w = 0.
//
// The first pose is published at the first update (t = 0 when the server starts paused, even before
// it runs) and then every 1 / pose_rate s of simulation time, so the ground truth
// covers the first sensor messages. A missing joint or link, pose_rate <= 0, a segment duration <= 0,
// or a physics step that does not divide the pose period stops the server (exit code 2): a broken
// setup must not produce a recording.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace sim_eval {

[[noreturn]] void Fail(const std::string &what) {
  gzerr << "ScriptedMotion: " << what << std::endl;
  std::cerr << "ScriptedMotion: FAIL: " << what << std::endl;
  std::_Exit(2);  // no static destructors while the server threads run
}

struct Segment {
  double duration = 0.0;
  double v0 = 0.0;
  double v1 = 0.0;
  std::string w_shape = "none";
  double w_amp = 0.0;
  double w_cycles = 1.0;
};

class ScriptedMotion : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate,
                       public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    const auto joint = [&](const char *key) {
      const std::string name = sdf->Get<std::string>(key);
      const gz::sim::Entity e = model.JointByName(ecm, name);
      if (e == gz::sim::kNullEntity) {
        Fail("joint [" + name + "] (" + key + ") not found");
      }
      return e;
    };
    joint_x_ = joint("joint_x");
    joint_y_ = joint("joint_y");
    joint_yaw_ = joint("joint_yaw");
    const std::string link_name = sdf->Get<std::string>("pose_link");
    pose_link_ = model.LinkByName(ecm, link_name);
    if (pose_link_ == gz::sim::kNullEntity) {
      Fail("link [" + link_name + "] not found");
    }
    pose_topic_ = sdf->Get<std::string>("pose_topic", "/sim_eval/imu_pose").first;
    const double rate = sdf->Get<double>("pose_rate", 100.0).first;
    if (!(rate > 0.0)) {
      Fail("pose_rate must be > 0");
    }
    pose_period_ns_ = static_cast<int64_t>(std::llround(1e9 / rate));

    double start = 0.0;
    for (auto e = sdf->FindElement("segment"); e; e = e->GetNextElement("segment")) {
      Segment seg;
      seg.duration = e->Get<double>("duration");
      seg.v0 = e->Get<double>("v0");
      seg.v1 = e->Get<double>("v1");
      seg.w_shape = e->Get<std::string>("w", "none").first;
      seg.w_amp = e->Get<double>("w_amp", 0.0).first;
      seg.w_cycles = e->Get<double>("w_cycles", 1.0).first;
      if (!(seg.duration > 0.0)) {
        Fail("segment duration must be > 0");
      }
      if (seg.w_shape != "none" && seg.w_shape != "bump" && seg.w_shape != "sine") {
        Fail("unknown segment w shape [" + seg.w_shape + "]");
      }
      segments_.push_back(seg);
      starts_.push_back(start);
      start += seg.duration;
    }
    gzmsg << "ScriptedMotion: " << segments_.size() << " segments, " << start << " s" << std::endl;

    gz::sim::Joint(joint_yaw_).EnablePositionCheck(ecm, true);
    publisher_ = node_.Advertise<gz::msgs::Pose>(pose_topic_);
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) {
      return;
    }
    if (!step_checked_) {
      const int64_t dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count();
      if (dt_ns <= 0 || pose_period_ns_ % dt_ns != 0) {
        Fail("the physics step (" + std::to_string(dt_ns) + " ns) does not divide the pose period (" +
             std::to_string(pose_period_ns_) + " ns)");
      }
      step_checked_ = true;
    }
    const double t = std::chrono::duration<double>(info.simTime).count();
    double v = 0.0;
    double w = 0.0;
    Profile(t, v, w);

    double yaw = 0.0;
    const auto q = gz::sim::Joint(joint_yaw_).Position(ecm);
    if (q && !q->empty()) {
      yaw = q->front();
    }
    gz::sim::Joint(joint_x_).SetVelocity(ecm, {v * std::cos(yaw)});
    gz::sim::Joint(joint_y_).SetVelocity(ecm, {v * std::sin(yaw)});
    gz::sim::Joint(joint_yaw_).SetVelocity(ecm, {w});
  }

  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    // The first pose is published even while paused (at t = 0, like the first LiDAR frame of a server
    // that starts paused), then every period while running.
    if (info.paused && published_) {
      return;
    }
    const int64_t t_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if (published_ && t_ns % pose_period_ns_ != 0) {
      return;
    }
    published_ = true;
    const gz::math::Pose3d pose = gz::sim::worldPose(pose_link_, ecm);
    gz::msgs::Pose msg;
    *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(info.simTime);
    auto *frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value("world");
    gz::msgs::Set(&msg, pose);
    publisher_.Publish(msg);
  }

 private:
  void Profile(double t, double &v, double &w) const {
    v = 0.0;
    w = 0.0;
    for (size_t i = 0; i < segments_.size(); ++i) {
      const Segment &seg = segments_[i];
      if (t < starts_[i] || t >= starts_[i] + seg.duration) {
        continue;
      }
      const double s = (t - starts_[i]) / seg.duration;
      v = seg.v0 + (seg.v1 - seg.v0) * 0.5 * (1.0 - std::cos(M_PI * s));
      if (seg.w_shape == "bump") {
        w = seg.w_amp * 0.5 * (1.0 - std::cos(2.0 * M_PI * s));
      } else if (seg.w_shape == "sine") {
        w = seg.w_amp * std::sin(2.0 * M_PI * seg.w_cycles * s);
      }
      return;
    }
  }

  gz::sim::Entity joint_x_ = gz::sim::kNullEntity;
  gz::sim::Entity joint_y_ = gz::sim::kNullEntity;
  gz::sim::Entity joint_yaw_ = gz::sim::kNullEntity;
  gz::sim::Entity pose_link_ = gz::sim::kNullEntity;
  std::string pose_topic_;
  int64_t pose_period_ns_ = 10000000;
  bool step_checked_ = false;
  bool published_ = false;
  std::vector<Segment> segments_;
  std::vector<double> starts_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
};

}  // namespace sim_eval

GZ_ADD_PLUGIN(sim_eval::ScriptedMotion, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
