// Scripted walking proxies for the SAR world.
//
// Two controllers, selected per model by <controller_version>:
//
//   1  KINEMATIC. The model is placed on its route pose every step. It is
//      seen by every sensor and never falls over, but it walks through
//      walls and through the robot. Proven configuration.
//   2  CONTACT-AWARE. Forces drive a dynamic body toward the route pose, so
//      the person collides with terrain, walls, props and the robot, and
//      stops when blocked instead of jumping ahead. Needs a canonical link
//      with inertia (sar_physics.py writes one).
//
// Physics entities are bound lazily in PreUpdate: ParentEntity may not exist
// yet when Gazebo calls Configure. Missing physics must never enable teleporting.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Element.hh>

namespace sar
{
class WaypointSystem : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
{
  struct Key { double time; gz::math::Pose3d pose; };
  struct Member { std::string name; gz::math::Vector3d offset;
                  gz::sim::Entity entity{gz::sim::kNullEntity}; };
  gz::sim::Model model;
  gz::sim::Link link;
  gz::sim::Entity world{gz::sim::kNullEntity};
  std::vector<Key> keys;
  std::vector<Member> formation;
  gz::math::Vector3d offset{0, 0, 0};
  double phase{0}, lastTime{0}, cruiseSpeed{2.0}, mass{75};
  int controller{1};
  bool carried{false};
  bool physicsReady{false};

  // Index of the route segment containing t. Never returns end(), so callers
  // can always dereference both this and the element before it.
  std::vector<Key>::const_iterator Segment(double t) const
  {
    if (!std::isfinite(t)) t = 0;
    auto next = std::upper_bound(this->keys.begin(), this->keys.end(), t,
        [](double value, const Key &key) { return value < key.time; });
    if (next == this->keys.begin()) ++next;
    if (next == this->keys.end()) --next;
    return next;
  }

  // Wrap a route clock into [0, period). Returns 0 rather than NaN for any
  // non-finite input, so a bad step can never index outside the route.
  double Wrap(double time) const
  {
    const double period = this->keys.back().time;
    if (!std::isfinite(time) || !std::isfinite(period) || period <= 0) return 0;
    double t = std::fmod(time, period);
    if (t < 0) t += period;
    return (t < period) ? t : 0;
  }

  gz::math::Pose3d Sample(double time) const
  {
    const double t = this->Wrap(time);
    const auto next = this->Segment(t);
    const auto &a = *(next - 1);
    const auto &b = *next;
    const double span = b.time - a.time;
    const double u = (span > 0) ? std::clamp((t - a.time) / span, 0.0, 1.0) : 0.0;
    return {a.pose.Pos() + u * (b.pose.Pos() - a.pose.Pos()),
            gz::math::Quaterniond::Slerp(u, a.pose.Rot(), b.pose.Rot())};
  }

  gz::math::Pose3d Target(double time, const gz::math::Vector3d &off) const
  {
    auto pose = this->Sample(time);
    pose.Pos() += pose.Rot().RotateVector(off);
    return pose;
  }

  // Route metres per route second on the segment holding t.
  double RouteSpeed(double t) const
  {
    const auto next = this->Segment(t);
    const auto &a = *(next - 1);
    const double span = next->time - a.time;
    return (span > 0) ? a.pose.Pos().Distance(next->pose.Pos()) / span : 0.0;
  }

  public: void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    auto config = _sdf->Clone();
    if (!this->model.Valid(_ecm) || !config->HasElement("waypoint"))
    {
      gzerr << "[SAR motion] Requires a model with timed waypoints. "
            << "Update the world with sar_physics.py.\n";
      return;
    }
    this->offset = config->Get<gz::math::Vector3d>("offset", gz::math::Vector3d::Zero).first;
    this->cruiseSpeed = config->Get<double>("cruise_speed", 2.0).first;
    this->mass = config->Get<double>("mass", 75.0).first;
    this->carried = config->Get<bool>("carried", false).first;
    this->controller = config->Get<int>("controller_version", 1).first;
    if (!std::isfinite(this->cruiseSpeed) || this->cruiseSpeed <= 0 ||
        !std::isfinite(this->mass) || this->mass <= 0 || !this->offset.IsFinite())
    {
      gzerr << "[SAR motion] Invalid speed, mass or offset.\n";
      return;
    }
    if (this->controller != 1 && this->controller != 2)
      this->controller = 1;

    for (auto wp = config->GetElement("waypoint"); wp; wp = wp->GetNextElement("waypoint"))
    {
      Key key{wp->Get<double>("time"), wp->Get<gz::math::Pose3d>("pose")};
      if (!std::isfinite(key.time) || !key.pose.Pos().IsFinite() ||
          !key.pose.Rot().IsFinite() || key.time < 0 ||
          (!this->keys.empty() && key.time <= this->keys.back().time))
      {
        gzerr << "[SAR motion] Invalid timed route.\n";
        this->keys.clear();
        return;
      }
      this->keys.push_back(key);
    }
    if (this->keys.size() < 2 || this->keys.front().time != 0 ||
        !(this->keys.back().time > 0))
    {
      gzerr << "[SAR motion] Route must start at t=0 and have a positive period.\n";
      this->keys.clear();
      return;
    }

    // Gazebo attaches the model's world parent after Configure. Binding
    // here caused a null dereference, and later a silent kinematic fallback.
    // Bind at the first physics update instead.
    if (this->controller == 2 && config->HasElement("member"))
    {
      const std::string self = this->model.Name(_ecm);
      for (auto member = config->GetElement("member"); member;
           member = member->GetNextElement("member"))
      {
        const auto name = member->Get<std::string>("name");
        if (name.empty() || name == self) continue;  // never wait on itself
        this->formation.push_back(
            {name, member->Get<gz::math::Vector3d>("offset")});
      }
    }
    gzmsg << "[SAR motion] " << this->model.Name(_ecm) << ": "
          << (this->controller == 2 ? "contact-aware" : "kinematic") << ", "
          << this->cruiseSpeed << " m/s, " << this->keys.back().time
          << " s repeating route\n";
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->keys.empty()) return;
    const double time = std::chrono::duration<double>(_info.simTime).count();
    if (time < this->lastTime) this->phase = 0;   // the world was reset
    this->lastTime = time;
    const double dt = std::max(0.0, std::chrono::duration<double>(_info.dt).count());

    if (this->controller == 1)
    {
      this->Advance(dt, 1.0);
      // Model::SetWorldPoseCmd owns the WorldPoseCmd component, so the
      // kinematic path needs no component header of its own.
      this->model.SetWorldPoseCmd(_ecm, this->Target(this->phase, this->offset));
      return;
    }

    if (!this->physicsReady)
    {
      const auto canonical = this->model.CanonicalLink(_ecm);
      const auto parent = _ecm.Component<gz::sim::components::ParentEntity>(this->model.Entity());
      if (canonical == gz::sim::kNullEntity || !parent ||
          !_ecm.Component<gz::sim::components::Inertial>(canonical)) return;
      if (this->model.Static(_ecm))
      {
        gzerr << "[SAR motion] Contact controller requires a dynamic model.\n";
        this->keys.clear();
        return;
      }
      this->link = gz::sim::Link(canonical);
      this->link.EnableVelocityChecks(_ecm);
      this->world = parent->Data();
      this->physicsReady = true;
    }

    const auto current = gz::sim::worldPose(this->model.Entity(), _ecm);
    auto target = this->Target(this->phase, this->offset);
    auto error = target.Pos() - current.Pos();
    // A blocked body stops its route clock. It cannot jump ahead through a
    // wall or sprint to catch up with a point far along the route once the
    // obstruction clears.
    bool advance = std::hypot(error.X(), error.Y()) < 0.45;
    for (auto &member : this->formation)
    {
      if (member.entity == gz::sim::kNullEntity)
        member.entity = _ecm.EntityByComponents(gz::sim::components::Model(),
            gz::sim::components::Name(member.name),
            gz::sim::components::ParentEntity(this->world));
      if (member.entity == gz::sim::kNullEntity ||
          !_ecm.Component<gz::sim::components::Pose>(member.entity))
      {
        advance = false;
        continue;
      }
      const auto p = gz::sim::worldPose(member.entity, _ecm).Pos();
      const auto delta = this->Target(this->phase, member.offset).Pos() - p;
      advance = advance && std::hypot(delta.X(), delta.Y()) < 0.45;
    }
    if (advance)
    {
      const double speed = this->RouteSpeed(this->Wrap(this->phase));
      // Walk at a bounded brisk pace; compress long authored rests.
      this->Advance(dt, speed > 0.01 ? this->cruiseSpeed / speed : 3.0);
      target = this->Target(this->phase, this->offset);
      error = target.Pos() - current.Pos();
    }

    auto desired = error * 6.0;
    desired.Z(0);
    if (desired.Length() > this->cruiseSpeed)
      desired = desired.Normalized() * this->cruiseSpeed;
    const auto velocity =
        this->link.WorldLinearVelocity(_ecm).value_or(gz::math::Vector3d::Zero);
    auto force = (desired - velocity) * (this->mass * 10.0);
    force.Z(0);  // people stand on the terrain under gravity, not on a Z curve
    const double limit = this->mass * 8.0;
    if (force.Length() > limit) force = force.Normalized() * limit;
    if (this->carried)
      force.Z(std::clamp(this->mass * (30.0 * error.Z() - 10.0 * velocity.Z()),
                         -limit, limit));
    if (!force.IsFinite()) return;
    this->link.AddWorldForce(_ecm, force);

    // Simplified balance controller keeps the rigid walking proxy upright.
    const auto up = current.Rot().RotateVector(gz::math::Vector3d::UnitZ);
    const auto angularVelocity = this->link.WorldAngularVelocity(_ecm)
        .value_or(gz::math::Vector3d::Zero);
    auto torque = up.Cross(gz::math::Vector3d::UnitZ) * 1200.0
        - angularVelocity * 220.0;
    const double yawError =
        std::remainder(target.Rot().Yaw() - current.Rot().Yaw(), 2.0 * GZ_PI);
    torque.Z(std::clamp(80.0 * yawError - 32.0 * angularVelocity.Z(), -80.0, 80.0));
    // Velocity commands suppress ALL wrenches in Harmonic, including the
    // walking force above. Use a damped balance torque instead.
    if (torque.IsFinite())
      this->link.AddWorldWrench(_ecm, gz::math::Vector3d::Zero, torque);
  }

  private: void Advance(double dt, double rate)
  {
    if (!std::isfinite(dt) || !std::isfinite(rate)) return;
    this->phase = this->Wrap(this->phase + dt * rate);
  }
};
}
GZ_ADD_PLUGIN(sar::WaypointSystem, gz::sim::System,
              sar::WaypointSystem::ISystemConfigure,
              sar::WaypointSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(sar::WaypointSystem, "sar::WaypointSystem")
