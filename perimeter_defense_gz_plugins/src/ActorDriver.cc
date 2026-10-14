// ActorDriver: moves an animated actor with a commanded planar velocity.
//
// The actor walks or runs in the commanded direction, with the skeleton
// animation matched to the distance travelled (same approach as the
// gz-sim FollowActor system). When the command is zero or stale the actor
// stands still with an idle animation. The true actor pose is published for
// the simulation side (scenario scripting, evaluation); the perimeter defense
// algorithms never use it.
//
// SDF parameters (all optional):
//   <cmd_topic>         gz.msgs.Vector3d world-frame velocity (x, y), default /actor/<name>/cmd_vel
//   <pose_topic>        gz.msgs.Pose true pose,                     default /actor/<name>/pose
//   <walk_animation>    animation name used below <run_speed>,      default walk
//   <run_animation>     animation name used from <run_speed> on,    default run
//   <idle_animation>    animation name used when stopped,           default stand
//   <run_speed>         m/s, default 1.9
//   <walk_anim_factor>  animation seconds per metre walked, default 2.2
//   <run_anim_factor>   animation seconds per metre run,    default 1.1
//   <cmd_timeout>       s, default 0.5
//   <pose_rate>         Hz, default 20

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace perimeter_defense
{
using namespace gz;
using namespace gz::sim;

class ActorDriver
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->actor = _entity;
    auto actorComp = _ecm.Component<components::Actor>(_entity);
    if (!actorComp)
    {
      gzerr << "ActorDriver must be attached to an actor." << std::endl;
      return;
    }
    auto nameComp = _ecm.Component<components::Name>(_entity);
    const std::string name = nameComp ? nameComp->Data() : "actor";

    auto get = [&](const char *key, auto fallback) {
      return _sdf->HasElement(key) ? _sdf->Get<decltype(fallback)>(key) : fallback;
    };
    const std::string cmdTopic = get("cmd_topic", std::string("/actor/" + name + "/cmd_vel"));
    const std::string poseTopic = get("pose_topic", std::string("/actor/" + name + "/pose"));
    this->walkAnim = get("walk_animation", std::string("walk"));
    this->runAnim = get("run_animation", std::string("run"));
    this->idleAnim = get("idle_animation", std::string("stand"));
    this->runSpeed = get("run_speed", 1.9);
    this->walkFactor = get("walk_anim_factor", 2.2);
    this->runFactor = get("run_anim_factor", 1.1);
    this->cmdTimeout = get("cmd_timeout", 0.5);
    this->posePeriod = 1.0 / get("pose_rate", 20.0);

    // Start from the spawn pose, controlled in 2D through the trajectory pose.
    // The rendered pose is Pose * TrajectoryPose, so the Pose component keeps
    // only the height; x, y and yaw all live in the trajectory pose.
    math::Pose3d start;
    if (auto poseComp = _ecm.Component<components::Pose>(_entity))
    {
      start = poseComp->Data();
      *poseComp = components::Pose(math::Pose3d(0, 0, start.Pos().Z(), 0, 0, 0));
    }
    start.Pos().Z(0);
    this->pose = start;
    _ecm.CreateComponent(_entity, components::TrajectoryPose(start));
    _ecm.CreateComponent(_entity, components::AnimationTime());
    this->SetAnimation(_ecm, this->idleAnim);

    this->node.Subscribe(cmdTopic, &ActorDriver::OnCmd, this);
    this->posePub = this->node.Advertise<msgs::Pose>(poseTopic);
    gzmsg << "ActorDriver [" << name << "] cmd on " << cmdTopic
          << ", pose on " << poseTopic << std::endl;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->actor == kNullEntity)
      return;
    const double now = std::chrono::duration<double>(_info.simTime).count();
    const double dt = std::chrono::duration<double>(_info.dt).count();

    // Commands are stamped with simulation time on arrival, so the timeout
    // also works when the simulation runs slower than real time.
    math::Vector3d vel;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->fresh)
      {
        this->lastCmdTime = now;
        this->fresh = false;
      }
      if (this->lastCmdTime >= 0.0 && now - this->lastCmdTime < this->cmdTimeout)
        vel = this->cmdVel;
    }
    vel.Z(0);
    const double speed = vel.Length();

    double animDt = dt;   // idle animation plays in real time
    if (speed > 0.05)
    {
      this->pose.Pos() += vel * dt;
      // Turn smoothly towards the direction of motion.
      const double target = std::atan2(vel.Y(), vel.X());
      const double yaw = this->pose.Rot().Yaw();
      double err = std::atan2(std::sin(target - yaw), std::cos(target - yaw));
      const double maxTurn = 6.0 * dt;
      err = std::max(-maxTurn, std::min(maxTurn, err));
      this->pose.Rot() = math::Quaterniond(0, 0, yaw + err);
      const bool running = speed >= this->runSpeed;
      this->SetAnimation(_ecm, running ? this->runAnim : this->walkAnim);
      animDt = speed * dt * (running ? this->runFactor : this->walkFactor);
    }
    else
    {
      this->SetAnimation(_ecm, this->idleAnim);
    }

    auto trajComp = _ecm.Component<components::TrajectoryPose>(this->actor);
    *trajComp = components::TrajectoryPose(this->pose);
    _ecm.SetChanged(this->actor, components::TrajectoryPose::typeId,
                    ComponentState::OneTimeChange);

    auto animComp = _ecm.Component<components::AnimationTime>(this->actor);
    *animComp = components::AnimationTime(animComp->Data() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(animDt)));
    _ecm.SetChanged(this->actor, components::AnimationTime::typeId,
                    ComponentState::OneTimeChange);

    if (now - this->lastPosePub >= this->posePeriod)
    {
      this->lastPosePub = now;
      msgs::Pose msg;
      msg.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(now));
      msg.mutable_header()->mutable_stamp()->set_nsec(
          static_cast<int32_t>((now - std::floor(now)) * 1e9));
      msg.mutable_position()->set_x(this->pose.Pos().X());
      msg.mutable_position()->set_y(this->pose.Pos().Y());
      msg.mutable_position()->set_z(0.0);
      const auto &q = this->pose.Rot();
      msg.mutable_orientation()->set_w(q.W());
      msg.mutable_orientation()->set_x(q.X());
      msg.mutable_orientation()->set_y(q.Y());
      msg.mutable_orientation()->set_z(q.Z());
      this->posePub.Publish(msg);
    }
  }

private:
  void OnCmd(const msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->cmdVel.Set(_msg.x(), _msg.y(), 0.0);
    this->fresh = true;
  }

  void SetAnimation(EntityComponentManager &_ecm, const std::string &_name)
  {
    if (_name == this->currentAnim)
      return;
    this->currentAnim = _name;
    auto comp = _ecm.Component<components::AnimationName>(this->actor);
    if (comp)
      *comp = components::AnimationName(_name);
    else
      _ecm.CreateComponent(this->actor, components::AnimationName(_name));
    _ecm.SetChanged(this->actor, components::AnimationName::typeId,
                    ComponentState::OneTimeChange);
  }

  Entity actor{kNullEntity};
  math::Pose3d pose;
  transport::Node node;
  transport::Node::Publisher posePub;
  std::mutex mutex;
  math::Vector3d cmdVel;
  bool fresh{false};
  double lastCmdTime{-1.0};
  std::string walkAnim, runAnim, idleAnim, currentAnim;
  double runSpeed{1.9}, walkFactor{2.2}, runFactor{1.1};
  double cmdTimeout{0.5}, posePeriod{0.05}, lastPosePub{-1.0};
};
}  // namespace perimeter_defense

GZ_ADD_PLUGIN(perimeter_defense::ActorDriver, gz::sim::System,
              perimeter_defense::ActorDriver::ISystemConfigure,
              perimeter_defense::ActorDriver::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(perimeter_defense::ActorDriver, "perimeter_defense::ActorDriver")
