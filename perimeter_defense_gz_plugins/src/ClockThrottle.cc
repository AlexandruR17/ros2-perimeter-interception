// Simulation clock at a lower rate for ROS.
//
// Gazebo publishes /clock at every physics step (250 Hz with a 4 ms step).
// Bridged to ROS, every node that uses simulation time receives and decodes
// all of them: with three robots that is about 35 processes, most of them
// Python, and a large share of the CPU. This world system publishes the
// simulation time on <topic> (default /clock_ros) every <period> seconds of
// simulation time (default 0.02 s, 50 Hz), and the ROS bridge forwards that
// topic to /clock instead. 20 ms is far below the periods of the ROS nodes
// (Nav2 controller 50 ms, allocator 300 ms); sensor messages keep their own
// exact simulation time stamps.
//
//   <plugin filename="ClockThrottle" name="perimeter_defense::ClockThrottle">
//     <topic>/clock_ros</topic>
//     <period>0.02</period>
//   </plugin>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <gz/msgs/clock.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace perimeter_defense
{
using namespace gz;
using namespace gz::sim;

class ClockThrottle
  : public System,
    public ISystemConfigure,
    public ISystemPostUpdate
{
public:
  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &, EventManager &) override
  {
    std::string topic = "/clock_ros";
    if (_sdf->HasElement("topic"))
      topic = _sdf->Get<std::string>("topic");
    if (_sdf->HasElement("period"))
      this->period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(_sdf->Get<double>("period")));
    this->pub = this->node.Advertise<msgs::Clock>(topic);
  }

  void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &) override
  {
    // Publish the first step, then once per period, also while paused (the
    // ROS nodes then see a stopped clock instead of no clock).
    if (this->published && _info.simTime - this->last < this->period && !_info.paused)
      return;
    if (this->published && _info.paused && _info.simTime == this->last)
      return;
    this->last = _info.simTime;
    this->published = true;
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
    msgs::Clock msg;
    msg.mutable_sim()->set_sec(static_cast<int64_t>(ns / 1000000000));
    msg.mutable_sim()->set_nsec(static_cast<int32_t>(ns % 1000000000));
    this->pub.Publish(msg);
  }

private:
  transport::Node node;
  transport::Node::Publisher pub;
  std::chrono::steady_clock::duration period{std::chrono::milliseconds(20)};
  std::chrono::steady_clock::duration last{0};
  bool published{false};
};
}  // namespace perimeter_defense

GZ_ADD_PLUGIN(perimeter_defense::ClockThrottle, gz::sim::System,
              perimeter_defense::ClockThrottle::ISystemConfigure,
              perimeter_defense::ClockThrottle::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(perimeter_defense::ClockThrottle, "perimeter_defense::ClockThrottle")
