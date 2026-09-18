#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/common/Console.hh>
#include <ignition/plugin/RegisterMore.hh>
#include <cmath>
#include <stdexcept>

namespace magnetic_robot {
namespace sim=ignition::gazebo;
class SteeringLimitMonitor : public sim::System,public sim::ISystemConfigure,
                             public sim::ISystemPostUpdate {
  sim::Entity joint;sim::EventManager*events=nullptr;double limit=M_PI/12;bool failed=false;
 public:
  void Configure(const sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,
                 sim::EntityComponentManager&ecm,sim::EventManager&manager) override {
    events=&manager;joint=sim::Model(e).JointByName(ecm,"front_steering_joint");
    if(joint==sim::kNullEntity)throw std::runtime_error("Missing front_steering_joint");
    limit=s->Get<double>("hard_limit",M_PI/12).first;
    if(limit<=0||limit>M_PI/12+1e-14)throw std::runtime_error("Steering hard limit exceeds 15 degrees");
    sim::Joint(joint).EnablePositionCheck(ecm);
  }
  void PostUpdate(const sim::UpdateInfo&,const sim::EntityComponentManager&ecm) override {
    auto p=sim::Joint(joint).Position(ecm);if(!p||p->empty())return;
    if(!std::isfinite(p->at(0))||std::abs(p->at(0))>limit){
      if(!failed)ignerr<<"STEERING_LIMIT_VIOLATION: "<<p->at(0)<<" rad; stopping simulation.\n";
      failed=true;events->Emit<sim::events::Stop>();
    }
  }
};

// Explicit test fixture, never present in production model/worlds.
class TestRig : public sim::System,public sim::ISystemConfigure,public sim::ISystemPreUpdate {
  sim::Entity steering,front,rear,base;double torque=0,wheelTorque=0,initialSpeed=0,pullForce=0;
  bool brake=false,started=false;double lastTime=-1;
 public:
  void Configure(const sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,
                 sim::EntityComponentManager&ecm,sim::EventManager&) override {
    auto m=sim::Model(e);steering=m.JointByName(ecm,"front_steering_joint");
    base=m.LinkByName(ecm,"base_link");
    front=m.JointByName(ecm,"front_wheel_joint");rear=m.JointByName(ecm,"rear_wheel_joint");
    torque=s->Get<double>("steering_torque",0).first;wheelTorque=s->Get<double>("wheel_torque",0).first;
    initialSpeed=s->Get<double>("initial_steering_speed",0).first;brake=s->Get<bool>("brake",false).first;
    pullForce=s->Get<double>("pull_force",0).first;
    for(auto j:{steering,front,rear}){sim::Joint(j).EnablePositionCheck(ecm);sim::Joint(j).EnableVelocityCheck(ecm);}
  }
  void PreUpdate(const sim::UpdateInfo&info,sim::EntityComponentManager&ecm) override {
    if(info.paused)return;
    double t=std::chrono::duration<double>(info.simTime).count();
    if(t<lastTime)started=false;
    lastTime=t;
    if(!started){if(initialSpeed!=0)sim::Joint(steering).ResetVelocity(ecm,{initialSpeed});started=true;}
    sim::Joint(steering).SetForce(ecm,{torque});
    if(t>=.2 && pullForce!=0)
      sim::Link(base).AddWorldWrench(ecm,{0,0,pullForce},{0,0,0});
    for(auto joint:{front,rear}){
      if(brake)sim::Joint(joint).SetVelocity(ecm,{0});
      else sim::Joint(joint).SetForce(ecm,{wheelTorque});
    }
  }
};
}
IGNITION_ADD_PLUGIN(magnetic_robot::SteeringLimitMonitor,ignition::gazebo::System,
 magnetic_robot::SteeringLimitMonitor::ISystemConfigure,
 magnetic_robot::SteeringLimitMonitor::ISystemPostUpdate)
IGNITION_ADD_PLUGIN(magnetic_robot::TestRig,ignition::gazebo::System,
 magnetic_robot::TestRig::ISystemConfigure,magnetic_robot::TestRig::ISystemPreUpdate)
