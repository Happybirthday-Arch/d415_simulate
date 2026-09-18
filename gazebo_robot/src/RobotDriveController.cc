#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/RegisterMore.hh>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
namespace magnetic_robot {
namespace sim=ignition::gazebo;
class RobotDriveController : public sim::System, public sim::ISystemConfigure,
 public sim::ISystemPreUpdate, public sim::ISystemPostUpdate {
 using Clock=std::chrono::steady_clock;
 ignition::transport::Node node;
 ignition::transport::Node::Publisher states,posePub,status;
 std::array<sim::Entity,3> joints{}; sim::Entity base{};
 std::array<std::string,3> names{{"front_wheel_joint","rear_wheel_joint","front_steering_joint"}};
 std::mutex mutex; double throttle=0,turn=0; bool armed=false,brake=true;
 Clock::time_point received{};
 double rpm=15,steerLimit=14*M_PI/180,rate=M_PI,steerRate=M_PI/6,timeout=.25;
 double wheelLimit=.8,steerTorque=5,kp=.08,ki=.8,skp=30,skd=1,ski=10,steerIntegral=0;
 double speed=0,angle=0,lastTime=-1,published=-1; std::array<double,2> integral{{0,0}};
 bool paused=false,emergency=true;
 static double ramp(double x,double y,double step){return x+std::clamp(y-x,-step,step);}
 void Command(const ignition::msgs::Vector3d&m){
   std::lock_guard<std::mutex> lock(mutex);
   if(!std::isfinite(m.x())||!std::isfinite(m.y())||!std::isfinite(m.z())) {armed=false;return;}
   received=Clock::now(); brake=m.z()>.5;
   if(brake){armed=false;throttle=turn=0;return;}
   // A neutral command is required after startup, timeout, emergency or pause.
   if(!armed && std::abs(m.x())<1e-6 && std::abs(m.y())<1e-6)armed=true;
   throttle=std::clamp(m.x(),-1.,1.);turn=std::clamp(m.y(),-1.,1.);
 }
 public:
 void Configure(const sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,
                sim::EntityComponentManager&ecm,sim::EventManager&) override {
   sim::Model model(e);base=model.LinkByName(ecm,"base_link");
   for(size_t i=0;i<3;i++){
     joints[i]=model.JointByName(ecm,names[i]);
     if(joints[i]==sim::kNullEntity)throw std::runtime_error("Drive joint missing");
     sim::Joint(joints[i]).EnablePositionCheck(ecm);sim::Joint(joints[i]).EnableVelocityCheck(ecm);
   }
   auto get=[&](const char*k,double v){return s->Get<double>(k,v).first;};
   rpm=get("rpm",rpm);steerLimit=get("steer_deg",14)*M_PI/180;
   rate=get("rpm_slew",30)*2*M_PI/60;steerRate=get("steer_slew_deg",30)*M_PI/180;
   timeout=get("timeout",timeout);wheelLimit=get("wheel_torque",wheelLimit);steerTorque=get("steer_torque",steerTorque);
   kp=get("wheel_kp",kp);ki=get("wheel_ki",ki);skp=get("steer_kp",skp);skd=get("steer_kd",skd);ski=get("steer_ki",ski);
   if(!(rpm>0&&rpm<=30&&steerLimit>0&&steerLimit<=14*M_PI/180&&rate>0&&steerRate>0&&timeout>0&&wheelLimit>0&&steerTorque>0))
     throw std::runtime_error("Invalid drive limits");
   node.Subscribe("/robot/drive_command",&RobotDriveController::Command,this);
   states=node.Advertise<ignition::msgs::Model>("/robot/joint_states");
   posePub=node.Advertise<ignition::msgs::Pose>("/robot/ground_truth");
   status=node.Advertise<ignition::msgs::Vector3d>("/robot/drive_status");
 }
 void PreUpdate(const sim::UpdateInfo&info,sim::EntityComponentManager&ecm) override {
   double t=std::chrono::duration<double>(info.simTime).count();
   double dt=std::chrono::duration<double>(info.dt).count();
   double drive=0,steer=0;
   {
     std::lock_guard<std::mutex> lock(mutex);
     if(info.paused||t<lastTime||paused){armed=false;throttle=turn=0;integral={0,0};steerIntegral=0;speed=angle=0;published=-1;}
     paused=info.paused;lastTime=t;
     if(info.paused)return;
     if(std::chrono::duration<double>(Clock::now()-received).count()>timeout)armed=false;
     emergency=!armed||brake;
     if(!emergency){drive=throttle;steer=turn;}
   }
   if(dt<=0)return;
   speed=ramp(speed,drive*rpm*2*M_PI/60,rate*dt);
   if(emergency)speed=0;
   angle=ramp(angle,steer*steerLimit,steerRate*dt);
   auto position=sim::Joint(joints[2]).Position(ecm);
   double delta=position && !position->empty()?position->at(0):0;
   // Rear axle to steering axis 61 mm, steering-axis to front axle 9 mm.
   // No-slip instantaneous geometry, including rotation of the 9 mm offset.
   double ratio=(.061*std::cos(delta)+.009)/(.061+.009*std::cos(delta));
   std::array<double,2> target{{speed,speed*ratio}};
   for(size_t i=0;i<3;i++){
     auto v=sim::Joint(joints[i]).Velocity(ecm);double velocity=v&&!v->empty()?v->at(0):0;
     double force=0;
     if(i<2){
       double error=target[i]-velocity;
       double candidate=integral[i]+ki*error*dt;
       double raw=kp*error+candidate;
       // Conditional integration prevents windup at motor saturation.
       if(std::abs(raw)<=wheelLimit || raw*error<0)integral[i]=candidate;
       integral[i]=std::clamp(integral[i],-wheelLimit,wheelLimit);
       force=std::clamp(kp*error+integral[i],-wheelLimit,wheelLimit);
     }else {
       double error=angle-delta;
       double candidate=steerIntegral+ski*error*dt;
       double raw=skp*error-skd*velocity+candidate;
       if(std::abs(raw)<=steerTorque || raw*error<0)steerIntegral=candidate;
       steerIntegral=std::clamp(steerIntegral,-steerTorque,steerTorque);
       force=std::clamp(skp*error-skd*velocity+steerIntegral,-steerTorque,steerTorque);
     }
     if(!std::isfinite(force))throw std::runtime_error("Non-finite drive state");
     sim::Joint(joints[i]).SetForce(ecm,{force});
   }
 }
 void PostUpdate(const sim::UpdateInfo&info,const sim::EntityComponentManager&ecm) override {
   double t=std::chrono::duration<double>(info.simTime).count();
   if(info.paused || t-published<.01)return;
   published=t;
   auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
   ignition::msgs::Model message;message.set_name("magnetic_robot");
   auto stamp=message.mutable_header()->mutable_stamp();stamp->set_sec(ns/1000000000);stamp->set_nsec(ns%1000000000);
   for(size_t i=0;i<3;i++){
     auto j=message.add_joint();j->set_name(names[i]);
     auto p=sim::Joint(joints[i]).Position(ecm),v=sim::Joint(joints[i]).Velocity(ecm);
     j->mutable_axis1()->set_position(p&&!p->empty()?p->at(0):0);
     j->mutable_axis1()->set_velocity(v&&!v->empty()?v->at(0):0);
   }
   states.Publish(message);
   ignition::msgs::Pose pose;ignition::msgs::Set(&pose,sim::worldPose(base,ecm));
   *pose.mutable_header()->mutable_stamp()=*stamp;
   auto frame=pose.mutable_header()->add_data();frame->set_key("frame_id");frame->add_value("world");
   posePub.Publish(pose);
   ignition::msgs::Vector3d state;state.set_x(speed*60/(2*M_PI));state.set_y(angle*180/M_PI);state.set_z(emergency?1:0);status.Publish(state);
 }
};
}
IGNITION_ADD_PLUGIN(magnetic_robot::RobotDriveController,ignition::gazebo::System,
 magnetic_robot::RobotDriveController::ISystemConfigure,
 magnetic_robot::RobotDriveController::ISystemPreUpdate,
 magnetic_robot::RobotDriveController::ISystemPostUpdate)
