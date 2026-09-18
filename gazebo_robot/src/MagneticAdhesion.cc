#include "SurfaceMesh.hh"
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <cstdlib>
#include <filesystem>
#include <iomanip>

namespace magnetic_robot {
namespace sim=ignition::gazebo;
class MagneticAdhesion : public sim::System,public sim::ISystemConfigure,
                         public sim::ISystemPreUpdate,public sim::ISystemPostUpdate {
  struct Target {std::string name;SurfaceMesh mesh;sim::Entity entity=sim::kNullEntity;};
  struct Wheel {sim::Entity entity,carrier;bool active=false;double force=0,gap=1;V vector=V::Zero;};
  std::vector<Target> targets;std::vector<Wheel>wheels;
  sim::Entity robot,steering,frontJoint,rearJoint;
  double force=200,capture=.0005,release=.001;bool ready=false;
  std::ofstream csv;size_t rows=0;
 public:
  void Configure(const sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,
                 sim::EntityComponentManager&ecm,sim::EventManager&) override {
    robot=e;auto model=sim::Model(e);
    force=s->Get<double>("force_per_wheel",200).first;capture=s->Get<double>("capture_gap",.0005).first;release=s->Get<double>("release_gap",.001).first;
    if(force<=0||capture<0||release<capture)throw std::runtime_error("Invalid adhesion configuration");
    const char* root=std::getenv("MAGNETIC_ROBOT_RESOURCE_ROOT");
    if(!root)throw std::runtime_error("MAGNETIC_ROBOT_RESOURCE_ROOT is required; use gazebo_robot/run.sh");
    auto add=[&](const std::string&name,const std::string&file){Target t;t.name=name;t.mesh.Load((std::filesystem::path(root)/file).string());targets.push_back(std::move(t));};
    if(s->HasElement("surface")){
      auto elem=s->FindElement("surface");while(elem){add(elem->Get<std::string>("model"),elem->Get<std::string>("file"));elem=elem->GetNextElement("surface");}
    }else add(s->Get<std::string>("default_surface_model"),s->Get<std::string>("default_surface_file"));
    for(auto name:{"front","rear"}) {
      std::string prefix=name;
      auto wheel=model.LinkByName(ecm,prefix+"_wheel_link");
      auto carrier=model.LinkByName(ecm,prefix=="front"?"front_steering_link":"base_link");
      if(wheel==sim::kNullEntity||carrier==sim::kNullEntity)throw std::runtime_error("Missing wheel or carrier link");
      wheels.push_back({wheel,carrier});
    }
    steering=model.JointByName(ecm,"front_steering_joint");frontJoint=model.JointByName(ecm,"front_wheel_joint");rearJoint=model.JointByName(ecm,"rear_wheel_joint");
    for(auto joint:{steering,frontJoint,rearJoint})sim::Joint(joint).EnablePositionCheck(ecm);
    if(const char*path=std::getenv("MAGNETIC_ROBOT_DIAGNOSTICS")){
      csv.open(path);if(!csv)throw std::runtime_error("Cannot write diagnostics");
      csv<<"time,steer,front_angle,rear_angle,x,y,z,roll,pitch,yaw,front_force,rear_force,front_gap,rear_gap,front_fx,front_fy,front_fz,rear_fx,rear_fy,rear_fz,contacts\n"<<std::setprecision(12);
    }
    ignmsg<<"MagneticAdhesion configured: "<<force<<" N per wheel, "<<targets.size()<<" finite surface set(s).\n";
  }
  void PreUpdate(const sim::UpdateInfo&info,sim::EntityComponentManager&ecm) override {
    if(info.paused)return;
    if(!ready){
      for(auto &t:targets){
        if(t.name=="__no_magnetic_target__")continue;
        t.entity=ecm.EntityByComponents(sim::components::Model(),sim::components::Name(t.name));
        if(t.entity==sim::kNullEntity)throw std::runtime_error("Magnetic target model not found: "+t.name);
      }
      ready=true;
    }
    if(info.dt<=decltype(info.dt)::zero()){
      for(auto&w:wheels){w.active=false;w.force=0;w.vector=V::Zero;}
      return;
    }
    for(size_t wi=0;wi<wheels.size();++wi){
      auto &w=wheels[wi];auto wp=sim::worldPose(w.entity,ecm),cp=sim::worldPose(w.carrier,ecm);
      // Carrier normal and wheel axle are invariant under wheel spin.
      const V axle=wp.Rot().RotateVector(V::UnitY).Normalized();
      const V down=cp.Rot().RotateVector(-V::UnitZ).Normalized();
      const double limit=.0315+(w.active?release:capture);
      struct Sample {V point,normal;double gap;};std::vector<Sample> samples;
      w.force=0;w.vector=V::Zero;w.gap=1;
      // Two samples in each tread band; total wheel force is normalized once.
      for(double localY:{-.014,-.006,.006,.014}){
        double y=wi==0?localY:-localY;V origin=wp.Pos()+axle*y;
        Hit best{limit,V::Zero,V::Zero};
        for(const auto &t:targets){
          if(t.entity==sim::kNullEntity)continue;
          auto tp=sim::worldPose(t.entity,ecm);
          V localOrigin=tp.Rot().RotateVectorReverse(origin-tp.Pos());
          V localDir=tp.Rot().RotateVectorReverse(down);
          auto h=t.mesh.Ray(localOrigin,localDir,best.distance);
          if(h.normal.Length()>0){best.distance=h.distance;best.normal=tp.Rot().RotateVector(h.normal);best.point=tp.Pos()+tp.Rot().RotateVector(h.point);}
        }
        if(best.normal.Length()==0)continue;
        // Reject side faces and gross penetration; normals point from steel to wheel.
        if(best.normal.Dot(-down)<.7||std::abs(best.normal.Dot(axle))>.35)continue;
        const double gap=best.distance-.0315;
        if(gap<-.002)continue;
        samples.push_back({origin+down*.0315,best.normal,gap});w.gap=std::min(w.gap,gap);
      }
      w.active=!samples.empty();if(!w.active)continue;
      for(const auto &s:samples){
        V f=-s.normal*(force/samples.size());
        // Fortress AddWorldForce silently does nothing without an opt-in
        // WorldPose component, and its offset is COM-relative. Compute the
        // world wrench about the link origin explicitly to avoid ambiguity.
        sim::Link(w.entity).AddWorldWrench(ecm,f,(s.point-wp.Pos()).Cross(f));
        w.vector+=f;
      }
      w.force=w.vector.Length();
    }
  }
  void PostUpdate(const sim::UpdateInfo&info,const sim::EntityComponentManager&ecm) override {
    if(info.paused||!csv)return;
    auto position=[&](sim::Entity e){auto p=sim::Joint(e).Position(ecm);return p&&!p->empty()?p->at(0):0.;};
    auto p=sim::worldPose(sim::Model(robot).LinkByName(ecm,"base_link"),ecm);auto r=p.Rot().Euler();
    int contacts=0;
    ecm.Each<sim::components::ContactSensorData,sim::components::ParentEntity>([&](const sim::Entity&,const auto*c,const auto*parent){for(auto&w:wheels)if(parent->Data()==w.entity)contacts+=c->Data().contact_size();return true;});
    csv<<std::chrono::duration<double>(info.simTime).count()<<','<<position(steering)<<','<<position(frontJoint)<<','<<position(rearJoint)<<','<<p.Pos().X()<<','<<p.Pos().Y()<<','<<p.Pos().Z()<<','<<r.X()<<','<<r.Y()<<','<<r.Z();
    for(auto&w:wheels)csv<<','<<w.force;
    for(auto&w:wheels)csv<<','<<w.gap;
    for(auto&w:wheels)csv<<','<<w.vector.X()<<','<<w.vector.Y()<<','<<w.vector.Z();
    csv<<','<<contacts<<'\n';if(++rows%1000==0)csv.flush();
  }
};
}
IGNITION_ADD_PLUGIN(magnetic_robot::MagneticAdhesion,ignition::gazebo::System,
 magnetic_robot::MagneticAdhesion::ISystemConfigure,
 magnetic_robot::MagneticAdhesion::ISystemPreUpdate,
 magnetic_robot::MagneticAdhesion::ISystemPostUpdate)
