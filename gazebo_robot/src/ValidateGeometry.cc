#include <fcl/fcl.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <vector>
#include <string>
#include <limits>

using Mesh=fcl::BVHModel<fcl::OBBRSSd>;
std::shared_ptr<Mesh> Load(const std::string &path) {
  std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error(path);
  f.seekg(80);uint32_t n=0;f.read(reinterpret_cast<char*>(&n),4);
  std::vector<fcl::Vector3d> v;std::vector<fcl::Triangle> faces;
  for(uint32_t i=0;i<n;++i){float d[12];uint16_t attr;f.read(reinterpret_cast<char*>(d),48);f.read(reinterpret_cast<char*>(&attr),2);if(!f)throw std::runtime_error("Truncated STL");for(int k=0;k<3;++k)v.emplace_back(d[3+3*k],d[4+3*k],d[5+3*k]);faces.emplace_back(3*i,3*i+1,3*i+2);}
  auto m=std::make_shared<Mesh>();m->beginModel();m->addSubModel(v,faces);m->endModel();return m;
}
int main(int argc,char**argv) {
  if(argc<3||argc>4){std::cerr<<"usage: validate_geometry validation_dir output.json [collision]\n";return 2;}
  std::map<std::string,std::shared_ptr<Mesh>> meshes;
  for(auto n:{"front_wheel","rear_wheel","front_motor","rear_motor","robot_chassis","front_steering"})meshes[n]=Load(std::string(argv[1])+"/"+n+(argc==4?"_collision_model.stl":"_model.stl"));
  // Adjacent rigid/hinge parts have intentional assembly contacts; scan separated pairs.
  std::vector<std::pair<std::string,std::string>> pairs={{"front_wheel","rear_wheel"},{"front_motor","rear_wheel"},{"front_wheel","rear_motor"},{"front_motor","rear_motor"},{"front_wheel","robot_chassis"},{"front_motor","robot_chassis"},{"front_steering","rear_wheel"}};
  std::ofstream out(argv[2]);out<<std::setprecision(12)<<"{\n  \"angle_step_deg\": 0.1,\n  \"pairs\": [\n";bool pass=true;
  fcl::Vector3d pivot(.026000008737,0,.046999994064);
  for(size_t i=0;i<pairs.size();++i){auto names=pairs[i];double minimum=1e9,angleMin=0;bool collided=false;
    for(int ai=-150;ai<=150;++ai){double angle=ai*.1*M_PI/180;
      fcl::Transform3d transform=fcl::Transform3d::Identity();transform.linear()=Eigen::AngleAxisd(angle,fcl::Vector3d::UnitZ()).toRotationMatrix();transform.translation()=pivot-transform.linear()*pivot;
      fcl::CollisionObjectd a(meshes[names.first],transform),b(meshes[names.second]);
      fcl::CollisionRequestd cr;fcl::CollisionResultd result;fcl::collide(&a,&b,cr,result);
      if(result.isCollision())collided=true;
      fcl::DistanceRequestd req;req.enable_nearest_points=true;fcl::DistanceResultd distance;fcl::distance(&a,&b,req,distance);
      if(distance.min_distance<minimum){minimum=distance.min_distance;angleMin=ai*.1;}
    }
    pass&=!collided;
    out<<"    {\"a\":\""<<names.first<<"\",\"b\":\""<<names.second<<"\",\"min_gap_m\":"<<minimum<<",\"angle_deg\":"<<angleMin<<",\"collision\":"<<(collided?"true":"false")<<"}"<<(i+1<pairs.size()?",":"")<<"\n";
    std::cout<<names.first<<" / "<<names.second<<": "<<minimum*1000<<" mm, collision="<<collided<<"\n";
  }
  out<<"  ],\n  \"passed\": "<<(pass?"true":"false")<<"\n}\n";return pass?0:1;
}
