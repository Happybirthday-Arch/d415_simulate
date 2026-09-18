#include "SurfaceMesh.hh"
#include <iostream>
int main() {
  using namespace magnetic_robot;
  SurfaceMesh mesh;
  mesh.Set({{V(-1,-1,0),V(1,-1,0),V(1,1,0)}, {V(-1,-1,0),V(1,1,0),V(-1,1,0)}});
  auto a=mesh.Ray(V(0,0,.0316),V(0,0,-1),.04);
  auto b=mesh.Ray(V(0,0,-.0316),V(0,0,1),.04);
  auto c=mesh.Ray(V(2,0,.0316),V(0,0,-1),.04);
  auto d=mesh.Ray(V(0,0,.05),V(0,0,-1),.04);
  if(std::abs(a.distance-.0316)>1e-9||a.normal.Z()<.99||b.normal.Z()>-.99||c.normal.Length()>0||d.normal.Length()>0)return 1;
  std::cout<<"finite face, both sides, edge and range queries passed\n";
}
