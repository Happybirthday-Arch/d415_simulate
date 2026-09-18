#pragma once
#include <ignition/math/Vector3.hh>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace magnetic_robot {
using V = ignition::math::Vector3d;
struct Triangle { V a, b, c; };
struct Hit { double distance; V normal; V point; };

// A static triangle BVH. Queries return the first finite surface intersection,
// so opposite faces of a plate and holes cannot act as infinite support planes.
class SurfaceMesh {
  struct Node { V lo, hi; int left=-1, right=-1; size_t begin=0,end=0; };
  std::vector<Triangle> triangles;
  std::vector<size_t> order;
  std::vector<Node> nodes;
  static V Low(const Triangle &t) { return V(std::min({t.a.X(),t.b.X(),t.c.X()}),std::min({t.a.Y(),t.b.Y(),t.c.Y()}),std::min({t.a.Z(),t.b.Z(),t.c.Z()})); }
  static V High(const Triangle &t) { return V(std::max({t.a.X(),t.b.X(),t.c.X()}),std::max({t.a.Y(),t.b.Y(),t.c.Y()}),std::max({t.a.Z(),t.b.Z(),t.c.Z()})); }
  int Build(size_t begin,size_t end) {
    Node n; n.begin=begin;n.end=end;n.lo=V(1e30,1e30,1e30);n.hi=-n.lo;
    for(size_t i=begin;i<end;++i) { auto lo=Low(triangles[order[i]]),hi=High(triangles[order[i]]);for(int k=0;k<3;++k){n.lo[k]=std::min(n.lo[k],lo[k]);n.hi[k]=std::max(n.hi[k],hi[k]);} }
    int index=nodes.size();nodes.push_back(n);
    if(end-begin>8) {
      V d=n.hi-n.lo;int axis=d.X()>d.Y()?0:1;if(d.Z()>d[axis])axis=2;
      size_t mid=(begin+end)/2;
      std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,[&](size_t a,size_t b){const auto &ta=triangles[a],&tb=triangles[b];return (ta.a[axis]+ta.b[axis]+ta.c[axis])<(tb.a[axis]+tb.b[axis]+tb.c[axis]);});
      int left=Build(begin,mid),right=Build(mid,end);nodes[index].left=left;nodes[index].right=right;
    }
    return index;
  }
  bool Bounds(const Node &n,const V &o,const V &d,double max) const {
    double low=0,high=max;
    for(int k=0;k<3;++k) {
      if(std::abs(d[k])<1e-14){if(o[k]<n.lo[k]-1e-10||o[k]>n.hi[k]+1e-10)return false;continue;}
      double a=(n.lo[k]-o[k])/d[k],b=(n.hi[k]-o[k])/d[k];if(a>b)std::swap(a,b);
      low=std::max(low,a);high=std::min(high,b);if(low>high+1e-10)return false;
    }
    return true;
  }
  void Query(int index,const V&o,const V&d,Hit &hit) const {
    const auto &n=nodes[index];if(!Bounds(n,o,d,hit.distance))return;
    if(n.left>=0){Query(n.left,o,d,hit);Query(n.right,o,d,hit);return;}
    for(size_t i=n.begin;i<n.end;++i) {
      const auto &t=triangles[order[i]];V e1=t.b-t.a,e2=t.c-t.a,p=d.Cross(e2);double det=e1.Dot(p);
      if(std::abs(det)<1e-15)continue;
      V s=o-t.a;double u=s.Dot(p)/det;if(u<-1e-9||u>1+1e-9)continue;
      V q=s.Cross(e1);double v=d.Dot(q)/det;if(v<-1e-9||u+v>1+1e-9)continue;
      double distance=e2.Dot(q)/det;if(distance<0||distance>=hit.distance)continue;
      V normal=e1.Cross(e2).Normalized();if(normal.Dot(d)>0)normal=-normal;
      hit={distance,normal,o+distance*d};
    }
  }
 public:
  void Set(std::vector<Triangle> ts){triangles=std::move(ts);if(triangles.empty())throw std::runtime_error("Empty magnetic surface");order.resize(triangles.size());std::iota(order.begin(),order.end(),0);nodes.clear();Build(0,order.size());}
  void Load(const std::string &path) {
    std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("Cannot open magnetic mesh: "+path);
    f.seekg(80);uint32_t count=0;f.read(reinterpret_cast<char*>(&count),4);
    if(count==0||count>10000000)throw std::runtime_error("Invalid STL count: "+path);
    std::vector<Triangle> ts;ts.reserve(count);
    for(uint32_t i=0;i<count;++i){float data[12];uint16_t attr;f.read(reinterpret_cast<char*>(data),48);f.read(reinterpret_cast<char*>(&attr),2);if(!f)throw std::runtime_error("Truncated STL: "+path);ts.push_back({V(data[3],data[4],data[5]),V(data[6],data[7],data[8]),V(data[9],data[10],data[11])});}
    Set(std::move(ts));
  }
  Hit Ray(const V&o,const V&direction,double max) const {
    Hit hit{max,V::Zero,V::Zero};if(!nodes.empty())Query(0,o,direction.Normalized(),hit);return hit;
  }
  size_t Size()const{return triangles.size();}
};
}
