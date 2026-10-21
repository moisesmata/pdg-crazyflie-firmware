// Read-only DataStream diagnostic: lists subject names and segment visibility.
#include "DataStreamClient.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>
using namespace ViconDataStreamSDK::CPP;
int main(int argc, char** argv) {
  Client c;
  auto result=c.Connect(argc>1?argv[1]:"127.0.0.1:801");
  std::cout<<"Connect result="<<result.Result<<std::endl;
  if(!c.IsConnected().Connected) return 1;
  c.EnableSegmentData();
  c.SetAxisMapping(Direction::Forward,Direction::Left,Direction::Up);
  c.SetStreamMode(StreamMode::ServerPush);
  unsigned samples=0;
  for(int attempt=0; attempt<500; ++attempt) {
    if(c.GetFrame().Result!=Result::Success) {std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;}
    auto count=c.GetSubjectCount();
    std::cout<<"Frame="<<c.GetFrameNumber().FrameNumber<<" subjects="<<count.SubjectCount<<std::endl;
    for(unsigned i=0;i<count.SubjectCount;++i) {
      auto name=c.GetSubjectName(i).SubjectName;
      std::cout<<"subject="<<std::string(name)<<" root="<<std::string(c.GetSubjectRootSegmentName(name).SegmentName)<<std::endl;
      for(unsigned j=0;j<c.GetSegmentCount(name).SegmentCount;++j){
        auto segment=c.GetSegmentName(name,j).SegmentName;
        auto p=c.GetSegmentGlobalTranslation(name,segment);
        auto q=c.GetSegmentGlobalRotationQuaternion(name,segment);
        std::cout<<" segment="<<std::string(segment)<<" result="<<p.Result<<" occluded="<<p.Occluded<<" position_mm="<<p.Translation[0]<<","<<p.Translation[1]<<","<<p.Translation[2]<<std::endl;
        if(q.Result==Result::Success && !q.Occluded){
          double x=q.Rotation[0],y=q.Rotation[1],z=q.Rotation[2],w=q.Rotation[3];
          double roll=std::atan2(2*(w*x+y*z),1-2*(x*x+y*y));
          double pitch=std::asin(std::clamp(2*(w*y-z*x),-1.0,1.0));
          double yaw=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
          std::cout<<" rpy_deg="<<roll*180/3.141592653589793<<","<<pitch*180/3.141592653589793<<","<<yaw*180/3.141592653589793<<std::endl;
        }
      }
    }
    if(++samples >= 5) { c.Disconnect(); return count.SubjectCount?0:2; }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return 3;
}
