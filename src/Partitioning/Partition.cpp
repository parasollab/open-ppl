#include "Partition.h"

Partition::Partition(){
  m_id =0;
};

Partition::Partition(Roadmap<CfgType,WeightType> *rdmp,int i):m_rdmp(rdmp),m_id(i){
  rdmp->m_pRoadmap->GetVerticesVID(m_vvid);
};

Partition::Partition(Partition & p){
   m_vvid=vector<VID>(p.GetVID());
   m_rdmp=p.GetRoadmap();
   m_id=p.GetID();
};

Partition::~Partition(){
  //fill in for delete m_nodes
};

BoundingBox Partition::GetBoundingBox(){
  BoundingBox bb(3,3);
  double minX=1e6, maxX=-1e6, minY=1e6, maxY=-1e6, minZ=1e6, maxZ=-1e6;
  for(size_t i=0; i<m_vvid.size() ; i++){
     CfgType cfg = m_rdmp->m_pRoadmap->find_vertex(m_vvid[i])->property();
    vector<double> position = cfg.GetPosition();
    if(position[0]<minX)minX=position[0];
    if(position[0]>maxX)maxX=position[0];
    if(position[1]<minY)minY=position[1];
    if(position[1]>maxY)maxY=position[1];
    if(position[2]<minZ)minZ=position[2];
    if(position[2]>maxZ)maxZ=position[2];
  }
  bb.SetParameter(0,minX,maxX);
  bb.SetParameter(1,minY,maxY);
  bb.SetParameter(2,minZ,maxZ);
  return bb;
};


