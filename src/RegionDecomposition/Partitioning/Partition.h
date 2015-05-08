#ifndef _PARTITION_H
#define _PARTITION_H


#include "Roadmap.h"
#include "CfgTypes.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup RegionDecomposition
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class Partition
{
 public:
  typedef RoadmapGraph<CfgType, WeightType>::VID VID;
  Partition();
  Partition(Roadmap<CfgType,WeightType> *rdmp, int i);
  Partition(Roadmap<CfgType,WeightType> *rdmp, vector<VID> vid):m_rdmp(rdmp), m_id(0), m_vvid(vid){}
  Partition(Partition & p);
  ~Partition();

  Roadmap<CfgType,WeightType>* GetRoadmap() {return m_rdmp;}
  void SetRoadmap(Roadmap<CfgType,WeightType>  *rdmp){m_rdmp=rdmp;}

  BoundingBox GetBoundingBox();

  int GetID(){return m_id;}
  void SetID(int i){m_id=i;}

  vector<VID>& GetVID() {return m_vvid;}
  void SetVID(vector<VID> vid){m_vvid=vid;}

  bool operator==(Partition &a){return m_vvid==a.GetVID();}

 private:
  Roadmap<CfgType,WeightType> *m_rdmp;
  int m_id;
  vector<VID> m_vvid;
};

#endif
