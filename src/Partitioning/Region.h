#ifndef _REGION_H
#define _REGION_H
#include "Roadmap.h"
#include "CfgTypes.h"
#include "Boundary.h"

//region class will save the associated bounding volume and the CC information in the region
typedef RoadmapGraph<CfgType, WeightType>::VID RVID;

class Region {
 public:
 
 
  Region();
  
  Region(const Region& _r);
  Region(shared_ptr<BoundingBox> _bbox, std::vector<pair<size_t, RVID> >& _pairIds ); //second input: vector of pair (cc size, vid of cc representative)
    
  ~Region();
  shared_ptr<BoundingBox> GetBoundingBox() const;
  void SetBoundingBox(shared_ptr<BoundingBox> _bb);
  pair <shared_ptr<BoundingBox>, vector<pair<size_t, RVID> > > GetRegionInfo() const;
 
  bool operator==(Region &_a);
  std::vector<RVID> GetAllVIDs() const; //returns all the vids from all the CCs of the region

  std::vector<pair<size_t, RVID> > GetCCs() const; //returns the vector of pair (size of cc, vid of cc representative)
  void SetCCs(vector<pair<size_t, RVID> >& _ccs);
  
  protected:
     
  shared_ptr<BoundingBox> m_bb;
  vector<pair<size_t, RVID> > m_ccs;//vector of pair (cc size, vid of cc representative)
   

};

#endif 
