#include "Region.h"
Region::Region(){ }
Region::~Region(){ }

Region::Region(const Region& _r){
  m_bb=_r.m_bb;
  m_ccs=_r.m_ccs;
}

Region::Region(shared_ptr<BoundingBox> _bbox, std::vector<pair<size_t, RVID> >& _pairIds ){
  m_bb=_bbox;
  m_ccs.clear();
  m_ccs=_pairIds;
}
  
shared_ptr<BoundingBox> 
Region::GetBoundingBox() const{
  return m_bb;
}

std::vector<pair<size_t, RVID>  > Region::GetCCs() const{
  return m_ccs;
}

pair <shared_ptr<BoundingBox>, vector<pair<size_t, RVID> > >
Region::GetRegionInfo() const{
  pair <shared_ptr<BoundingBox>, vector<pair<size_t, RVID> > > infoPair = make_pair(m_bb,m_ccs );
  return info_pair;
}
  
void
Region::SetBoundingBox(shared_ptr<BoundingBox> _bb){
  m_bb=_bb;
}
  
std::vector<RVID>
Region::GetAllVIDs() const{
  vector<RVID> allVIDs;
  return allVIDs;
} 

bool 
Region::operator==(Region &_a){
  return (m_bb == _a.m_bb) &&
  (m_ccs == _a.m_ccs) ;
}  

void
Region::SetCCs(vector <pair<size_t, RVID> >& _ccs){
  m_ccs=_ccs;
}  

