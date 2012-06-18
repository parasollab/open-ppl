#include "Region.h"

/////////////////////////
//Constructors
////////////////////////
Region::Region(){ }

Region::~Region(){ }

Region::Region(const Region& _r){
  m_bb=_r.m_bb;
  m_ccs=_r.m_ccs;
  m_vids = _r.m_vids;
}

Region::Region(shared_ptr<BoundingBox> _bbox, std::vector<pair<size_t, VID> >& _pairIds ){
  m_bb=_bbox;
  m_ccs.clear();
  m_ccs=_pairIds;
}

Region::Region(shared_ptr<BoundingBox> _bbox, std::vector<VID>& _ids ){
  m_bb=_bbox;
  m_vids.clear();
  typedef std::vector<VID>::iterator itr;
  for(itr vit = _ids.begin(); vit!=_ids.end(); vit++){
    m_vids.push_back(*vit);
  }
}

shared_ptr<BoundingBox> 
Region::GetBoundingBox() const{
  return m_bb;
}

std::vector<pair<VID, size_t> >& 
Region::GetCCs() const{
  return m_ccs;
}

pair<shared_ptr<BoundingBox>, vector<pair<size_t, VID> > >
Region::GetRegionInfo() const{
  pair<shared_ptr<BoundingBox>, vector<pair<size_t, VID> > > infoPair = make_pair(m_bb, m_ccs);
  return infoPair;
}

std::vector<VID>&
Region::RegionVIDs() const {
  return m_vids;
}

void
Region::SetVIDs(std::vector<VID>& _ids) {
  std::vector<VID>::iterator itr;
  for (itr = _ids.begin(); itr != _ids.end(); ++itr) {
    m_vids.push_back(*itr) ;
  }
}

size_t
Region::RegionWeight() const {
  return m_vids.size();
}

void
Region::SetBoundingBox(shared_ptr<BoundingBox> _bb){
  m_bb=_bb;
}

///This doesn't make much sense, Shuvra needs to look into it.
std::vector<VID>
Region::GetAllVIDs() const{
  vector<VID> allVIDs;
  return allVIDs;
} 

bool 
Region::operator==(const Region& _a) const{
  return (m_bb == _a.m_bb) &&
    (m_ccs == _a.m_ccs) &&
    (m_vids == _a.m_vids);
} 

ostream& operator<< (ostream& _os, const Region& _r) {
  _r.Print(_os);
  return _os;
}

void
Region::SetCCs(std::vector<pair<VID, size_t> >& _ccs){
  m_ccs=_ccs;
}


void
Region::
Print(ostream& _os) const  {
  typedef std::vector<VID>::const_iterator itr;
  _os << "  " << "Region Weight :" << m_vids.size() << " "; 
  _os << "Region VIDs: ";
  for(itr vit = m_vids.begin(); vit!=m_vids.end(); vit++){
    _os << "  " << *vit<< "";
  }
}

