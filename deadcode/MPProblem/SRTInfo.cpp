#include "SRTInfo.h"
#include "MPProblem.h"
#include "MetricUtils.h"

typedef typename RoadmapGraph<CfgType,WeightType>::vertex_descriptor VID;

SRTInfo::
SRTInfo(BoundingBox &m_bbox, std::pair<CfgType,std::vector<VID> > &m_ids ){
  tree = m_ids;
  bb = m_bbox;
}

SRTInfo::
SRTInfo(const SRTInfo &input_bbinfo)  {
  bb = input_bbinfo.GetBoundingBox();
  tree = input_bbinfo.GetTree();
}

SRTInfo::
SRTInfo(){ }

SRTInfo::
~SRTInfo() { }

bool
SRTInfo::
operator==(const SRTInfo& bbi) const {
  return (srt_info == bbi.srt_info) &&
         (bb == bbi.bb) &&
         (tree == bbi.tree) ;
}

ostream& operator<< (ostream& s, const SRTInfo& bbi) {
  bbi.Print(s);
  return s;
}

CfgType
SRTInfo::
GetCandidate() const {
  return tree.first;
}

std::vector<VID>
SRTInfo::
GetVIDs() const {
  return tree.second;
}

std::pair<CfgType,std::vector<VID> >
SRTInfo::
GetTree() const {
  return tree;
}

BoundingBox
SRTInfo::
GetBoundingBox() const {
  return bb;
}

const pair<BoundingBox, pair<CfgType,vector<VID> > >
SRTInfo::
GetData() const {
  return srt_info;
}

void
SRTInfo::
SetBoundingBox(BoundingBox &bbox) {
  srt_info.first = bbox;
}

void
SRTInfo::
SetTree(std::pair<CfgType,std::vector<VID> > &m_ids) {
  tree = m_ids;
}

void
SRTInfo::
SetData(BoundingBox &m_bbox, std::pair<CfgType,std::vector<VID> > &m_ids) {
  SetBoundingBox(m_bbox);
  SetTree(m_ids);
}


void
SRTInfo::
Print(ostream &os) const  {
  typedef std::vector<VID>::const_iterator itr;
  os << "[ " ;
  os<< " -bbox ";
  bb.Print(os);
  os << "\n Candidate" << tree.first << "\n VIDs: ";
  for(itr vit = tree.second.begin(); vit!=tree.second.end(); vit++){
    os << " " << *vit;
  }
  os << " ]\n";
}
