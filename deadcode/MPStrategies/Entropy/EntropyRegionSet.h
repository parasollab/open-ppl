#ifndef CREGIONSET_h
#define CREGIONSET_h

#include "EntropyRegion.h"
static int RegionSetID_val = 0;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class CRegionSet {
 public:

  CRegionSet();
  CRegionSet(int i);
  CRegionSet(const CRegionSet<CFG>& regionset);
  CRegionSet(const CRegion<CFG>& rs);
  ~CRegionSet();

  void AddRegion( CRegion<CFG>& region );

  bool operator==(const CRegionSet<CFG>& rs) const { return ID == rs.ID; }
  bool operator!=(const CRegionSet<CFG>& rs) const { return ID != rs.ID; }
  void SetRegionStats();

  int Classify(Environment* env, Stat_Class& Stats, CollisionDetection* cd,
	       CDInfo* cdInfo, DistanceMetric* dm,
	       double LowEntropy, int KSamples, int Tries);

  void SetVID(const CFG& c, VID v);

  template <class regionset_iterator>
  void
  GetOverlappingRegionsReference(regionset_iterator first,
				 regionset_iterator last,
				 Environment* env, DistanceMetric* dm,
				 vector<regionset_iterator>& overlapping);
  void GetPercentFreeNodeAndVID(double free_pct,
				vector<CFG>& nodes, vector<VID>& vids) const;
  void GetPercentBlockedNode(double pct, vector<CFG>& nodes) const;
  vector<VID> GetPercentFreeVIDs(double free_pct) const;

  std::string GetStringType() const { return m_RegionSet[0].GetStringType(); }
  int size() const { return NumFreeNodes+NumCollisionNodes; }
  int NumSubRegions(){ return m_RegionSet.size(); }
  /*
  CRegion<CFG>& GetRegionReference(int i) {
    if( i<0 || i>=m_RegionSet.size() ) {

    }
  }
  */

  //data members
  int type;
  vector< CRegion<CFG> > m_RegionSet;
  int ID, m_NumRegions;
  int NumCollisionNodes, NumFreeNodes;
  double entropy;
};

//io operators for graph class
template<class CFG>
ostream& operator<<(ostream& os, const CRegionSet<CFG>& rs) {
  os << rs.ID;
  return os;
}
template<class CFG>
istream& operator>>(istream& is, CRegionSet<CFG>& rs) {
  is >> rs.ID;
  return is;
}
template <class CFG>
CRegionSet<CFG>::
CRegionSet() {
  type=UNKNOWN;
  ID = RegionSetID_val;
  RegionSetID_val++;
  NumCollisionNodes = NumFreeNodes = 0;
  m_RegionSet.clear();
}
template <class CFG>
CRegionSet<CFG>::
CRegionSet(int i) {
  type=UNKNOWN;
  ID = RegionSetID_val;
  RegionSetID_val++;
  NumCollisionNodes = NumFreeNodes = 0;
  cerr << "\n\nWARNING: using an empty region\n\n";
  m_RegionSet.clear();
}


template <class CFG>
CRegionSet<CFG>::
CRegionSet(const CRegionSet<CFG>& rs) {
  type=rs.type;
  ID = rs.ID;
  NumCollisionNodes = rs.NumCollisionNodes;
  NumFreeNodes = rs.NumFreeNodes;
  m_RegionSet = rs.m_RegionSet;
}
///*
template <class CFG>
CRegionSet<CFG>::
CRegionSet(const CRegion<CFG>& r) {
  type=r.type;
  ID = RegionSetID_val;
  NumCollisionNodes = r.NumCollisionNodes;
  NumFreeNodes = r.NumFreeNodes;
  m_RegionSet.push_back( r );
}
//*/

template <class CFG>
CRegionSet<CFG>::
~CRegionSet() {
}

template <class CFG>
void
CRegionSet<CFG>::
AddRegion(CRegion<CFG>& region) {
  CRegion<CFG> t_region = region;
  m_RegionSet.push_back( t_region );
  type = region.type;
  NumFreeNodes += region.NumFreeNodes;
  NumCollisionNodes += region.NumCollisionNodes;
}

template <class CFG>
void
CRegionSet<CFG>::
SetRegionStats() {

  typename vector< CRegion<CFG> >::iterator rs;
  int I=0;
  int cur_type;
  NumCollisionNodes = 0;
  NumFreeNodes = 0;
  entropy = 0;
  for(rs=m_RegionSet.begin(); rs!=m_RegionSet.end(); ++rs,++I) {
    rs->SetRegionStats();
    if( I==0 ) { cur_type = rs->type; }
    else {
      if( rs->type != cur_type ) {
	cerr << "RegionSet type mis-match...non homogenous RegionSet"<<endl;
      }
    }
    NumCollisionNodes += rs->NumCollisionNodes;
    NumFreeNodes += rs->NumFreeNodes;
    entropy += rs->entropy;
  }//end for rs!=m_RegionSet.end();
  entropy /= m_RegionSet.size();
  type = cur_type;
}

template <class CFG>
int
CRegionSet<CFG>::
Classify(Environment* env, Stat_Class& Stats, CollisionDetection* cd,
	 CDInfo* cdInfo, DistanceMetric* dm,
	 double LowEntropy, int KSamples, int Tries) {

  NumCollisionNodes = 0;
  NumFreeNodes = 0;
  entropy = 0;
  int cur_type;
  for(int I=0; I<m_RegionSet.size(); I++) {

    m_RegionSet[I].Classify(env, Stats, cd, cdInfo, dm,
			    LowEntropy, KSamples, Tries);

    if( I==0 ) { cur_type = m_RegionSet[I].type; }
    else {
      if( m_RegionSet[I].type != cur_type ) {
	cerr << "RegionSet type mis-match...non homogenous RegionSet"<<endl;
      }
    }
    NumCollisionNodes += m_RegionSet[I].NumCollisionNodes;
    NumFreeNodes += m_RegionSet[I].NumFreeNodes;
    entropy += m_RegionSet[I].entropy;
  }

  entropy /= m_RegionSet.size();
  type = cur_type;
  return type;
}

template <class CFG>
template <class regionset_iterator>
void
CRegionSet<CFG>::
GetOverlappingRegionsReference(regionset_iterator first,
			       regionset_iterator last,
			       Environment* env, DistanceMetric* dm,
			       vector<regionset_iterator>& overlapping) {
  for(regionset_iterator R = first; R != last; ++R)
    if(R->data.ID != ID) {
      //check if overlaps ... they are blobs (no longer spheres)

      bool overlaps = false;
      vector< CRegion<CFG> >& R_regions = R->data.m_RegionSet;
      for(int I=0; (I<R_regions.size()) && !overlaps; I++) {
	//for each subregion check if overlaps

	for(int J=0; J<m_RegionSet.size(); J++) {
	  if( m_RegionSet[J].Overlaps( R_regions[I], env, dm ) ) {
	    overlaps = true;
	    break;
	  }
	}//end for J<m_RegionSet.size()

      }//end for I<R_regions.size()

      if( overlaps )
	overlapping.push_back(R);

    }//endif same region

}
template <class CFG>
void
CRegionSet<CFG>::
GetPercentFreeNodeAndVID(double free_pct,
			 vector<CFG>& nodes, vector<VID>& vids) const {

  int min_nodes = int(free_pct*NumFreeNodes);
  //cout << "pct_free: " << free_pct << " getting number of nodes: " << min_nodes
  //     << " out of: " << NumFreeNodes << endl;
  int num_obtained = 0;
  for(int I=0; (I<m_RegionSet.size())&&(num_obtained<min_nodes); I++) {
    vector<CFG> new_nodes;
    vector<VID> new_vids;
    m_RegionSet[I].GetPercentFreeNodeAndVID(free_pct, new_nodes, new_vids);
    num_obtained+=new_nodes.size();
    nodes.insert(nodes.end(),new_nodes.begin(),new_nodes.end());
    vids.insert(vids.end(),new_vids.begin(),new_vids.end());
  }
}

template <class CFG>
void
CRegionSet<CFG>::
GetPercentBlockedNode(double pct, vector<CFG>& nodes) const {
  int min_nodes = int(pct*NumCollisionNodes);
  int num_obtained = 0;
  for(int I=0; (I<m_RegionSet.size()) && (num_obtained<min_nodes); ++I) {
    vector<CFG> new_nodes;
    m_RegionSet[I].GetPercentBlockedNode(pct, new_nodes);
    num_obtained += new_nodes.size();
    nodes.insert(nodes.end(), new_nodes.begin(), new_nodes.end());
  }
}


template <class CFG>
vector<VID>
CRegionSet<CFG>::
GetPercentFreeVIDs(double free_pct) const {
  vector<VID> vids;
  int min_nodes = int(free_pct*NumFreeNodes);
  //cout << "pct_free: " << free_pct << " getting number of nodes: " << min_nodes
  //     << " out of: " << NumFreeNodes << endl;
  int num_obtained = 0;
  for(int I=0; (I<m_RegionSet.size())&&(num_obtained<min_nodes); I++) {
    vector<VID> new_vids =
      m_RegionSet[I].GetPercentFreeVIDs(free_pct);
    num_obtained+=new_vids.size();
    vids.insert(vids.end(),new_vids.begin(),new_vids.end());
  }
  return vids;
}


//calling this funtion will differ slightly
//because things in the same region set
//are not necessarily what was connected
//in our previous region map (hope it will not ruin things)
template <class CFG>
void
CRegionSet<CFG>::
SetVID(const CFG& c, VID v) {


  for(int I=0; I<m_RegionSet.size(); I++) {
    m_RegionSet[I].SetVID(c,v);
  }//end for I<m_RegionSet.size()


}

#endif
