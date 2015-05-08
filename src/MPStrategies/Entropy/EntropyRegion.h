#ifndef CREGION_h
#define CREGION_h

#include <string>

enum rtype { UNKNOWN=-1, FREE=0, TRANSITION=1, SURFACE=2, NARROW=4, BLOCKED=3};

static int ID_val = 0;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
struct Sample {
  Sample() {}
  Sample(const CFG& n, bool iC, double d, VID v = INVALID_VID) :
    node(n), isColl(iC), distance(d), vid(v) {}
  ~Sample() {}

  CFG node;
  bool isColl;
  double distance;
  VID vid;
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
struct less_distance : public binary_function<Sample<CFG>, Sample<CFG>, bool> {
  bool operator()(const Sample<CFG>& s1, const Sample<CFG>& s2) const {
    return less<double>()(s1.distance, s2.distance);// (s1.distance < s2.distance);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
struct equal_node : public binary_function<Sample<CFG>, Sample<CFG>, bool> {
  bool operator()(const Sample<CFG>& s1, const Sample<CFG>& s2) const {
    return (s1.node == s2.node);
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG>
class CRegion {
 public:
  typedef typename vector<Sample<CFG> >::iterator sample_iterator;
  typedef typename vector<Sample<CFG> >::const_iterator sample_const_iterator;

  CRegion();
  CRegion(int i);
  CRegion(const CRegion<CFG>& region);
  ~CRegion();

  bool operator==(const CRegion<CFG>& r) const { return ID == r.ID; }
  bool operator!=(const CRegion<CFG>& r) const { return ID != r.ID; }

  void AddNode(CFG t, bool _iC, double _dist, VID v = INVALID_VID);
  void SetVID(const CFG& c, VID v);

  CFG getCenter() const { return center; }
  int size() const { return samples.size(); }
  double getRadius() const { return radius; }
  double getEntropy() const { return entropy; }

  std::string GetStringType() const {
    if( type == FREE ) return std::string("free");
    else if( type == TRANSITION ) return std::string("transition");
    else if( type == SURFACE ) return std::string("surface");
    else if( type == NARROW ) return std::string("narrow");
    else if( type == BLOCKED ) return std::string("blocked");
    else return std::string("unknown");
  };

  void SetRegionStats() {
    if(samples.empty()) {
      cout << " Region::SetRegionStats - no nodes in region." << endl;
      cout << " can't set stats " << endl;
      return;
    }

    center = samples[0].node; // set center to first node i.e. node with 0 dist

    if(!RadiusSet) {
      radius = max_element(samples.begin(), samples.end(), less_distance<CFG>())->distance;
      radius_cfg = max_element(samples.begin(), samples.end(), less_distance<CFG>())->node;
      RadiusSet = true;
    }
    // check case where there was only one node added
    // make it small factor
    if(samples.size() == 1)
      radius = 0.6*acceptable_dist;
    acceptable_dist = radius;

    if(NumCollisionNodes + NumFreeNodes != samples.size()) {
      NumCollisionNodes = 0;
      for(sample_const_iterator S = samples.begin(); S != samples.end(); ++S)
	if(S->isColl)
	  NumCollisionNodes++;
      NumFreeNodes = samples.size() - NumCollisionNodes;
    }

    entropy = (1.0*NumCollisionNodes)/(1.0*samples.size());
  };

  vector< CFG > GetPercentFreeNodes(double free_pct) const {
    vector<CFG> return_nodes;
    if( free_pct > 1 ) free_pct = 1;
    if( free_pct < 0 ) return return_nodes;//free_pct = 0;

    int return_num = int( free_pct * NumFreeNodes );

    int i=0;
    sample_const_iterator S = samples.begin();
    while(i < return_num && S != samples.end()) {
      if(!S->isColl) {
        return_nodes.push_back(S->node);
        ++i;
      }
      ++S;
    }

    return return_nodes;
  };

  vector<VID> GetPercentFreeVIDs(double free_pct) const {
    vector<VID> vids;
    free_pct = min(free_pct, 1);
    if(free_pct <= 0)
      return vids;

    int return_num = int(free_pct*NumFreeNodes);

    int i=0;
    sample_const_iterator S = samples.begin();
    while(i < return_num && S != samples.end()) {
      if(!S->isColl) {
	vids.push_back(S->vid);
	++i;
      }
      ++S;
    }

    return vids;
  }

  void GetPercentFreeNodeAndVID(double free_pct,
				vector<CFG>& nodes, vector<VID>& vids) const {
    free_pct = min(free_pct, 1);
    if(free_pct <= 0)
      return;

    int return_num = int(free_pct*NumFreeNodes);

    int i=0;
    sample_const_iterator S = samples.begin();
    while(i < return_num && S != samples.end()) {
      if(!S->isColl) {
	nodes.push_back(S->node);
	vids.push_back(S->vid);
	++i;
      }
      ++S;
    }
  }

  void GetPercentBlockedNode(double pct, vector<CFG>& nodes) const {
    pct = min(pct, 1);
    if(pct <= 0)
      return;

    int return_num = int(pct*NumCollisionNodes);

    int i=0;
    sample_const_iterator S = samples.begin();
    while(i < return_num && S != samples.end()) {
      if(S->isColl) {
	nodes.push_back(S->node);
	++i;
      }
      ++S;
    }
  }

  bool IsSufaceRegion(Environment* _env, DistanceMetric *dm,
		      double LowEntropy) const;

  /*
  template <class region_const_iterator>
  vector<CRegion<CFG> > GetOverlappingRegions(region_const_iterator first,
					      region_const_iterator last,
					      Environment* env,
					      DistanceMetric* dm) const;
  */
  /*
  template <class region_const_iterator>
  void
  GetOverlappingRegionsReference(region_const_iterator first,
				 region_const_iterator last,
				 Environment* env, DistanceMetric* dm,
				 vector<region_const_iterator>& overlapping) const;
  */

  bool Overlaps(CRegion<CFG>& R, Environment* env, DistanceMetric* dm);

  void AddRegionSamples(Environment* _env, DistanceMetric* dm,
                        Stat_Class& Stats, CollisionDetection* cd,
			CDInfo* cdInfo, int KSamples);
  void GenNearCFG(Environment* _env, DistanceMetric *dm, CFG& ret_cfg) const;

  int Classify(Environment* env, Stat_Class& Stats, CollisionDetection* cd,
	       CDInfo* cdInfo, DistanceMetric* dm,
	       double LowEntropy, int KSamples, int Tries);


  vector<Sample<CFG> > samples;

  CFG center;
  CFG radius_cfg;
  double entropy;
  double radius;
  double acceptable_dist;
  int type;
  int NumCollisionNodes, NumFreeNodes;

  int ID;
  bool RadiusSet;
};

//io operators for graph class
template<class CFG>
ostream& operator<<(ostream& os, const CRegion<CFG>& r) {
  os << r.ID;
  return os;
}
template<class CFG>
istream& operator>>(istream& is, CRegion<CFG>& r) {
  is >> r.ID;
  return is;
}


template <class CFG>
CRegion<CFG>::
CRegion() {
  type=UNKNOWN;
  ID = ID_val;
  ID_val++;
  RadiusSet = false;
  NumCollisionNodes = NumFreeNodes = 0;
}


template <class CFG>
CRegion<CFG>::
CRegion(int i) {
  type=UNKNOWN;
  ID = ID_val;
  ID_val++;
  RadiusSet = false;
  NumCollisionNodes = NumFreeNodes = 0;
  cerr << "\n\nWARNING: using an empty region\n\n";
}


template <class CFG>
CRegion<CFG>::
CRegion(const CRegion<CFG>& region) {
  samples = region.samples;
  center = region.center;
  entropy = region.entropy;
  radius = region.radius;
  acceptable_dist = region.acceptable_dist;
  type = region.type;
  NumCollisionNodes = region.NumCollisionNodes;
  NumFreeNodes = region.NumFreeNodes;
  ID = region.ID;
  radius_cfg = region.radius_cfg;
  RadiusSet = region.RadiusSet;
}


template <class CFG>
CRegion<CFG>::
~CRegion() {
}


template <class CFG>
void
CRegion<CFG>::
AddNode(CFG t, bool _iC, double _dist, VID v) {
  samples.push_back(Sample<CFG>(t, _iC, _dist, v));
  if(_iC)
    NumCollisionNodes++;
  else
    NumFreeNodes++;
}


template <class CFG>
void
CRegion<CFG>::
SetVID(const CFG& c, VID v) {
  Sample<CFG> s(c, true, -1);
  sample_iterator S = find_if(samples.begin(), samples.end(),
			      bind1st(equal_node<CFG>(), s));
  while(S != samples.end()) {
    S->vid = v;
    S = find_if(S+1, samples.end(), bind1st(equal_node<CFG>(), s));
  }
}


template <class CFG>
bool
CRegion<CFG>::
IsSufaceRegion(Environment* _env, DistanceMetric *dm,double LowEntropy) const {
  //##
  // compute free and collision centers

  vector<double> free_center_data;
  vector<double> coll_center_data;
  int num_free = 0;
  int num_coll = 0;
  for(sample_const_iterator S = samples.begin(); S != samples.end(); ++S) {

    const CFG& cfg = S->node;
    if(S->isColl) {

      if( coll_center_data.size() == 0 )
	coll_center_data = cfg.GetData();
      else {
	vector<double> cfg_data = cfg.GetData();
	for(int J=0; J<coll_center_data.size(); J++) {
	  coll_center_data[J] += cfg_data[J];
	}//end for J<coll_center_data
      }

      num_coll++;

    }//endif isColl[I]
    else {

      if( free_center_data.size() == 0 )
	free_center_data = cfg.GetData();
      else {
	vector<double> cfg_data = cfg.GetData();
	for(int J=0; J<free_center_data.size(); J++) {
	  free_center_data[J] += cfg_data[J];
	}//end for J<coll_center_data
      }

      num_free++;

    }//end else


  }//end for I<nodes.size()

  if( num_free <= 1 || num_coll <=1 ) return false;

  CFG free_center;
  CFG coll_center;
  // Normalize: do for both
  for(int I=0; I<free_center_data.size(); I++) {
    free_center_data[I] = free_center_data[I]/(1.0*num_free);
    coll_center_data[I] = coll_center_data[I]/(1.0*num_coll);
    if( I!= free_center_data.size()-1 ) {
      free_center.SetSingleParam(I, free_center_data[I]);
      coll_center.SetSingleParam(I, coll_center_data[I]);
    }
    else {
      free_center.SetSingleParam(I, free_center_data[I]);
      coll_center.SetSingleParam(I, coll_center_data[I]);
    }
  }//end for I<free_center_data.size()

  //##
  // find radius for each subregion (free and coll)
  double free_radius = -1, coll_radius = -1;
  for(sample_const_iterator S = samples.begin(); S != samples.end(); ++S) {

    const CFG& cfg = S->node;
    if(S->isColl) {
      double dist = dm->Distance( _env, cfg, coll_center );
      if( dist >= coll_radius ) coll_radius = dist;
    }//endif isColl[I]
    else {
      double dist = dm->Distance( _env, cfg, free_center );
      if( dist >= free_radius ) free_radius = dist;
    }//end else

  }//end for I<nodes.size()

  //##
  // entropy of each region type
  int num_free_in_free = 0, num_coll_in_free = 0;
  int num_free_in_coll = 0, num_coll_in_coll = 0;
  for(sample_const_iterator S = samples.begin(); S != samples.end(); ++S) {
    /*
    // cfg can be in both sub regions
    const CFG& cfg = S->node;
    // check collision
    double dist_to_coll_center = dm->Distance( _env, cfg, coll_center );
    if( dist_to_coll_center < coll_radius ) { // it's in count it in that reg.
      if(S->isColl) num_coll_in_coll++;
      else num_free_in_coll++;
    }//end if dist_to_coll_center < coll_radius
    // check free
    double dist_to_free_center = dm->Distance( _env, cfg, free_center );
    if( dist_to_free_center < free_radius ) { // it's in count it in that reg.
      if(S->isColl) num_coll_in_free++;
      else num_free_in_free++;
    }//end if dist_to_coll_center < coll_radius
    */

    //put the node in the region who's center is closest
    if(dm->Distance(_env, S->node, coll_center) < dm->Distance(_env, S->node, free_center)) {
      if(S->isColl) num_coll_in_coll++;
      else num_free_in_coll++;
    } else {
      if(S->isColl) num_coll_in_free++;
      else num_free_in_free++;
    }
  }

  //if either region is empty, return false
  if(((num_free_in_free + num_coll_in_free) == 0) ||
     ((num_free_in_coll + num_coll_in_coll) == 0))
    return false;

  //compute entropy of both regions
  double free_reg_entropy  = (1.0*num_coll_in_free)/
    (num_free_in_free+num_coll_in_free);
  double coll_reg_entropy  = (1.0*num_free_in_coll)/
    (num_free_in_coll+num_coll_in_coll);

  //isSurface if both regions have low entropy/mostly of same type of node
  return ((free_reg_entropy < LowEntropy) && (coll_reg_entropy < LowEntropy));

  //return ((free_reg_entropy < LowEntropy) || (coll_reg_entropy < LowEntropy));

  /*
  //##
  // if either sub region has low entropy
  // it is a surface node - hope it works
  // check empty
  bool isSurface = false;
  if( (num_free_in_free+num_coll_in_free) > 1 ) {
    double free_reg_entropy=(1.0*num_coll_in_free)/(num_free_in_free+num_coll_in_free);
    if( free_reg_entropy < LowEntropy ) isSurface = true;
  }
  if( (num_free_in_coll+num_coll_in_coll) > 1 ) {
    double coll_reg_entropy=(1.0*num_coll_in_coll)/(num_free_in_coll+num_coll_in_coll);
    if( coll_reg_entropy < LowEntropy ) isSurface = true;
  }

  return isSurface;
  */
}

/*
template <class CFG>
template <class region_const_iterator>
vector<CRegion<CFG> >
CRegion<CFG>::
GetOverlappingRegions(region_const_iterator first, region_const_iterator last,
		      Environment* env, DistanceMetric* dm) const {
  vector<CRegion<CFG> > overlapping;
  for(region_const_iterator R = first; R != last; ++R)
    if( (R->data.ID != ID) &&
        (dm->Distance(env, R->data.center, center) < (R->data.radius + radius)) )
      overlapping.push_back(R->data);
  return overlapping;
}
*/
/*
// This function is now in RegionSet
template <class CFG>
template <class region_const_iterator>
void
CRegion<CFG>::
GetOverlappingRegionsReference(region_const_iterator first,
			       region_const_iterator last,
			       Environment* env, DistanceMetric* dm,
			       vector<region_const_iterator>& overlapping) const {
  for(region_const_iterator R = first; R != last; ++R)
    if((R->data.ID != ID) &&
       (dm->Distance(env, R->data.center, center) < (R->data.radius + radius) ))
      overlapping.push_back(R);
}

*/

template <class CFG>
bool
CRegion<CFG>::
Overlaps(CRegion<CFG>& R, Environment* env, DistanceMetric* dm) {

  if( dm->Distance(env, R.center, center) < (R.radius + radius) )
    return true;
  else return false;

}
template <class CFG>
void
CRegion<CFG>::
AddRegionSamples(Environment* _env, DistanceMetric *dm,
                 Stat_Class& Stats, CollisionDetection* cd, CDInfo* cdInfo,
                 int KSamples) {
  for(int i=0; i<KSamples; i++) {
    CFG new_cfg;
    GenNearCFG(_env, dm, new_cfg);
    AddNode(new_cfg, new_cfg.isCollision(_env, Stats, cd, *cdInfo), dm->Distance(_env, center, new_cfg));
  }//end for i<KSamples
}


template <class CFG>
void
CRegion<CFG>::
GenNearCFG(Environment* _env, DistanceMetric *dm, CFG& ret_cfg) const {
  //int attempt = 0;
  do {
    //cout << " attempt at GenNearCFG: " << attempt << " with radius: " << radius<<  endl;
    //attempt++;
    ret_cfg.GetRandomRay(drand48()*radius/1.1, _env, dm); //since ray is only approx
    ret_cfg.add(center, ret_cfg);
  } while((dm->Distance(_env, center, ret_cfg) > radius) ||
          !(ret_cfg.InBoundary(_env)));

  if(!ret_cfg.InBoundary(_env))
    cout << " generated node outside bbox " << endl;

  /*
  cout << " start: " << center << endl
       << " near : " << ret_cfg
       << " at dist: " << dm->Distance( _env, center, ret_cfg )
       << endl << endl;
  */
}


template <class CFG>
int
CRegion<CFG>::
Classify(Environment* env, Stat_Class& Stats, CollisionDetection* cd,
	 CDInfo* cdInfo, DistanceMetric* dm,
	 double LowEntropy, int KSamples, int Tries) {
  //learn FREE/SURFACE
  for(int T=0; T<Tries; T++) {
    AddRegionSamples(env, dm, Stats, cd, cdInfo, KSamples);
    SetRegionStats();
    if(entropy < LowEntropy) {
      type = FREE;
      return type;
    } else if(IsSufaceRegion(env, dm, LowEntropy)) {
      type = SURFACE;
      return type;
    }
  }

  //learn BLOCKED/NARROW
  AddRegionSamples(env, dm, Stats, cd, cdInfo, KSamples);
  SetRegionStats();
  //if(entropy >= 1) //--- not sure if this works for all compilers
  if(NumFreeNodes == 0) {
    type = BLOCKED;
    return type;
  }
  if(IsSufaceRegion(env, dm, LowEntropy)) {
    type = SURFACE;
    return type;
  }
  if(entropy >= 1-LowEntropy) {
    type = NARROW;
    return type;
  }

  //type = UNKNOWN;
  type = SURFACE; //(default)
  return type;
}


#endif
