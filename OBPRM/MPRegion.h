#ifndef _MPRegion_h_
#define _MPRegion_h_

#include "Boundary.h"
#include "Environment.h"


template <class CFG, class WEIGHT>
class MPRegion : public Environment {
 public:
  MPRegion(
	   Environment &i_env, 	
/* 	   GenerateMapNodes<CFG> &i_gn_map,  */
/* 	   GenerateMapNodes<CFG> &i_gn_features,  */
/* 	   ConnectMap<CFG,WEIGHT> &i_cm_map, */
/* 	   ConnectMap<CFG,WEIGHT> &i_cm_features, */
/* 	   ConnectMap<CFG,WEIGHT> &i_cm_combine, */
/* 	   LocalPlanners<CFG,WEIGHT> &i_lp_map,  */
/* 	   LocalPlanners<CFG,WEIGHT> &i_lp_features,  */
/* 	   DistanceMetric &i_dm,  */
/* 	   CollisionDetection &i_cd/\*,ValidityTest &i_vt*\/, */
	   BoundingBox &i_bb, int index,
	   MPRegion<CFG,WEIGHT>* i_parent);
  ~MPRegion();

  void PrintValues(ostream& _os);
/*   Environment* GetEnvironment(); */

 public:
  Stat_Class feature_stats, map_stats, combine_stats;

  Roadmap<CFG,WEIGHT> roadmap;
  Roadmap<CFG,WEIGHT> feature_roadmap;

  string region_id;

 private:
  //Environment &env;
  //vector<RoadmapFeature> features;
  //string characterization;
  MPRegion<CFG,WEIGHT>* parent;
  vector< MPRegion<CFG,WEIGHT>* > subregions;

};

template <class CFG, class WEIGHT>
MPRegion<CFG,WEIGHT>::
  MPRegion(Environment &i_env, 
	   BoundingBox &i_boundaries, int index,
	   MPRegion<CFG,WEIGHT>* i_parent) :
  Environment(i_env, i_boundaries), 
  parent(i_parent) {

  // empty initial roadmaps
  roadmap.SetEnvironment(this);
  feature_roadmap.SetEnvironment(this);

  // set region_id
  std::stringstream ss;
  ss << index;
  std::string str_index;
  ss >> str_index;
  if (parent == NULL) 
    region_id = str_index;
  else
    region_id = parent->region_id + "-" + str_index;  
}

template <class CFG, class WEIGHT>
MPRegion<CFG,WEIGHT>::
~MPRegion() {
  cout << " ~MPRegion() . TODO ALL " << endl;
}

template <class CFG, class WEIGHT>
void 
MPRegion<CFG,WEIGHT>::
PrintValues(ostream& _os) {
  _os << "MPRegion::PrintValues ID(" << region_id << "): ";
  boundaries->Print(_os);
}

/* template <class CFG, class WEIGHT> */
/* Environment*  */
/* MPRegion<CFG,WEIGHT>:: */
/* GetEnvironment() { */
/*   return &env; */
/* } */


#endif /*_MPRegion_h_*/
