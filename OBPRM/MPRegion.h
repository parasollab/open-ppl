#ifndef _MPRegion_h_
#define _MPRegion_h_

#include "Boundary.h"
#include "Environment.h"
#include "CfgTypes.h"

///Design info.  Each MPRegion should be a self contained
///MP solution.  It will have its own roadmaps, stats, etc.
///In the future, it will be able to construct its own SubRegions.


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
  
  MPRegion(int in_RegionId, MPProblem* in_pProblem);
  Roadmap<CFG,WEIGHT>* GetRoadmap() { return &roadmap;};
  Roadmap<CFG,WEIGHT>* GetColRoadmap() { return &col_roadmap;};
  Stat_Class* GetStatClass() {return &StatClass;};
  void AddToRoadmap(vector<CFG >& in_Cfgs);
  void WriteRoadmapForVizmo();
  
  ~MPRegion();

  void PrintValues(ostream& _os);
/*   Environment* GetEnvironment(); */

 public:
  Stat_Class feature_stats, map_stats, combine_stats;
  
  
  Stat_Class StatClass;
  Roadmap<CFG,WEIGHT> roadmap;
  Roadmap<CFG,WEIGHT> feature_roadmap;
  Roadmap<CFG,WEIGHT> col_roadmap;

  
  ///\todo May need a reall region id into MPProblem later.
  string region_id;

  int m_RegionId;
 private:
  //Environment &env;
  //vector<RoadmapFeature> features;
  //string characterization;
   ///\todo remove w/ Marco's approval 
  MPRegion<CFG,WEIGHT>* parent;
  ///\todo Change to a typedeffed or sub class called MPSubRegion.
  ///Also, would be much better if not a vector of pointers
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
MPRegion(int in_RegionId, MPProblem* in_pProblem) :
    Environment(in_pProblem), 
  parent(NULL) {

  // empty initial roadmaps
    roadmap.SetEnvironment(this);
    //feature_roadmap.SetEnvironment(this);
    col_roadmap.SetEnvironment(this);

    
    m_RegionId = in_RegionId;
    /*
  // set region_id
    std::stringstream ss;
    ss << index;
    std::string str_index;
    ss >> str_index;
    if (parent == NULL) 
      region_id = str_index;
    else
      region_id = parent->region_id + "-" + str_index;  
    */
  
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




template <class CFG, class WEIGHT>
void MPRegion<CFG,WEIGHT>::
AddToRoadmap(vector<CFG>& in_Cfgs) {

  typename vector< CFG >::iterator I;
  for(I=in_Cfgs.begin(); I!=in_Cfgs.end(); I++) {
    if((*I).IsLabel("VALID")) {  
      if((*I).GetLabel("VALID")) {//Add to Free roadmap
        roadmap.m_pRoadmap->AddVertex((*I));
      } else {  //Add to Coll Roadmap 
        col_roadmap.m_pRoadmap->AddVertex((*I));
      }
    }
  }
}


template <class CFG, class WEIGHT>
void MPRegion<CFG,WEIGHT>::
WriteRoadmapForVizmo() {

  LOG_DEBUG_MSG("MPRegion::WriteRoadmapForVizmo()");
  
  std::stringstream ss;
  ss << m_RegionId;
  std::string str_index;
  ss >> str_index;
  
  string outputFilename = GetMPProblem()->GetOutputRoadmap() + "RegionId" + str_index + ".map";
  ofstream  myofstream(outputFilename.c_str());
  
  if (!myofstream) {
    LOG_ERROR_MSG("MPRegion::WriteRoadmapForVizmo: can't open outfile: ");
    exit(-1);
  }
  
  myofstream << "Roadmap Version Number " << RDMPVER_CURRENT_STR;
  myofstream << endl << "#####PREAMBLESTART#####";
  myofstream << endl << "../obprm -f " << GetMPProblem()->GetEnvFileName() << " ";//commandLine;
  myofstream << " -bbox "; GetBoundingBox()->Print(myofstream, ',', ',');
  myofstream << endl << "#####PREAMBLESTOP#####";
  
  myofstream << endl << "#####ENVFILESTART#####";
  myofstream << endl << GetMPProblem()->GetEnvFileName();
  myofstream << endl << "#####ENVFILESTOP#####";
  
  GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->WriteLPs(myofstream);
  GetMPProblem()->GetCollisionDetection()->WriteCDs(myofstream);
  GetMPProblem()->GetDistanceMetric()->WriteDMs(myofstream);
  GetRoadmap()->WriteRNGseed(myofstream);

  GetRoadmap()->m_pRoadmap->WriteGraph(myofstream);         // writes verts & adj lists
  myofstream.close();
  LOG_DEBUG_MSG("~MPRegion::WriteRoadmapForVizmo()");
}






/* template <class CFG, class WEIGHT> */
/* Environment*  */
/* MPRegion<CFG,WEIGHT>:: */
/* GetEnvironment() { */
/*   return &env; */
/* } */


#endif /*_MPRegion_h_*/
