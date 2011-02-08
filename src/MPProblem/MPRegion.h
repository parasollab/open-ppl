#ifndef _MPRegion_h_
#define _MPRegion_h_

#include "Boundary.h"
#include "Environment.h"
#include "Stat_Class.h"

///Design info.  Each MPRegion should be a self contained
///MP solution.  It will have its own roadmaps, stats, etc.
///In the future, it will be able to construct its own SubRegions.


template <class CFG, class WEIGHT>
class MPRegion : public Environment {
 public:
 typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
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
  Roadmap<CFG,WEIGHT>* GetBlockRoadmap() { return &block_roadmap;};
  Stat_Class* GetStatClass() {return &StatClass;};
  vector<VID> AddToRoadmap(vector<CFG >& in_Cfgs);
  vector<VID> AddToBlockRoadmap(vector<CFG >& in_Cfgs);
  void WriteRoadmapForVizmo();
  void WriteRoadmapForVizmo(ostream& out_os, vector<BoundingBox*>* bboxes, bool block);
  
  ~MPRegion();

  void PrintValues(ostream& _os);
/*   Environment* GetEnvironment(); */

 public:
  Stat_Class feature_stats, map_stats, combine_stats;
  
  
  Stat_Class StatClass;
  Roadmap<CFG,WEIGHT> roadmap;
  Roadmap<CFG,WEIGHT> feature_roadmap;
  Roadmap<CFG,WEIGHT> col_roadmap;
  Roadmap<CFG,WEIGHT> block_roadmap;

  
  ///\todo May need a reall region id into MPProblem later.
  string region_tag;

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
  block_roadmap.SetEnvironment(this);

  // set region_tag
  std::stringstream ss;
  ss << index;
  std::string str_index;
  ss >> str_index;
  if (parent == NULL) 
    region_tag = str_index;
  else
    region_tag = parent->region_tag + "-" + str_index;  
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
    block_roadmap.SetEnvironment(this);

    
    m_RegionId = in_RegionId;
    /*
  // set region_tag
    std::stringstream ss;
    ss << index;
    std::string str_index;
    ss >> str_index;
    if (parent == NULL) 
      region_tag = str_index;
    else
      region_tag = parent->region_tag + "-" + str_index;  
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
  _os << "MPRegion::PrintValues ID(" << region_tag << "): ";
  boundaries->Print(_os);
}



///\todo this is a bad implementation only returns FREE vids ... we need COLORS!
template <class CFG, class WEIGHT>
vector<typename RoadmapGraph<CFG, WEIGHT>::VID> 
MPRegion<CFG,WEIGHT>::
AddToRoadmap(vector<CFG>& in_Cfgs) {
  vector<typename RoadmapGraph<CFG, WEIGHT>::VID> returnVec;
  typename vector< CFG >::iterator I;
  for(I=in_Cfgs.begin(); I!=in_Cfgs.end(); I++) {
    if((*I).IsLabel("VALID")) {  
      if((*I).GetLabel("VALID")) {//Add to Free roadmap
        returnVec.push_back(roadmap.m_pRoadmap->AddVertex((*I)));
      } else {  //Add to Coll Roadmap 
        //LOG_DEBUG_MSG("MPRegion::AddToRoadmap() -- Adding Coll CFG");
        
        // commented by Bryan on 5/4/08, we only want one roadmap
        //col_roadmap.m_pRoadmap->AddVertex((*I));

      }
    } else {LOG_DEBUG_MSG("MPRegion::AddToRoadmap() -- UNLABELED!!!!!!!");}
  }
  return returnVec;
}

///\todo this is a bad implementation only returns FREE vids ... we need COLORS!
template <class CFG, class WEIGHT>
vector<typename RoadmapGraph<CFG, WEIGHT>::VID> 
MPRegion<CFG,WEIGHT>::
AddToBlockRoadmap(vector<CFG>& in_Cfgs) {
  vector<typename RoadmapGraph<CFG, WEIGHT>::VID> returnVec;
  typename vector< CFG >::iterator I;
  for(I=in_Cfgs.begin(); I!=in_Cfgs.end(); I++) {
    if((*I).IsLabel("INVALID")) {  
      if((*I).GetLabel("INVALID")) {//Add to Free roadmap
        returnVec.push_back(roadmap.m_pRoadmap->AddVertex((*I)));
      }
    } else {LOG_DEBUG_MSG("MPRegion::AddToRoadmap() -- UNLABELED!!!!!!!");}
  }
  return returnVec;
}


template <class CFG, class WEIGHT>
void MPRegion<CFG,WEIGHT>::
WriteRoadmapForVizmo() {
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
  WriteRoadmapForVizmo(myofstream);
  myofstream.close();
};
template <class CFG, class WEIGHT>
void MPRegion<CFG,WEIGHT>::
WriteRoadmapForVizmo(ostream& myofstream, vector<BoundingBox*>* bboxes = NULL, bool block = false) {

  LOG_DEBUG_MSG("MPRegion::WriteRoadmapForVizmo()");
  
  
  
  myofstream << "Roadmap Version Number " << RDMPVER_CURRENT_STR;
  myofstream << endl << "#####PREAMBLESTART#####";
  myofstream << endl << "../obprm -f " << GetMPProblem()->GetEnvironment()->GetEnvFileName() << " ";//commandLine;
  myofstream << " -bbox "; GetBoundingBox()->Print(myofstream, ',', ',');
  if(bboxes!=NULL){
     typedef vector<BoundingBox*>::iterator BIT;
     for(BIT bit = bboxes->begin(); bit!=bboxes->end(); bit++){
        myofstream << " -bbox "; (*bit)->Print(myofstream, ',', ',');
     }
  }
  myofstream << endl << "#####PREAMBLESTOP#####";
  
  myofstream << endl << "#####ENVFILESTART#####";
  myofstream << endl << GetMPProblem()->GetEnvironment()->GetEnvFileName();
  myofstream << endl << "#####ENVFILESTOP#####";
  myofstream << endl;

  ///TODO: fix so vizmo can understand the following 3 lines instead of the explicit printouts below
  /*
  GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->WriteLPsForVizmo(myofstream);
  GetMPProblem()->GetCollisionDetection()->WriteCDsForVizmo(myofstream);
  GetMPProblem()->GetDistanceMetric()->WriteDMsForVizmo(myofstream);
  */
  myofstream << "#####LPSTART#####" << endl << "0" << endl << "#####LPSTOP#####" << endl;
  myofstream << "#####CDSTART#####" << endl << "0" << endl << "#####CDSTOP#####" << endl;
  myofstream << "#####DMSTART#####" << endl << "0" << endl << "#####DMSTOP#####";
  GetRoadmap()->WriteRNGseed(myofstream);
  myofstream << endl;

  //GetRoadmap()->m_pRoadmap->WriteGraph(myofstream);         // writes verts & adj lists
  if(!block)
   write_graph(*(GetRoadmap()->m_pRoadmap), myofstream);         // writes verts & adj lists
  else 
   write_graph(*(GetBlockRoadmap()->m_pRoadmap), myofstream);         // writes verts & adj lists
  
  LOG_DEBUG_MSG("~MPRegion::WriteRoadmapForVizmo()");
}






/* template <class CFG, class WEIGHT> */
/* Environment*  */
/* MPRegion<CFG,WEIGHT>:: */
/* GetEnvironment() { */
/*   return &env; */
/* } */


#endif /*_MPRegion_h_*/
