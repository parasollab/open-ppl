#ifndef _NaiveMapCombine_h_
#define _NaiveMapCombine_h_

#include "ConnectionMethod.h"


template <class CFG, class WEIGHT>
class NaiveMapCombine: public ConnectionMethod<CFG,WEIGHT> {
 public:
  NaiveMapCombine();
  ~NaiveMapCombine();
  char* GetName();
  void SetDefault();

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(std::istringstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual ConnectionMethod<CFG,WEIGHT>* CreateCopy();


  void ConnectComponents();
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges);

  ///Takes in an MPRegion as a target (output), and a two MPRegions
  ///to compine.  It simply copies all verticies and edges from the 
  ///input roadmaps into the target_region's roadmap.  Then calls
  ///ConnectComponents on a vector<vector<CFG>> where each 
  ///vector<CFG> is from an input subregion.
  void CombineRegions(MPRegion<CFG,WEIGHT>* target_region, ///< Output MPregion
		      MPRegion<CFG,WEIGHT>* &l_subregion, ///< MPRegion (left subregions) to merge
		      MPRegion<CFG,WEIGHT>* &r_subregion, ///< MPRegion (right subregions) to merge
		      ConnectMap<CFG, WEIGHT>& cm,
		      CollisionDetection& cd, DistanceMetric& dm,
		      LocalPlanners<CFG,WEIGHT>& lp,
		      bool addPartialEdge, bool addAllEdges);

    
  ///Takes in an MPRegion as a target (output), and a vector<MPRegion> 
  ///to compine.  It simply copies all verticies and edges from the 
  ///input roadmaps into the target_region's roadmap.  Then calls
  ///ConnectComponents on a vector<vector<CFG>> where each 
  ///vector<CFG> is from an input subregion.
  void CombineRegions(MPRegion<CFG,WEIGHT>* target_region,         ///< Output MPregion
		      vector<MPRegion<CFG,WEIGHT>* > &subregions,  ///< Vector of MPRegion (subregions) to merge
		      ConnectMap<CFG, WEIGHT>& cm,
		      CollisionDetection& cd, DistanceMetric& dm,
		      LocalPlanners<CFG,WEIGHT>& lp,
		      bool addPartialEdge, bool addAllEdges);

};


template <class CFG, class WEIGHT>
NaiveMapCombine<CFG,WEIGHT>::
NaiveMapCombine() : ConnectionMethod<CFG,WEIGHT>() {
  SetDefault();
}

template <class CFG, class WEIGHT>
NaiveMapCombine<CFG,WEIGHT>::
~NaiveMapCombine() {
}

template <class CFG, class WEIGHT>
char*
NaiveMapCombine<CFG,WEIGHT>::
GetName() {
  return "NaiveMapCombine";
}

template <class CFG, class WEIGHT>
void
NaiveMapCombine<CFG,WEIGHT>::
SetDefault() {

}

template <class CFG, class WEIGHT>
void
NaiveMapCombine<CFG,WEIGHT>::
ParseCommandLine(std::istringstream& is) {

}

template <class CFG, class WEIGHT>
void
NaiveMapCombine<CFG,WEIGHT>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  _os << "\n" << GetName() << " ";
  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG, class WEIGHT>
void 
NaiveMapCombine<CFG,WEIGHT>::
PrintValues(ostream& _os) {
  _os << "\n" << GetName() << " ";
  _os << endl;
}

template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>*
NaiveMapCombine<CFG,WEIGHT>::
CreateCopy() {
  ConnectionMethod<CFG,WEIGHT> * _copy = new NaiveMapCombine<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void NaiveMapCombine<CFG,WEIGHT>::
ConnectComponents() {
  cout << "DOING NOTHING" << endl;
}
 

template <class CFG, class WEIGHT>
void NaiveMapCombine<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		  CollisionDetection* cd , 
		  DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge, 
		  bool addAllEdges) {
  cout << "DOING NOTHING" << endl;
}


template <class CFG, class WEIGHT>
void NaiveMapCombine<CFG,WEIGHT>::
CombineRegions(MPRegion<CFG,WEIGHT>* target_region,
	       MPRegion<CFG,WEIGHT>* &l_subregion,
	       MPRegion<CFG,WEIGHT>* &r_subregion,
	       ConnectMap<CFG, WEIGHT>& cm,
	       CollisionDetection& cd, DistanceMetric& dm,
	       LocalPlanners<CFG,WEIGHT>& lp,
	       bool addPartialEdge, bool addAllEdges) {


  vector< vector< CFG > > selectedCFGs; // Vector to store selected nodes from each region

  //Get boundary of left and right regions and print
  BoundingBox* l_boundary = l_subregion->GetBoundingBox();  
  BoundingBox* r_boundary = r_subregion->GetBoundingBox();
  cout << " Combining Regions with BoundingBoxes as follows: " << endl;
  l_boundary->Print(cout);
  r_boundary->Print(cout);
  


  /////////////////////  Working with Left Region ///////////////////////
  //Get Left map's vertices and copy merge map into target map
  vector<VID> l_map_vertices;
  l_subregion->roadmap.m_pRoadmap->GetVerticesVID(l_map_vertices);
  vector<VID> l_map_newVids = target_region->roadmap.m_pRoadmap->MergeRoadMap(l_subregion->roadmap.m_pRoadmap,
									      l_map_vertices);
  //Convert Left Map's vertices into a vector of CFGs
  vector<CFG> l_map_CFG2;
  for(vector<VID>::iterator vidi=l_map_vertices.begin(); vidi<l_map_vertices.end(); vidi++) {
    l_map_CFG2.push_back(target_region->roadmap.m_pRoadmap->GetData(*vidi));
  }
  //Push all of Left Map's CFGs into selected vector for connection
  selectedCFGs.push_back(l_map_CFG2);




  /////////////////////  Working with Right Region ///////////////////////
  //Get right map's vertices and copy merge map into target map
  vector<VID> r_map_vertices;
  r_subregion->roadmap.m_pRoadmap->GetVerticesVID(r_map_vertices);
  vector<VID> r_map_newVids = target_region->roadmap.m_pRoadmap->MergeRoadMap(r_subregion->roadmap.m_pRoadmap,
									      r_map_vertices);
  //Convert Left Map's vertices into a vector of CFGs
  vector<CFG> r_map_CFG2;
  for(vector<VID>::iterator vidi=r_map_vertices.begin(); vidi<r_map_vertices.end(); vidi++) {
    r_map_CFG2.push_back(target_region->roadmap.m_pRoadmap->GetData(*vidi));
  }
  //Push all of Left Map's CFGs into selected vector for connection
  selectedCFGs.push_back(r_map_CFG2);


  //Call connection method using our selected CFG
  cm.ConnectComponents(&(target_region->roadmap), target_region->combine_stats,
			 &cd, &dm, &lp,
			 addPartialEdge, 
			addAllEdges, selectedCFGs);
}


template <class CFG, class WEIGHT>
void NaiveMapCombine<CFG,WEIGHT>::
CombineRegions(MPRegion<CFG,WEIGHT>* target_region, 
	       vector<MPRegion<CFG,WEIGHT>* > &subregions, 
	       ConnectMap<CFG, WEIGHT>& cm,
	       CollisionDetection& cd, DistanceMetric& dm,
	       LocalPlanners<CFG,WEIGHT>& lp,
	       bool addPartialEdge, bool addAllEdges) {
  
  ///TODO:Implement this function in MPRegion...
  //target_region->computeBoundary(subregions);
  //for all pairs of subranges in vector<MPRegion<CFG,WEIGHT>>
  for (int i=0;i<subregions.size();++i) {
    for (int j=i+1; j<subregions.size();++j) {
      this->CombineRegions(target_region, subregions[i], subregions[j],cm,cd,dm,lp,addPartialEdge,addAllEdges);
    }
  }
}

#endif /*_NaiveMapCombine_h_*/
