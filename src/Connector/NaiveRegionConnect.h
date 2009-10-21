#ifndef _NaiveRegionConnect_h_
#define _NaiveRegionConnect_h_

#include "RegionConnectionMethod.h"


template <class CFG, class WEIGHT>
class NaiveRegionConnect: public RegionConnectionMethod<CFG,WEIGHT> {
 public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  NaiveRegionConnect();
  virtual ~NaiveRegionConnect();

  NaiveRegionConnect(XMLNodeReader& in_Node, MPProblem* in_pProblem);

  char* GetName();
  void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);

  virtual void PrintOptions(ostream& out_os);
  virtual RegionConnectionMethod<CFG,WEIGHT>* CreateCopy();


  ///Takes in an MPRegion as a target (output), and a two MPRegions
  ///to compine.  It simply copies all verticies and edges from the 
  ///input roadmaps into the target_region's roadmap.  Then calls
  ///ConnectComponents on a vector<vector<CFG>> where each 
  ///vector<CFG> is from an input subregion.
  void Connect(MPRegion<CFG,WEIGHT>* target_region, ///< Output MPregion
	       MPRegion<CFG,WEIGHT>* &l_region, ///< left subregion to connect
	       MPRegion<CFG,WEIGHT>* &r_region, ///< right subregions) to merge
	       ConnectMap<CFG, WEIGHT>& cm,
	       DistanceMetric& dm,
	       LocalPlanners<CFG,WEIGHT>& lp,
	       bool addPartialEdge, bool addAllEdges);

    
  ///Takes in an MPRegion as a target (output), and a vector<MPRegion> 
  ///to compine.  It simply copies all verticies and edges from the 
  ///input roadmaps into the target_region's roadmap.  Then calls
  ///ConnectComponents on a vector<vector<CFG>> where each 
  ///vector<CFG> is from an input subregion.
  void Connect(MPRegion<CFG,WEIGHT>* target_region,         // Output MPregion
	       vector<MPRegion<CFG,WEIGHT>* > &source_regions, // regions to merge
	       ConnectMap<CFG, WEIGHT>& cm,
	       DistanceMetric& dm,
	       LocalPlanners<CFG,WEIGHT>& lp,
	       bool addPartialEdge, bool addAllEdges);

};


template <class CFG, class WEIGHT>
NaiveRegionConnect<CFG,WEIGHT>::
NaiveRegionConnect() : RegionConnectionMethod<CFG,WEIGHT>() {
  SetDefault();
}

template <class CFG, class WEIGHT>
NaiveRegionConnect<CFG,WEIGHT>::
~NaiveRegionConnect() {
}

template <class CFG, class WEIGHT>
char*
NaiveRegionConnect<CFG,WEIGHT>::
GetName() {
  return "NaiveRegionConnect";
}

template <class CFG, class WEIGHT>
void
NaiveRegionConnect<CFG,WEIGHT>::
SetDefault() {

}

template <class CFG, class WEIGHT>
void
NaiveRegionConnect<CFG,WEIGHT>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  _os << "\n" << GetName() << " ";
  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG, class WEIGHT>
void 
NaiveRegionConnect<CFG,WEIGHT>::
PrintValues(ostream& _os) {
  _os << "\n" << GetName() << " ";
  _os << endl;
}

template <class CFG, class WEIGHT>
void 
NaiveRegionConnect<CFG,WEIGHT>::
PrintOptions(ostream& out_os) {
  out_os << "\n" << GetName() << " ";
  out_os << endl;
}

template <class CFG, class WEIGHT>
RegionConnectionMethod<CFG,WEIGHT>*
NaiveRegionConnect<CFG,WEIGHT>::
CreateCopy() {
  RegionConnectionMethod<CFG,WEIGHT> * _copy = 
    new NaiveRegionConnect<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void NaiveRegionConnect<CFG,WEIGHT>::
Connect(MPRegion<CFG,WEIGHT>* target_region,
	MPRegion<CFG,WEIGHT>* &l_region,
	MPRegion<CFG,WEIGHT>* &r_region,
	ConnectMap<CFG, WEIGHT>& cm,
	DistanceMetric& dm,
	LocalPlanners<CFG,WEIGHT>& lp,
	bool addPartialEdge, bool addAllEdges) {


  vector< vector< VID > > selected_vertices; // selected nodes from each region

  //Get boundary of left and right regions and print
  BoundingBox* l_boundary = l_region->GetBoundingBox();  
  BoundingBox* r_boundary = r_region->GetBoundingBox();
  cout << " Combining Regions with BoundingBoxes as follows: " << endl;
  l_boundary->Print(cout);
  r_boundary->Print(cout);

  /////////////////////  Working with Left Region ///////////////////////
  //Get Left map's vertices and copy merge map into target map
  vector<VID> l_map_vertices;
  l_region->roadmap.m_pRoadmap->GetVerticesVID(l_map_vertices);
  vector<VID> l_map_newVids = 
    target_region->roadmap.m_pRoadmap->MergeRoadMap(l_region->roadmap.m_pRoadmap,
						    l_map_vertices);
  //Push all of Left Map's CFGs into selected vector for connection
  selected_vertices.push_back(l_map_newVids);


  /////////////////////  Working with Right Region ///////////////////////
  //Get right map's vertices and copy merge map into target map
  vector<VID> r_map_vertices;
  r_region->roadmap.m_pRoadmap->GetVerticesVID(r_map_vertices);
  vector<VID> r_map_newVids = 
    target_region->roadmap.m_pRoadmap->MergeRoadMap(r_region->roadmap.m_pRoadmap,
						    r_map_vertices);
  //Push all of Left Map's CFGs into selected vector for connection
  selected_vertices.push_back(r_map_newVids);

  //Call connection method using our selected CFG
  cm.ConnectComponents(&(target_region->roadmap), target_region->combine_stats,
		       &dm, &lp, addPartialEdge, addAllEdges, 
		       selected_vertices);
}


template <class CFG, class WEIGHT>
void NaiveRegionConnect<CFG,WEIGHT>::
Connect(MPRegion<CFG,WEIGHT>* target_region, 
	       vector<MPRegion<CFG,WEIGHT>* > &source_regions, 
	       ConnectMap<CFG, WEIGHT>& cm,
	       DistanceMetric& dm,
	       LocalPlanners<CFG,WEIGHT>& lp,
	       bool addPartialEdge, bool addAllEdges) {
  
  ///TODO:Implement this function in MPRegion...
  //target_region->computeBoundary(source_regions);
  //for all pairs of ranges in vector<MPRegion<CFG,WEIGHT>>
  for (int i=0;i<source_regions.size();++i) {
    for (int j=i+1; j<source_regions.size();++j) {
      this->Connect(target_region, source_regions[i], source_regions[j],cm,dm,lp,addPartialEdge,addAllEdges);
    }
  }
}

#endif /*_NaiveRegionConnect_h_*/
