#ifndef _RegionOverlapMapCombine_h_
#define _RegionOverlapMapCombine_h_

#include "ConnectionMethod.h"


template <class CFG, class WEIGHT>
class RegionOverlapMapCombine: public ConnectionMethod<CFG,WEIGHT> {
 public:
  RegionOverlapMapCombine();
  ~RegionOverlapMapCombine();
  char* GetName();
  void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual ConnectionMethod<CFG,WEIGHT>* CreateCopy();


  void ConnectComponents();
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, StatClass& Stats, 
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
RegionOverlapMapCombine<CFG,WEIGHT>::
RegionOverlapMapCombine() : ConnectionMethod<CFG,WEIGHT>() {
  SetDefault();
}

template <class CFG, class WEIGHT>
RegionOverlapMapCombine<CFG,WEIGHT>::
~RegionOverlapMapCombine() {
}

template <class CFG, class WEIGHT>
char*
RegionOverlapMapCombine<CFG,WEIGHT>::
GetName() {
  return "RegionOverlapMapCombine";
}

template <class CFG, class WEIGHT>
void
RegionOverlapMapCombine<CFG,WEIGHT>::
SetDefault() {

}

template <class CFG, class WEIGHT>
void
RegionOverlapMapCombine<CFG,WEIGHT>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  _os << "\n" << GetName() << " ";
  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG, class WEIGHT>
void 
RegionOverlapMapCombine<CFG,WEIGHT>::
PrintValues(ostream& _os) {
  _os << "\n" << GetName() << " ";
  _os << endl;
}

template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>*
RegionOverlapMapCombine<CFG,WEIGHT>::
CreateCopy() {
  ConnectionMethod<CFG,WEIGHT> * _copy = new RegionOverlapMapCombine<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void RegionOverlapMapCombine<CFG,WEIGHT>::
ConnectComponents() {
  cout << "DOING NOTHING" << endl;
}
 

template <class CFG, class WEIGHT>
void RegionOverlapMapCombine<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
		  CollisionDetection* cd , 
		  DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge, 
		  bool addAllEdges) {
  cout << "DOING NOTHING" << endl;
}


template <class CFG, class WEIGHT>
void RegionOverlapMapCombine<CFG,WEIGHT>::
CombineRegions(MPRegion<CFG,WEIGHT>* target_region,
	       MPRegion<CFG,WEIGHT>* &l_subregion,
	       MPRegion<CFG,WEIGHT>* &r_subregion,
	       ConnectMap<CFG, WEIGHT>& cm,
	       CollisionDetection& cd, DistanceMetric& dm,
	       LocalPlanners<CFG,WEIGHT>& lp,
	       bool addPartialEdge, bool addAllEdges) {

  cout << "I am in RegionOverlapMapCombine::CombineRegions()" << endl;
 

  ///////////////////////////////////////////
  ///
  ///             PseudoCode
  ///
  ///////////////////////////////////////////
  ///
  /// 1) get l_subregions nodes
  /// 2) get r_subregions nodes
  /// 3) calculate which nodes are in overlaping region
  /// 4) create vector of these nodes for l_subregion
  /// 5) create vector of these nodes for r_subregion
  /// 6) call a connection method for these groups of nodes
  ///////////////////////////////////////////
  


}


template <class CFG, class WEIGHT>
void RegionOverlapMapCombine<CFG,WEIGHT>::
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

#endif /*_RegionOverlapMapCombine_h_*/
