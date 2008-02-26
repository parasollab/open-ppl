#ifndef RegionConnectionMethod_h
#define RegionConnectionMethod_h

#include "util.h"

#include "MPRegion.h"

// Abstract Interface Class for node connection methods
template <class CFG, class WEIGHT>
class RegionConnectionMethod : MPBaseObject { 
 public:
  
  //////////////////////
  // Constructors and Destructor
  RegionConnectionMethod();
  RegionConnectionMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  virtual ~RegionConnectionMethod();
  
  //////////////////////
  // Access
  virtual char* GetName();
  virtual void SetDefault() = 0;
  
  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(std::istringstream& is) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  ///Used in new MPProblem framework. \todo remove the "{ }" later
  virtual void PrintOptions(ostream& out_os) { };
  virtual RegionConnectionMethod<CFG, WEIGHT>* CreateCopy() = 0;
  
  //////////////////////
  // Connection methods 
  virtual void Connect(MPRegion<CFG,WEIGHT>* target_region,
		       MPRegion<CFG,WEIGHT>* &l_region, 
		       MPRegion<CFG,WEIGHT>* &r_region, 
		       ConnectMap<CFG, WEIGHT>& cm,
		       CollisionDetection& cd, DistanceMetric& dm,
		       LocalPlanners<CFG,WEIGHT>& lp,
		       bool addPartialEdge, bool addAllEdges) = 0;

  virtual void Connect(MPRegion<CFG,WEIGHT>* target_region,    // Output MPregion
	       vector<MPRegion<CFG,WEIGHT>* > &source_regions, // regions to merge
	       ConnectMap<CFG, WEIGHT>& cm,
	       CollisionDetection& cd, DistanceMetric& dm,
	       LocalPlanners<CFG,WEIGHT>& lp,
	       bool addPartialEdge, bool addAllEdges) = 0;

 protected:
  //////////////////////
  // Data
  char* element_name; //Method name in the command line
 
  //////////////////////////////////////////////////////
  // 
  // Public Data
  //
 public:
  CDInfo* cdInfo;
  double connectionPosRes, ///< Position resolution for node connection
    connectionOriRes; ///< Orientation resolution for node connection
};


template <class CFG, class WEIGHT>
RegionConnectionMethod<CFG,WEIGHT>::
RegionConnectionMethod() {
}


template <class CFG, class WEIGHT>
RegionConnectionMethod<CFG,WEIGHT>::
RegionConnectionMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
  MPBaseObject(in_pNode,in_pProblem) {
}



template <class CFG, class WEIGHT>
RegionConnectionMethod<CFG,WEIGHT>::
~RegionConnectionMethod() {
}

template <class CFG, class WEIGHT>
char* 
RegionConnectionMethod<CFG,WEIGHT>::
GetName() { 
  return element_name; 
}

#endif
