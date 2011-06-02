#ifndef ComponentConnectionMethod_h
#define ComponentConnectionMethod_h
#include "util.h"
#include "RoadmapGraph.h"

// Abstract Interface Class for node connection methods
template <class CFG, class WEIGHT>
class ComponentConnectionMethod : public MPBaseObject{ 
 public:
  typedef typename RoadmapGraph<CFG,WEIGHT>::vertex_descriptor VID;  
  //////////////////////
  // Constructors and Destructor
  ComponentConnectionMethod();
  ComponentConnectionMethod(string elem_name, CDInfo* cd, double connPosRes, double connOriRes);
  ComponentConnectionMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~ComponentConnectionMethod();
  
  //////////////////////
  // Access
  virtual void SetDefault() = 0;
  
  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;   
  ///Used in new MPProblem framework. \todo remove the "{ }" later
  virtual void PrintOptions(ostream& out_os) { };
  virtual ComponentConnectionMethod<CFG, WEIGHT>* CreateCopy() = 0;
  
  //////////////////////
  // Connection methods 
  /*virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges) = 0;
  virtual void Connect(Roadmap<CFG,WEIGHT>* _rm, Stat_Class& Stats,
		       DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges,
		       vector<VID>& vids1, vector<VID>& vids2)=0;
  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges,
		       vector<vector<VID> >& vids) {
    typename vector<vector<VID> >::iterator I, J;
    for(I = vids.begin(); I+1 != vids.end(); ++I) 
      for(J = I+1; J != vids.end(); ++J)
	this->Connect(rm, Stats, dm, lp, addPartialEdge, addAllEdges,
		      *I, *J);
  }*/

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
ComponentConnectionMethod<CFG,WEIGHT>::
ComponentConnectionMethod() {
}

template <class CFG, class WEIGHT>
ComponentConnectionMethod<CFG,WEIGHT>::
ComponentConnectionMethod(string elem_name, CDInfo* cd, double connPosRes, double connOriRes): cdInfo(cd), connectionPosRes(connPosRes), connectionOriRes(connOriRes) {
  this->SetName(elem_name);
}

template <class CFG, class WEIGHT>
ComponentConnectionMethod<CFG,WEIGHT>::
ComponentConnectionMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
    MPBaseObject(in_Node,in_pProblem){
}




template <class CFG, class WEIGHT>
ComponentConnectionMethod<CFG,WEIGHT>::
~ComponentConnectionMethod() {
}

#endif
