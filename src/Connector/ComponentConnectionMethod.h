#ifndef ComponentConnectionMethod_h
#define ComponentConnectionMethod_h
#include "util.h"

// Abstract Interface Class for node connection methods
template <class CFG, class WEIGHT>
class ComponentConnectionMethod : public MPBaseObject{ 
 public:
  
  //////////////////////
  // Constructors and Destructor
  ComponentConnectionMethod();
  ComponentConnectionMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~ComponentConnectionMethod();
  
  //////////////////////
  // Access
  virtual char* GetName();
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
  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges) = 0;
  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges,
		       vector<VID>& vids1, vector<VID>& vids2) = 0;
  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges,
		       vector<vector<VID> >& vids) {
    vector<vector<VID> >::iterator I, J;
    for(I = vids.begin(); I+1 != vids.end(); ++I) 
      for(J = I+1; J != vids.end(); ++J)
	this->Connect(rm, Stats, dm, lp, addPartialEdge, addAllEdges,
		      *I, *J);
  }

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
ComponentConnectionMethod<CFG,WEIGHT>::
ComponentConnectionMethod() {
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

template <class CFG, class WEIGHT>
char* 
ComponentConnectionMethod<CFG,WEIGHT>::
GetName() { 
  return element_name; 
}

#endif
