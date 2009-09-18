#ifndef NodeConnectionMethod_h
#define NodeConnectionMethod_h

#include "util.h"

// Abstract Interface Class for node connection methods
template <class CFG, class WEIGHT>
class NodeConnectionMethod : public MPBaseObject { 
 public:
 typedef typename RoadmapGraph<CFG, WEIGHT>::vertex_descriptor VID; 
  //////////////////////
  // Constructors and Destructor
  NodeConnectionMethod();
  NodeConnectionMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~NodeConnectionMethod();
  
  //////////////////////
  // Access
  char* GetName() const;
  virtual void SetDefault() = 0;
  
  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  virtual void ParseXML(XMLNodeReader& in_Node) {
    m_CheckIfSameCC = in_Node.boolXMLParameter(string("CheckIfSameCC"), false, true, string("check if same cc boolean"));
  }
  virtual void PrintOptions(ostream& out_os) { 
    out_os << "    " << GetName() << "::\n";
    out_os << "      CheckIfSameCC = " << m_CheckIfSameCC << endl;
  }
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy() = 0;
  
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
		       vector<VID>& cfgs1, vector<VID>& cfgs2) = 0;
  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges,
		       vector<vector<VID> >& cfgs) {
    typename vector<vector<VID> >::iterator I, J;
    for(I = cfgs.begin(); I+1 != cfgs.end(); ++I) 
      for(J = I+1; J != cfgs.end(); ++J)
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
  bool m_CheckIfSameCC; ///< Check if Same Connected Component
};


template <class CFG>
struct CfgDist_Compare : public binary_function<pair<CFG,double>,
						pair<CFG,double>,
						bool> {
  bool operator()(const pair<CFG,double>& p1, 
		  const pair<CFG,double>&p2) const {
    return (p1.second < p2.second);
  }
};


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>::
NodeConnectionMethod() : m_CheckIfSameCC(true) {
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>::
NodeConnectionMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPBaseObject(in_Node,in_pProblem) {
  m_CheckIfSameCC  = in_Node.boolXMLParameter(string("CheckIfSameCC"),false,
                                              true,
                                              string("Check if same CC"));
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>::
~NodeConnectionMethod() {
}

template <class CFG, class WEIGHT>
char* 
NodeConnectionMethod<CFG,WEIGHT>::
GetName() const { 
  return element_name; 
}

#endif
