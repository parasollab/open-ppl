#ifndef NodeConnectionMethod_h
#define NodeConnectionMethod_h

#include "util.h"

// Abstract Interface Class for node connection methods
template <class CFG, class WEIGHT> class ConnectMap;

template <class CFG, class WEIGHT>
class NodeConnectionMethod : public MPBaseObject {
 private:
 ConnectMap<CFG,WEIGHT>* my_connect;
 
 public:
 typedef typename RoadmapGraph<CFG, WEIGHT>::vertex_descriptor VID; 
  //////////////////////
  // Constructors and Destructor
  NodeConnectionMethod();
  NodeConnectionMethod(string elem_name, vector<pair<pair<VID, VID>, bool> > conn_attempts, CDInfo* cd, double connPosRes, double connOriRes, bool checkCC ); 
  NodeConnectionMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~NodeConnectionMethod();
  
  
  ConnectMap<CFG,WEIGHT>* GetConnectMap() {
  LOG_DEBUG_MSG("NodeConnectionMethod::ConnectMap()");
  return my_connect;
  };
  
  //////////////////////
  // Access
  virtual void SetDefault() = 0;
  
  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  virtual void ParseXML(XMLNodeReader& in_Node) {
    m_CheckIfSameCC = in_Node.boolXMLParameter(string("CheckIfSameCC"), false, true, string("check if same cc boolean"));
  }
  virtual void PrintOptions(ostream& out_os) { 
    out_os << "    " << this->GetName() << "::\n";
    out_os << "      CheckIfSameCC = " << m_CheckIfSameCC << endl;
  }
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy() = 0;
  
  //////////////////////
  // Connection methods 
/*  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges) = 0;*/
/*  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges,
		       vector<VID>& cfgs1, vector<VID>& cfgs2) = 0;*/
/*
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
*/

  typename vector<pair<pair<VID, VID>, bool> >::const_iterator connection_attempts_begin() const { return connection_attempts.begin(); }
  typename vector<pair<pair<VID, VID>, bool> >::const_iterator connection_attempts_end() const { return connection_attempts.end(); }
  void clear_connection_attempts() { connection_attempts.clear(); }
  
 protected:
  //////////////////////
  // Data
  vector<pair<pair<VID, VID>, bool> > connection_attempts;
 
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
NodeConnectionMethod(string elem_name, vector<pair<pair<VID, VID>, bool> > conn_attempts, CDInfo* cd, double connPosRes,
double connOriRes, bool checkCC = true ) : connection_attempts(conn_attempts), cdInfo(cd), connectionPosRes(connPosRes), connectionOriRes(connOriRes), m_CheckIfSameCC(checkCC)
{
  this->SetName(elem_name);
}

template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>::
NodeConnectionMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPBaseObject(in_Node,in_pProblem) {
  m_CheckIfSameCC  = in_Node.boolXMLParameter(string("CheckIfSameCC"),false,
                                              true,
                                              string("If true, do not connect if edges are in the same CC"));
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>::
~NodeConnectionMethod() {
}

#endif
