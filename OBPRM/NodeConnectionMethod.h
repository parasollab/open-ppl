#ifndef NodeConnectionMethod_h
#define NodeConnectionMethod_h

// Abstract Interface Class for node connection methods
template <class CFG, class WEIGHT>
class NodeConnectionMethod { 
 public:
  
  //////////////////////
  // Constructors and Destructor
  NodeConnectionMethod();
  ~NodeConnectionMethod();
  
  //////////////////////
  // Access
  virtual char* GetName();
  virtual void SetDefault() = 0;
  
  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(std::istringstream& is) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;   
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy() = 0;
  
  //////////////////////
  // Connection methods 
  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       CollisionDetection* cd, DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges) = 0;
  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       CollisionDetection* cd, DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges,
		       vector<CFG>& cfgs1, vector<CFG>& cfgs2) = 0;
  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       CollisionDetection* cd, DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges,
		       vector<vector<CFG> >& cfgs) {
    vector<vector<CFG> >::iterator I, J;
    for(I = cfgs.begin(); I+1 != cfgs.end(); ++I) 
      for(J = I+1; J != cfgs.end(); ++J)
	this->Connect(rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges,
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
NodeConnectionMethod() {
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>::
~NodeConnectionMethod() {
}

template <class CFG, class WEIGHT>
char* 
NodeConnectionMethod<CFG,WEIGHT>::
GetName() { 
  return element_name; 
}

#endif
