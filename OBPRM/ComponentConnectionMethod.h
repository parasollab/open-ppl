#ifndef ComponentConnectionMethod_h
#define ComponentConnectionMethod_h

// Abstract Interface Class for node connection methods
template <class CFG, class WEIGHT>
class ComponentConnectionMethod { 
 public:
  
  //////////////////////
  // Constructors and Destructor
  ComponentConnectionMethod();
  ~ComponentConnectionMethod();
  
  //////////////////////
  // Access
  virtual char* GetName();
  virtual void SetDefault() = 0;
  
  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(std::istringstream& is) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;   
  virtual ComponentConnectionMethod<CFG, WEIGHT>* CreateCopy() = 0;
  
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
		       vector<VID>& vids1, vector<VID>& vids2) = 0;
  virtual void Connect(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
		       CollisionDetection* cd, DistanceMetric* dm,
		       LocalPlanners<CFG,WEIGHT>* lp,
		       bool addPartialEdge, bool addAllEdges,
		       vector<vector<VID> >& vids) {
    vector<vector<VID> >::iterator I, J;
    for(I = vids.begin(); I+1 != vids.end(); ++I) 
      for(J = I+1; J != vids.end(); ++J)
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


template <class CFG, class WEIGHT>
ComponentConnectionMethod<CFG,WEIGHT>::
ComponentConnectionMethod() {
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
