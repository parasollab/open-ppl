#ifndef ConnectFirst_h
#define ConnectFirst_h

#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"


template <class CFG, class WEIGHT>
class ConnectFirst : public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  ConnectFirst();
  ~ConnectFirst();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods

  void ParseCommandLine(std::istringstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();

  //////////////////////
  // Core: Connection method
  void Connect(Roadmap<CFG, WEIGHT>* rm, Stat_Class& Stats, 
	       CollisionDetection* cd, DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges);  
  void Connect(Roadmap<CFG, WEIGHT>* rm, Stat_Class& Stats,
	       CollisionDetection* cd, DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges,
	       vector<CFG>& cfgs1, vector<CFG>& cfgs2);
  
 private:
  //////////////////////
  // Data
  
  int kfirst;
};


template <class CFG, class WEIGHT>
ConnectFirst<CFG,WEIGHT>::
ConnectFirst() : NodeConnectionMethod<CFG,WEIGHT>() { 
  element_name = "connectfirst"; 
  SetDefault();
}


template <class CFG, class WEIGHT>
ConnectFirst<CFG,WEIGHT>::
~ConnectFirst() { 
}


template <class CFG, class WEIGHT>
void ConnectFirst<CFG,WEIGHT>::
ParseCommandLine(std::istringstream& is) {
  char c;
  SetDefault();
  try {
    c = is.peek();
    while(c == ' ' || c == '\n') {
      is.get();
      c = is.peek();
    }    
    if (c >= '0' && c <= '9') {
      if (is >> kfirst) {
        if (kfirst < 0)
          throw BadUsage();
      } else
        throw BadUsage();
    }

  } catch (BadUsage) {
    cerr << "ERROR in \'connectfirst\' parameters\n";
    PrintUsage(cerr);
    exit(-1);
  }
}


template <class CFG, class WEIGHT>
void ConnectFirst<CFG,WEIGHT>::
SetDefault() {
  kfirst = 1;
}


template <class CFG, class WEIGHT>
void
ConnectFirst<CFG, WEIGHT>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER (default " << 1 << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ConnectFirst<CFG, WEIGHT>::
PrintValues(ostream& _os) {
  _os << "\n" << GetName() << " kfirst = ";
  _os << kfirst;
  _os << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
ConnectFirst<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new ConnectFirst<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void 
ConnectFirst<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	CollisionDetection* cd, DistanceMetric* dm,
	LocalPlanners<CFG,WEIGHT>* lp,
	bool addPartialEdge, bool addAllEdges) {
  vector<CFG> cfgs;
  _rm->m_pRoadmap->GetVerticesData(cfgs);
  return Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges,
		 cfgs, cfgs);
}


template <class CFG, class WEIGHT>
void 
ConnectFirst<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	CollisionDetection* cd, DistanceMetric* dm,
	LocalPlanners<CFG,WEIGHT>* lp,
	bool addPartialEdge, bool addAllEdges,
	vector<CFG>& cfgs1, vector<CFG>& cfgs2) {
#ifndef QUIET
  cout << "connectfirst(k=" << kfirst << "): " << flush;
#endif

  typename vector<CFG>::iterator C1, C2;
  typename vector<pair<CFG,double> >::iterator D;
  for(C1 = cfgs1.begin(); C1 != cfgs1.end(); ++C1) {
    // sort cfgs2 in by distance from C
    vector<pair<CFG,double> > distances;
    for(C2 = cfgs2.begin(); C2 != cfgs2.end(); ++C2)
      distances.push_back(make_pair(*C2, dm->Distance(_rm->GetEnvironment(), 
						      *C1, *C2)));
    sort(distances.begin(), distances.end(), CfgDist_Compare<CFG>());

    //try to connect, return after failure or connect first k
    int numFound = 0;
    for(D = distances.begin(); (D != distances.end()) && (numFound < kfirst); 
	++D) {
      LPOutput<CFG,WEIGHT> _ci;
      if(!_rm->m_pRoadmap->IsEdge(*C1, D->first) && 
	 lp->IsConnected(_rm->GetEnvironment(), Stats, cd, dm, *C1, D->first, 
			 &_ci, _rm->GetEnvironment()->GetPositionRes(), 
			 _rm->GetEnvironment()->GetOrientationRes(), 
			 true, true)) {
	VID _cfgVID1, _cfgVID2;
	if(_rm->m_pRoadmap->IsVertex(*C1))
	  _cfgVID1 = _rm->m_pRoadmap->GetVID(*C1);
	else
	  _cfgVID1 = _rm->m_pRoadmap->AddVertex(*C1);
	if(_rm->m_pRoadmap->IsVertex(D->first))
	  _cfgVID2 = _rm->m_pRoadmap->GetVID(D->first);
	else
	  _cfgVID2 = _rm->m_pRoadmap->AddVertex(D->first);
	_rm->m_pRoadmap->AddEdge(_cfgVID1, _cfgVID2, _ci.edge);
	numFound++;
      } 
    }
  }
}


#endif
