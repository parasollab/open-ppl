#ifndef ConnectFirst_h
#define ConnectFirst_h
#include "ConnectionMethod.h"


template <class CFG, class WEIGHT>
class ConnectFirst : public ConnectionMethod<CFG,WEIGHT> {
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
  virtual ConnectionMethod<CFG, WEIGHT>* CreateCopy();

  //////////////////////
  // Core: Connection method
  void ConnectComponents();
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge, bool addAllEdges);  
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
			 CollisionDetection*, DistanceMetric*,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge, bool addAllEdges,
			 vector<vector<CFG> >& verticesList);
  void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge, bool addAllEdges,
			 CFG& cfg, vector<CFG>& vec);

 private:
  //////////////////////
  // Data

  int kfirst;
};


template <class CFG, class WEIGHT>
ConnectFirst<CFG,WEIGHT>::
ConnectFirst() : ConnectionMethod<CFG,WEIGHT>() { 
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
ConnectionMethod<CFG,WEIGHT>* 
ConnectFirst<CFG,WEIGHT>::
CreateCopy() {
  ConnectionMethod<CFG,WEIGHT>* _copy = 
           new ConnectFirst<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void 
ConnectFirst<CFG,WEIGHT>::
ConnectComponents() {
}


template <class CFG, class WEIGHT>
void ConnectFirst<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
                  CollisionDetection* cd , 
                  DistanceMetric * dm,
                  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges) {
#ifndef QUIET
  cout << "connectfirst(k="<< kfirst <<"): "<<flush;
#endif

  //do nothing for now...  

}


template <class CFG, class WEIGHT>
void 
ConnectFirst<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
                  CollisionDetection* cd , 
                  DistanceMetric * dm,
                  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges,
		  vector<vector<CFG> >& verticesList) {

  //assume that first "vector" has the 1 cfg you want to connect...
  for(int i=1; i<verticesList.size(); ++i)
    ConnectComponents(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges,
		      verticesList[0][0], verticesList[i]);
}


template <class CFG, class WEIGHT>
void 
ConnectFirst<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
                  CollisionDetection* cd , 
                  DistanceMetric * dm,
                  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges,
		  CFG& cfg, vector<CFG>& vec) {
#ifndef QUIET
  cout << "connectfirst*(k="<< kfirst <<"): "<<flush;
#endif

  // sort the cfgs in vec by distance from cfg
  SortByDistFromCfg(_rm->GetEnvironment(),dm,cfg,vec);
   
  //try to connect, return after failure or connect first k
  int numFound = 0;
  for(typename vector<CFG>::iterator I = vec.begin(); I != vec.end(); ++I) {
    LPOutput<CFG,WEIGHT> _ci;
    if(!_rm->m_pRoadmap->IsEdge(cfg, *I) && 
       lp->IsConnected(_rm->GetEnvironment(), Stats, cd, dm, cfg, *I, 
		       &_ci, _rm->GetEnvironment()->GetPositionRes(), 
		       _rm->GetEnvironment()->GetOrientationRes(), 
		       true, true)) {
       VID _cfgVID;
       if(_rm->m_pRoadmap->IsVertex(cfg))
	 _cfgVID = _rm->m_pRoadmap->GetVID(cfg);
       else
	 _cfgVID = _rm->m_pRoadmap->AddVertex(cfg);
       _rm->m_pRoadmap->AddEdge(_cfgVID, _rm->m_pRoadmap->GetVID(*I), 
				_ci.edge);
       numFound++;
       if(numFound >= kfirst)
	 return;
     } 
   }
}

#endif
