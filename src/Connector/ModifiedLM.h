#ifndef ModifiedLM_h
#define ModifiedLM_h
#include "ConnectionMethod.h"
#include "BasicOBPRM.h"


//--------------------------------------------------------------------
//   "modifiedLM" -- modified Laumond's method. During connection 
//   phase, nodes are randomly generated, they are kept if they can 
//   be connected to no CCs or more than one CCs, otherwise it will 
//   be tossed away(only connected to one CC) if its 'distance' from 
//   the 'center' of that CC not larger than user-specified 'r' times 
//   the radius of that CC, i.e, it will be kept only when it 
//   'expand's that CC.
//
//   Parameters:
//
//   1) kclosest: num of closest nodes in each CC that this node is
//		going to try connection.
//   2) maxNum: the maximum numbers of nodes that are going to be
//		added into the roadmap during this.
//   3) rfactor: multiplier for 'radius' of CC, w/in which thrown out, 
//		outside of which kept.
// Pseudo-code
//------------
//   while (more than one CC remains *AND* added fewer new Cfgs than requested)
//      generate a random configuration, cfg
//      get current connected components from roadmap
//      for each connected component, CC
//           if possible to connect cfg to CC
//                 increment count of connections, #connections
//           endif
//      endfor
//      if (#connections is zero *OR* #connections greater than one )
//         *OR*
//         (#connections is one *AND* cfg distance to CCcenter > rfactor * CCradius)
//                increment count of new Cfgs added
//                add cfg & all edges
//      endif
//   endwhile
// ------------------------------------------------------------------


#define KCLOSEST      5
#define MAXNUM        20        
#define RFACTOR       2        


template <class CFG, class WEIGHT>
class ModifiedLM: public ConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  ModifiedLM();
  ~ModifiedLM();
 
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
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges);

 private:
  //////////////////////
  // Data
        
  int kclosest;
  int    maxNum;            ///< used by modifiedLM
  double rfactor;           ///< used by modifiedLM
};


///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  ModifiedLM
template <class CFG, class WEIGHT>
ModifiedLM<CFG,WEIGHT>::ModifiedLM():ConnectionMethod<CFG,WEIGHT>() { 
  element_name = "modifiedLM"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
ModifiedLM<CFG,WEIGHT>::~ModifiedLM() { 
}


template <class CFG, class WEIGHT>
void ModifiedLM<CFG,WEIGHT>::
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
      if (is >> kclosest) {
        if (kclosest < 0)
	  throw BadUsage();

        c = is.peek();
        while(c == ' ' || c == '\n') {
          is.get();
          c = is.peek();
        }    
        if (c >= '0' && c <= '9') {
          if (is >> maxNum) {
            if (maxNum < 0)
	      throw BadUsage();

            c = is.peek();
            while(c == ' ' || c == '\n') {
              is.get();
              c = is.peek();
            }    
            if (c >= '0' && c <= '9') {
	      if (is >> rfactor) {
	        if (rfactor < 0)
	          throw BadUsage();
	      } else
                throw BadUsage();
            }

          } else
            throw BadUsage();
        }

      } else
        throw BadUsage();
    }

  } catch (BadUsage) {
    cerr << "Error in \'modifiedLM\' parameters" << endl;
    PrintUsage(cerr);
    exit(-1);
  }
}


template <class CFG, class WEIGHT>
void ModifiedLM<CFG,WEIGHT>::SetDefault() {
  kclosest = KCLOSEST;
  maxNum = MAXNUM;        
  rfactor = RFACTOR; 
}


template <class CFG, class WEIGHT>
void
ModifiedLM<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER INTEGER INTEGER (default kclosest=" << KCLOSEST 
      << ", maxNum=" << MAXNUM 
      << ", rfactor=" << RFACTOR 
      << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ModifiedLM<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " kclosest = ";
  _os << kclosest;
  _os << ", maxNum = " << maxNum;
  _os << ", rfactor = " << rfactor;
  _os << endl;
}


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>* 
ModifiedLM<CFG,WEIGHT>::
CreateCopy() {
  ConnectionMethod<CFG,WEIGHT>* _copy = 
           new ModifiedLM<CFG,WEIGHT>(*this);
  return _copy;
}


template <class CFG, class WEIGHT>
void ModifiedLM<CFG,WEIGHT>::
ConnectComponents() {
}

 
template <class CFG, class WEIGHT>
void ModifiedLM<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		  CollisionDetection* cd , 
		  DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges) {
  cout << "Connecting with modifiedLM" << endl;
  #ifndef QUIET
    // display information specific to method
    cout << "(kclosest=" << kclosest;
    cout << ", maxNum=" << maxNum;
    cout << ", rfactor=" << rfactor<< "): "<<flush;
  #endif
    const int requested = maxNum;

  // get local copies of necessary data
  Environment *env = _rm->GetEnvironment();
  int numMultiBody = env->GetMultiBodyCount();
  int robot        = env->GetRobotIndex();
  BasicOBPRM<CFG> gn = BasicOBPRM<CFG>();
  gn.cdInfo = cdInfo;
  vector<CFG> nodes;
  int numNodes = 1;             // only need one new node at a time
  
  // init counter
  int numCfgAdded  = 0;
  
  //-- while (more than one CC remains *AND* added fewer new Cfgs than requested)
  vector<pair<int, VID> > tempv;
  while(GetCCStats(*(_rm->m_pRoadmap),tempv) > 1 && numCfgAdded<requested) {
    //init counter
    int numTries=0;
    
    CFG cfg;
    while ( numTries++ < MAXNUM ){      
      nodes.erase(nodes.begin(), nodes.end());
      
      //-- generate new 'cfg'
      gn.GenerateNodes(env,Stats,cd,dm,nodes);
      
      // if a cfg node generated exit loop else keep trying...
      if ( nodes.size() > 0 ) 
	break;
      
    }//endwhile numTries
    
     // OBPRM-variants may return more than one node because of "shells"
     // and multiple obstacles to find the surface of so go thru all nodes returned
    for (int k=nodes.size()-1;k>=0;k--){
      // try to add this cfg
      cfg = nodes[k];
      
      // declare data structures necessary to return connection info
      vector< pair<CFG,CFG> > edges;
      vector< pair<WEIGHT,WEIGHT> > edgelpinfos;
      
      // init counter
      int numofConnection=0;
      
      //-- get current connected components from roadmap
      vector< pair<int,VID> > allCC;
      GetCCStats(*(_rm->m_pRoadmap), allCC);
      
      //-- for each connected component, CC
      LPOutput<CFG,WEIGHT> lpOutput;

      for(int i=0; i<allCC.size(); ++i) {	
	CFG          tmp = _rm->m_pRoadmap->GetData(allCC[i].second);
	vector<CFG>   CC;
	GetCC(*(_rm->m_pRoadmap),tmp,CC);
	
	vector< pair<CFG,CFG> > kp = dm->FindKClosestPairs(env,
							   cfg, CC, kclosest);
	
	//-- if possible to connect cfg to CC
	for(int j=0; j<kp.size(); ++j) {
          if (!_rm->m_pRoadmap->IsEdge(kp[j].first,kp[j].second)
              && lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm, kp[j].first,kp[j].second,&lpOutput, _rm->GetEnvironment()->GetPositionRes(), _rm->GetEnvironment()->GetOrientationRes(), (!addAllEdges) ))  {
	    //-- increment count of connections, #connections
	    ++numofConnection;
	    
	    // record edge in case we decide to add it later 
	    edges.push_back(kp[j]);
	    edgelpinfos.push_back(lpOutput.edge);
	    
	    // only need one connection per CC so exit for j loop
	    break;
	    
	  } //endif (lp-> ...)
	  
	} //endfor j
	
      } //endfor i
      
      // default decision is to not add cfg
      bool addExpandingCfg = false;
      
      // if only connected to one CC 
      if(numofConnection == 1) {	
	// get all cfg's in CC
	vector<CFG> CC;
	GetCC(*(_rm->m_pRoadmap), edges[0].second,CC);
	// calculate CC's center (of mass), CCcenter
	CFG CCcenter; // sum initialized to 0 by constructor.
	int i;
	for(i=0; i<CC.size(); ++i) {
	  double centerWeight = float(i)/(i+1);
	  CCcenter.WeightedSum(CC[i], CCcenter, centerWeight);
	}
	
	// calculate CCradius 
	double CCradius = 0.0;
	for(i=0; i<CC.size(); ++i) {
	  CCradius += dm->Distance(env, CCcenter, CC[i]);
	}
	CCradius /= CC.size();
	
	// calculate distance of cfg to CCcenter 
	double distFromCenter = dm->Distance(env, CCcenter, cfg);
	
	// if cfg distance to CCcenter > rfactor * CCradius
	if(distFromCenter > rfactor * CCradius) {
	  // adding cfg 'expands' CC sufficiently so decide to add cfg
	  addExpandingCfg = true;
	}
	
      }//endif(numofConnection == 1)
      
      
      //-- if (#connections is zero *OR* #connections greater than one )
      //-- *OR*
      //-- (#connections is one *AND* cfg distance to CCcenter > rfactor * CCradius)
      if ( numofConnection != 1  ||  addExpandingCfg ) {	
	//-- increment count of new Cfgs added
	++numCfgAdded;
	
	//-- add cfg & all edges
	_rm->m_pRoadmap->Graph<DG<CFG,WEIGHT>, NMG<CFG,WEIGHT>, WG<CFG,WEIGHT>, CFG, WEIGHT>::AddVertex(cfg);
	for(int i=0; i<edges.size(); ++i) {
	  _rm->m_pRoadmap->AddEdge(edges[i].first,  // always 'cfg'
				   edges[i].second, // cfg in CC
				   edgelpinfos[i]); 
	}//endfor i
	
      } //endif (numofConnection != 1 || addExpandingCfg)
      
    } //endfor k
    
  } //endwhile

}

#endif
