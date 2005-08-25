#ifndef ConnectCCs_h
#define ConnectCCs_h

#include "ComponentConnectionMethod.h"


//ConnectCCs
   /**Try to connect different connected components of the roadmap.
    *We try to connect all pairs of connected components. If both
    *components are small (less than "smallcc" nodes), then we try to 
    *connect all pairs of nodes.  If at least one of the components is 
    *large, we try to connect the "kpairs" closest pairs of nodes.
    *
    *@see WeightedGraph ::GetCCStats, WeightedGraph ::GetCC, 
    *WeightedGraph ::IsSameCC for information about connected 
    *component in graph,and ConnectSmallCCs, ConnectBigCCs for 
    *connecting between connected component.
    */


#define KPAIRS        4        // default for connectCCs
#define SMALL_CC      3        // default for connectCCs: smallcc size


template <class CFG, class WEIGHT>
class ConnectCCs: public ComponentConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  ConnectCCs();
  ConnectCCs(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  ConnectCCs(Roadmap<CFG,WEIGHT>*, CollisionDetection*, 
		      DistanceMetric*, LocalPlanners<CFG,WEIGHT>*);
  ~ConnectCCs();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods

  void ParseCommandLine(std::istringstream& is);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os);
  virtual ComponentConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection method
  /**Connect two small connected components.
   *Algorithm:
   *   -# for every Cfg, cfg1, in cc1
   *       -# for every Cfg. cgf2, in cc2
   *           -# if cfg1 and cfg2 could be connected.
   *           -# then add this edge to roadmap, _rm.
   *              and return.
   *           -# else if info.addPartialEdge
   *               -# then add failed path to roadmap
   *                  (cfg1->failed_point)
   *           -# end if
   *       -# end for
   *   -# end for
   *
   *@param _cc1id a node id which defines the first connected component.
        *@param _cc2id a node id which defines the second connected component.
        */
  void ConnectSmallCCs(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		       CollisionDetection *cd, LocalPlanners<CFG,WEIGHT>* lp,
		       DistanceMetric * dm, 
		       vector<CFG>& cc1vec, 
		       vector<CFG>& cc2vec,
		       bool addPartialEdge,
		       bool addAllEdges);  
  /**Connect two big connected components.
   *Algorithm:
   *   -# find k pairs of closest Cfgs from cc1 to cc2.
   *   -# for every pair, p, in k pairs.
   *       -# if p.first and p.second could be connected.
   *          (p.first is from cc1 and p.second is from cc2)
   *       -# then add this edge to roadmap, _rm.
   *          and return.
   *       -# else if info.addPartialEdge
   *           -# then add failed path to roadmap
   *              (cfg1->failed_point)
   *       -# end if
   *   -# end for
   *
   *In step 1, FindKClosestPairs is used to find first k
   *closest pairs from the first connected component to 
   *the second connected component.
   *
   *@param _cc1id a node id which defines the first connected component.
   *@param _cc2id a node id which defines the second connected component.
   *@param _cn _cn.GetKPairs define k above.
   */
  void ConnectBigCCs(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		     CollisionDetection* cd, LocalPlanners<CFG,WEIGHT>* lp,
		     DistanceMetric* dm, vector<CFG>& cc1vec, 
		     vector<CFG>& cc2vec,
		     bool addPartialEdge, bool addAllEdges);

  //@}

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
		 CollisionDetection*, 
		 DistanceMetric *,
		 LocalPlanners<CFG,WEIGHT>*,
		 bool addPartialEdge,
		 bool addAllEdges);
  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
		 CollisionDetection*, 
		 DistanceMetric *,
		 LocalPlanners<CFG,WEIGHT>*,
		 bool addPartialEdge,
		 bool addAllEdges,
		 vector<VID>& vids1, vector<VID>& vids2);

 private:
  //////////////////////
  // Data

  int kpairs;
  int smallcc;
};


///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  ConnectCCs
template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::
ConnectCCs():ComponentConnectionMethod<CFG,WEIGHT>() { 
  element_name = "components"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::
ConnectCCs(TiXmlNode* in_pNode, MPProblem* in_pProblem):
  ComponentConnectionMethod<CFG,WEIGHT>(in_pNode, in_pProblem) { 
  element_name = "components"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::ConnectCCs(Roadmap<CFG,WEIGHT> * rdmp, CollisionDetection* cd, DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp):
  ComponentConnectionMethod<CFG,WEIGHT>(rdmp, cd, dm, lp) {
  element_name = string("components");

  SetDefault();
}


template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::~ConnectCCs() { 
}


template <class CFG, class WEIGHT>
void ConnectCCs<CFG,WEIGHT>::
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
      if (is >> kpairs) {
        if (kpairs < 0)
  	  throw BadUsage();

        c = is.peek();
        while(c == ' ' || c == '\n') {
          is.get();
          c = is.peek();
        }
        if (c >= '0' && c <='9') {
          if (is >> smallcc) {
    	    if (smallcc < 0)
	      throw BadUsage();
          } else
            throw BadUsage();
        }

      } else
        throw BadUsage();
    }

  } catch (BadUsage) {
    cerr << "Error in \'components\' parameters" << endl;
    PrintUsage(cerr);
    exit(-1);
  }
}


template <class CFG, class WEIGHT>
void ConnectCCs<CFG,WEIGHT>::SetDefault() {
  kpairs = KPAIRS;
  smallcc = SMALL_CC;

}


template <class CFG, class WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER INTEGER (default kpairs:" << KPAIRS << ", smallcc:" << SMALL_CC << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " kpairs = ";
  _os << kpairs << "smallcc = " << smallcc ;
  _os << endl;
}


template <class CFG, class WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << "::  kpairs = ";
  out_os << kpairs << "  smallcc = " << smallcc ;
  out_os << endl;
}


template <class CFG, class WEIGHT>
ComponentConnectionMethod<CFG,WEIGHT>* 
ConnectCCs<CFG,WEIGHT>::
CreateCopy() {
  ComponentConnectionMethod<CFG,WEIGHT>* _copy = 
           new ConnectCCs<CFG,WEIGHT>(*this);
  return _copy;
}


//
// try to connect all pairs of cfgs in the two CCs
//
template <class CFG, class WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::
ConnectSmallCCs(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		CollisionDetection *cd, LocalPlanners<CFG,WEIGHT>* lp,
		DistanceMetric* dm, 
		vector<CFG>& cc1vec, vector<CFG>& cc2vec,
		bool addPartialEdge, bool addAllEdges) {
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  
  // created a temporary variable since GetCC requires &
  LPOutput<CFG,WEIGHT> lpOutput;
  for (int c1 = 0; c1 < cc1vec.size(); c1++){
    for (int c2 = 0; c2 < cc2vec.size(); c2++){
      Stats.IncConnections_Attempted();
      if (!_rm->m_pRoadmap->IsEdge(cc1vec[c1],cc2vec[c2]) 
          && lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,
			     cc1vec[c1],cc2vec[c2],
			     &lpOutput, 
			     connectionPosRes, connectionOriRes, 
			     (!addAllEdges)) ) {
	Stats.IncConnections_Made();
	pMap->AddEdge(cc1vec[c1], cc2vec[c2], lpOutput.edge);
	return;
      }
      else if(addPartialEdge) {
	typename vector<pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > >::iterator I;
	for(I=lpOutput.savedEdge.begin(); I!=lpOutput.savedEdge.end(); I++) {
	  CFG tmp = I->first.second;
	  if(!tmp.AlmostEqual(cc1vec[c1])) {
	    pMap->AddVertex(tmp);
	    pMap->AddEdge(cc1vec[c1], tmp, I->second);
	  }
	}
      }
    }//end for c2
  }//end for c1
}


//
// try to connect kclosest pairs of cfgs in the two CCs
//
template <class CFG, class WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::
ConnectBigCCs(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	      CollisionDetection *cd, LocalPlanners<CFG,WEIGHT>* lp, 
	      DistanceMetric* dm, 
	      vector<CFG>& cc1vec, vector<CFG>& cc2vec,
	      bool addPartialEdge, bool addAllEdges) { 
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  
  int k = (int)min(kpairs, cc2vec.size());
  
  vector< pair<CFG,CFG> > kp = 
    dm->FindKClosestPairs(_rm->GetEnvironment(),cc1vec,cc2vec,k);
  
  LPOutput<CFG,WEIGHT> lpOutput;
  for (int i = 0; i < kp.size(); i++) {
    Stats.IncConnections_Attempted();
    if (!_rm->m_pRoadmap->IsEdge(kp[i].first,kp[i].second) 
	&& lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,kp[i].first,kp[i].second,&lpOutput, connectionPosRes, connectionOriRes, (!addAllEdges)) ) {
      pMap->AddEdge(kp[i].first, kp[i].second, lpOutput.edge); 
      Stats.IncConnections_Made();
      return;
    }
    else if(addPartialEdge) {
      typename vector<pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > >::iterator I;
      for(I=lpOutput.savedEdge.begin(); I!=lpOutput.savedEdge.end(); I++) {
	CFG tmp = I->first.second;
	if(!tmp.AlmostEqual(kp[i].first)) {
	  pMap->AddVertex(tmp);
	  pMap->AddEdge(kp[i].first, tmp, I->second);
	}
      }
    }//end for c2
  }//end for c1
}



template <class CFG, class WEIGHT>
void ConnectCCs<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
          CollisionDetection* cd , 
          DistanceMetric * dm,
          LocalPlanners<CFG,WEIGHT>* lp,
          bool addPartialEdge,
	  bool addAllEdges) {
  vector< pair<int,VID> > ccs1;
  GetCCStats(*(_rm->m_pRoadmap),ccs1);
  vector< pair<int,VID> > ccs2 = ccs1;
  
#ifndef QUIET
  cout << "components(kpairs="<< kpairs ;
  cout << ", smallcc="<<smallcc <<"): "<<flush;
#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  
  // process components from smallest to biggest  
  for (int cc1 = ccs1.size()-1 ; cc1 >= 0 ; cc1--) {
    for (int cc2 = ccs2.size() - 2 ; cc2 >= 0 ; cc2--) {
      VID cc1id = ccs1[cc1].second; 
      VID cc2id = ccs2[cc2].second;
      CFG cc1Cfg = pMap->GetData(cc1id); 
      CFG cc2Cfg = pMap->GetData(cc2id); 
      
      // if cc1 & cc2 not already connected, try to connect them 
      if ( !IsSameCC(*pMap,cc1id,cc2id) ) {
        vector<CFG> cc1;
        GetCC(*pMap,cc1Cfg,cc1);
        vector<CFG> cc2;
        GetCC(*pMap,cc2Cfg,cc2);
        if(cc1.size() < smallcc && cc2.size() < smallcc ) {
          ConnectSmallCCs(_rm,Stats,cd,lp,dm,cc1,cc2,addPartialEdge,addAllEdges);
        } else {
          ConnectBigCCs(_rm,Stats,cd,lp,dm,cc1,cc2,addPartialEdge,addAllEdges);
        }
      } 
    }/*endfor cc2*/ 
    ccs2.pop_back();
  }/*endfor cc1*/
}


template <class CFG, class WEIGHT>
void ConnectCCs<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
          CollisionDetection* cd , 
          DistanceMetric * dm,
          LocalPlanners<CFG,WEIGHT>* lp,
          bool addPartialEdge,
          bool addAllEdges,
	  vector<VID> & vids1, vector<VID> & vids2) {
#ifndef QUIET
  cout << "components(kpairs="<< kpairs ;
  cout << ", smallcc="<<smallcc <<"): "<<flush;
#endif
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  
  for (int cc1 = 0; cc1 < vids1.size(); cc1++) {
    for (int cc2 = 0; cc2 < vids2.size(); cc2++) {
      VID cc1id = vids1[cc1]; 
      VID cc2id = vids2[cc2];
      CFG cc1Cfg = pMap->GetData(cc1id); 
      CFG cc2Cfg = pMap->GetData(cc2id); 
      
      // if cc1 & cc2 not already connected, try to connect them 
      if ( !IsSameCC(*pMap,cc1id,cc2id) ) {
        vector<CFG> cc1Cfgs;
        GetCC(*pMap,cc1Cfg,cc1Cfgs);
        vector<CFG> cc2Cfgs;
        GetCC(*pMap,cc2Cfg,cc2Cfgs);
        if(cc1Cfgs.size() < smallcc && cc2Cfgs.size() < smallcc ) {
          ConnectSmallCCs(_rm,Stats,cd,lp,dm,cc1Cfgs,cc2Cfgs,addPartialEdge,addAllEdges);
        } else {
          ConnectBigCCs(_rm,Stats,cd,lp,dm,cc1Cfgs,cc2Cfgs,addPartialEdge,addAllEdges);
        }
      } 
    }/*endfor cc2*/ 
  }/*endfor cc1*/
}

#endif
