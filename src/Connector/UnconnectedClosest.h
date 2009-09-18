#ifndef UnconnectedClosest_h
#define UnconnectedClosest_h
#include "ConnectionMethod.h"


//Connect K Unconnected Closest
   /**Connect nodes in map to their k closest neighbors not in same cc.
    *
    *@param info provides information other than connection, like
    *collision dection, local planner, and distance metrics.
    *@param _cn provides information for specific node connection 
    *paramters.
    *@param lp Local planner for connecting given 2 Cfgs.
    *
    *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected
    */


#define KUNCONNECTEDCLOSEST 5 


template <class CFG, class WEIGHT>
class UnconnectedClosest: public ConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  UnconnectedClosest();
  ~UnconnectedClosest();
 
  //////////////////////
  // Access
  void SetDefault();

  //////////////////////
  // I/O methods
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

  /* void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges
			 vector<CFG>& v1, vector<CFG>& v2);*/


void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
			 CollisionDetection*, DistanceMetric*,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge, bool addAllEdges,
			 vector<vector<CFG> >& verticesList);






 void ConnectComponents(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
			 CollisionDetection*, DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge, bool addAllEdges,
			 vector<CFG>& v1, vector<CFG>& v2);




 private:
  //////////////////////
  // Data

  int kclosest;
};


template <class CFG, class WEIGHT>
UnconnectedClosest<CFG,WEIGHT>::UnconnectedClosest():ConnectionMethod<CFG,WEIGHT>() { 
  element_name = "unconnectedClosest"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
UnconnectedClosest<CFG,WEIGHT>::~UnconnectedClosest() { 
}


template <class CFG, class WEIGHT>
void UnconnectedClosest<CFG,WEIGHT>::SetDefault() {
  kclosest = KUNCONNECTEDCLOSEST;
}


template <class CFG, class WEIGHT>
void
UnconnectedClosest<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER (default " << KUNCONNECTEDCLOSEST << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
UnconnectedClosest<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " kclosest = ";
  _os << kclosest;
  _os << endl;
}


template <class CFG, class WEIGHT>
ConnectionMethod<CFG,WEIGHT>* 
UnconnectedClosest<CFG,WEIGHT>::
CreateCopy() {
  ConnectionMethod<CFG,WEIGHT>* _copy = 
           new UnconnectedClosest<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void UnconnectedClosest<CFG,WEIGHT>::
ConnectComponents() {
  //cout << "Connecting CCs with method: unconnectedClosest k="<< kclosest << endl ;
  //cout << "DOING NOTHING" << endl;
}


template <class CFG, class WEIGHT>
void UnconnectedClosest<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
                  CollisionDetection* cd , 
                  DistanceMetric * dm,
                  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges) {
  //cout << "Connecting CCs with method: unconnectedClosest k="<< kclosest << endl;
#ifndef QUIET
  cout << "unconnectedClosest(k="<< kclosest <<"): "<<flush;
#endif

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<CFG> vertices;
  pMap->GetVerticesData(vertices);
  const int k = min(kclosest,vertices.size());

  bool done = false;
  for(int z=0; (z<vertices.size())&&!done;++z) {  
    vector< pair<VID,VID> > kp;
    // Find k closest cfgs to each cfg in the roadmap
    if(k < vertices.size()) {
      kp = dm->KUnconnectedClosest(_rm, vertices[z], vertices, k); 
    } 
    else { // all the pairs
      for(int i=0; i<vertices.size(); ++i)
	for(int j=0; j<vertices.size(); ++j){
	  if( vertices[i]==vertices[j] ) continue;
	  kp.push_back(pair<VID,VID>(pMap->GetVID(vertices[i]), pMap->GetVID(vertices[j])));
	  done=true;
	}
    }
    
   stapl::vector_property_map< stapl::stapl_color<size_t> > cmap; 
    //	 for each pair identified
    LPOutput<CFG,WEIGHT> lpOutput;
    for (int j=0; j < kp.size(); j++) {
      if( _rm->m_pRoadmap->IsEdge(kp[j].first, kp[j].second)) continue;
      cmap.reset();
      if(this->m_CheckIfSameCC && is_same_cc(*(_rm->m_pRoadmap), cmap, kp[j].first,kp[j].second)) continue;
      Stats.IncConnections_Attempted();
      if (lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,
			  _rm->m_pRoadmap->find_vertex(kp[j].first).property(),
			  _rm->m_pRoadmap->find_vertex(kp[j].second).property(),
			  &lpOutput,
			  connectionPosRes, connectionOriRes, 
			  (!addAllEdges) )) {
	_rm->m_pRoadmap->AddEdge(kp[j].first, kp[j].second, lpOutput.edge);
	Stats.IncConnections_Made();
      }
    } //endfor j
  }//endfor z
}




template <class CFG, class WEIGHT>
void  UnconnectedClosest<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
                  CollisionDetection* cd , 
                  DistanceMetric * dm,
                  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges,
		  vector<vector<CFG> >& verticesList) {
  for(int i=0; i<verticesList.size()-1; ++i)
    for(int j=i+1; j<verticesList.size(); ++j)
      ConnectComponents(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges,
			verticesList[i], verticesList[j]);
}


template <class CFG, class WEIGHT>
void  UnconnectedClosest<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
                  CollisionDetection* cd , 
                  DistanceMetric * dm,
                  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges,
		  vector<CFG>& v1, vector<CFG>& v2) {
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
#ifndef QUIET
  cout << "UnconnectedClosest2b(k="<< kclosest <<"): "<<flush;
#endif
  

  cout << "V1 size: " << v1.size() << endl;
  cout << "V2 size: " << v2.size() << endl;
  //vector<CFG> vec1;
  //_rm->m_pRoadmap->GetVerticesData(vec1);
  const int verticeSize = v1.size();//vec1.size();
  const int k = min(kclosest,verticeSize);
  //vector<CFG> vec2 = vec1;

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  //  vector< pair<VID,VID> > kp;
  // Find k closest cfgs to each cfg in the roadmap
  /*  if(k < v2.size() - 1) {
    kp = dm->FindKClosestPairs(_rm, v1, v2, k); 
    kp = dm->FindKClosestPairs(_rm, v2, v1, k);
  } 

  else { // all the pairs
    for(int i=0; i<v1.size(); ++i)
      for(int j=0; j<v2.size(); ++j){
        if( v1[i]==v2[j] ) continue;
        kp.push_back(pair<VID,VID>(pMap->GetVID(v1[i]), pMap->GetVID(v2[j])));
      }
      }
  // for each pair identified
  LPOutput<CFG,WEIGHT> lpOutput;
  for (int j=0; j < kp.size(); j++) {
    if( _rm->m_pRoadmap->IsEdge(kp[j].first, kp[j].second)) continue;
    if(this->m_CheckIfSameCC && Is SameCC(*(_rm->m_pRoadmap), kp[j].first,kp[j].second)) continue;
    if (lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,
                        _rm->m_pRoadmap->find_vertex(kp[j].first).property(),
                        _rm->m_pRoadmap->find_vertex(kp[j].second).property(),
                        &lpOutput,
                        connectionPosRes, connectionOriRes, 
                        (!addAllEdges) )) {
      _rm->m_pRoadmap->AddEdge(kp[j].first, kp[j].second, lpOutput.edge);
    }
  } //endfor j*/
  bool done = false;
  vector< pair<VID,VID> > kp;
   vector< pair<VID,VID> > kp_temp;
  for(int r=0; r<v1.size(); r++)
    {
      for(int z=0; (z<v2.size())&&!done;z++) {  
	
	// Find k closest cfgs to each cfg in the roadmap
	if(k < v2.size()) {
	  kp_temp = dm->KUnconnectedClosest(_rm, v1[r], v2, k); 
	  kp.insert(kp.end(),kp_temp.begin(),kp_temp.end());
	  
	  //  cout << "calling dm->KUnconnectedClosest with v2.size = " << v2.size() << endl;
	} 
	else { // all the pairs
	  for(int i=0; i<v2.size(); ++i)
	    {
	      //	for(int j=0; j<vertices.size(); ++j){
	      if( v1[r]==v2[i] ) continue;
	      kp.push_back(pair<VID,VID>(pMap->GetVID(v1[r]), pMap->GetVID(v2[i])));

	      //	      done=true;
	    }
	  //}
	}
      }
    }

  /*
  
 for(int r=0; r<v2.size(); r++)
    {
      for(int z=0; (z<v1.size())&&!done;++z) {  
	
	// Find k closest cfgs to each cfg in the roadmap
	if(k < v1.size()) {
	  kp_temp = dm->KUnconnectedClosest(_rm, v2[r], v1, k); 
	  kp.insert(kp.end(),kp_temp.begin(),kp_temp.end());
	} 
	else { // all the pairs
	  for(int i=0; i<v1.size(); ++i)
	    {
	      //	for(int j=0; j<vertices.size(); ++j){
	      if( v2[r]==v1[i] ) continue;
	      kp.push_back(pair<VID,VID>(pMap->GetVID(v2[r]), pMap->GetVID(v1[i])));
	      //   done=true;
	    }
	  //}
	}
      }
    }
  */
	//	 for each pair identified
	LPOutput<CFG,WEIGHT> lpOutput;
        stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
	for (int j=0; j < kp.size(); j++) {
	  if( _rm->m_pRoadmap->IsEdge(kp[j].first, kp[j].second)) continue;
	  cmap.reset();
	  if(this->m_CheckIfSameCC && is_same_cc(*(_rm->m_pRoadmap), cmap, kp[j].first, kp[j].second)) continue;
	  Stats.IncConnections_Attempted();
	  if (lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,
			      _rm->m_pRoadmap->find_vertex(kp[j].first).property(),
			      _rm->m_pRoadmap->find_vertex(kp[j].second).property(),
			      &lpOutput,
			      connectionPosRes, connectionOriRes, 
			      (!addAllEdges) )) {
	    _rm->m_pRoadmap->AddEdge(kp[j].first, kp[j].second, lpOutput.edge);
	    Stats.IncConnections_Made();
      }
    } //endfor j
	//}//endfor z
	// }









}















#endif
