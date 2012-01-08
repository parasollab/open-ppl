#ifndef ObstBased_h
#define ObstBased_h
#include "NodeConnectionMethod.h"


   /**Connect nodes depends on Obstacle information.
    *Obstacle to Obstacle connections are attempted for 
    *the "k" closest nodes.
    *Each "body" of a multibody is considered an obstacle.
    *Obstacles have id's. The k-closest cfg's, one of which 
    *was generated wrt the "first" obstacle (body_i) and 
    *the other of which was generated wrt the "second" (body_j), 
    *will have a connection attempted between them.
    *This continues for all unique id pairings.
    *For example:
    *Given obstacle id's 2,3,4 & 5,
    *connection is attempted for the k-closest in each obst-obst
    *(i,j) pairing below:
    *   - 2-2  2-3  2-4  2-5
    *   - 3-3  3-4  3-5
    *   - 4-4  4-5
    *   - 5-5
    *When i=j, the "k" value may be different than otherwise
    *
    *Following Algorithm is used:
    *   -# classify all nodes in roadmap by their obstacle id
    *      (obstacle id is assignment in node generation time
    *       to Cfg::obst in every Cfg)
    *   -# for each obstacle id, i
    *       -# for each obstacle id, j ,here j>=i
    *       -# find k closest pairs between Cfgs haveing obs_id=i
    *          and Cfgs haveing obs_id=j
    *       -# lp_set is a local planner set defined in info.lpsetid
    *       -# for every pair, i
    *           -# using local planning functions in lp_set
    *              to connect pair.first and pair.second
    *              (pair.first has obs_id=i and pair.second has obs_id=j)
    *           -# if connected, add this edge to map, _rm.
    *       -#end for
    *   -# end for
    *
    *For k value used in above algorithm:
    *if i!=j, then k=CN::GetKOther. Otherwise, k=GetKSelf.
    *
    *@param info provides inforamtion other than connection, like
    *collision dection, local planner, and distance metrics.
    *@param _cn provides information for specific node connection 
    *paramters.
    *@param lp Local planner for connecting given 2 Cfgs.
    *
    *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected,
    *and Get_Cfgs_By_Obst for classifing Cfgs.
    */


#define K_OTHER      10        // default for obst-other connections
#define K_SELF        3        // default for obst-self  connections


template <class CFG, class WEIGHT>
class ObstBased: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  ObstBased();
  ~ObstBased();
 
  //////////////////////
  // Access
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os); 
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();
  //////////////////////
  // Core: Connection method
  /**Compare Cfg::obst between two given Cfgs.
   *@return (_cc1.obst < _cc2.obst )
   *@note this is used to "sort" Cfg's by obst generation number
   *@see Get_Cfgs_By_Obst
   */
  //static bool info_Compare(const CFG _cc1, const CFG _cc2);
  /**Classify Cfgs by from which obstacles these Cfg are generated.
   *Vertices are stored as generated but they may have been originally
   *generated with respect to some obstacle. This proceedure
   *will return a vector where every element is a vector of cfg's
   *corresponding to a unique (& valid) id value.
   *
   *@return Return a 2D "array", each row contains Cfgs generated 
   *"from" same obstacle.
   */

  static vector<vector<VID> > Get_Cfgs_By_Obst(Roadmap<CFG,WEIGHT>*,
					       vector<VID>& vert);

  void Connect();
  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
			 CollisionDetection*, 
			 DistanceMetric *,
			 LocalPlanners<CFG,WEIGHT>*,
			 bool addPartialEdge,
			 bool addAllEdges);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
             CollisionDetection*, DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<CFG>& v1, vector<CFG>& v2);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
             CollisionDetection*, DistanceMetric*,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<vector<CFG> >& verticesList);

  void ConnectVVectorsByKClosest(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
				 CollisionDetection* cd , 
				 DistanceMetric * dm,
				 LocalPlanners<CFG,WEIGHT>* lp,
				 bool addPartialEdge,
				 bool addAllEdges, 
				 vector< vector<VID> > &body);


 private:
  //////////////////////
  // Data

  int k_other;
  int k_self;
};


///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  ObstBased
template <class CFG, class WEIGHT>
ObstBased<CFG,WEIGHT>::ObstBased():NodeConnectionMethod<CFG,WEIGHT>() { 
  element_name = "obstBased"; 

  SetDefault();
}


template <class CFG, class WEIGHT>
ObstBased<CFG,WEIGHT>::~ObstBased() { 
}


template <class CFG, class WEIGHT>
void ObstBased<CFG,WEIGHT>::SetDefault() {
  k_other = K_OTHER;
  k_self = K_SELF;
}


template <class CFG, class WEIGHT>
void
ObstBased<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);  
  _os << "\n" << GetName() << " ";
  _os << "\tINTEGER INTEGER (default kother:" << K_OTHER << ", kself:" << K_SELF << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ObstBased<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " k_other = ";
  _os << k_other << ", k_self = " << k_self;
  _os << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
ObstBased<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new ObstBased<CFG,WEIGHT>(*this);
  return _copy;
}


//
// used to "sort" Cfg's by obst generation number
//
/*
template <class CFG, class WEIGHT>
bool
ObstBased<CFG, WEIGHT>::
info_Compare(const CFG _cc1, const CFG _cc2){
  return (_cc1.obst < _cc2.obst ) ;
}
*/

template <class CFG, class WEIGHT>
struct Info_Compare : public binary_function<VID,VID,bool> {
  RoadmapGraph<CFG,WEIGHT>* pMap;
  Info_Compare(RoadmapGraph<CFG,WEIGHT>* graph) : pMap(graph) {} 
  bool operator()(const VID v1, const VID v2) {
    CFG c1 = (*(pMap->find_vertex(v1))).property();
    CFG c2 = (*(pMap->find_vertex(v2))).property();
    return(c1.obst < c1.obst);
  }
};


template <class CFG, class WEIGHT>
vector<vector<VID> > 
ObstBased<CFG, WEIGHT>::
Get_Cfgs_By_Obst(Roadmap<CFG,WEIGHT>* rm, vector<VID>& vert) {
  //-- get all vertices
  //-- sort by info

  sort(vert.begin(), vert.end(), 
       Info_Compare<CFG,WEIGHT>(rm->m_pRoadmap));
  
  //-- declare value to return
  vector<vector<VID> > byInfo;
  
  //-- build up a separate vector for each unique & valid id
  vector<VID>  tmp;
  
  int  id, prev= -505;
  bool firstList=true;
  for (int i=0;i<vert.size();++i){
    id = rm->m_pRoadmap->(find_vertex(vert[i]).property()).obst;
    if (id != prev)
      if (prev == -1) {
	tmp.erase(tmp.begin(),tmp.end());
      } else { // valid body
	if (firstList){
	  firstList=false;
	} else {
	  byInfo.push_back(tmp);
	  tmp.erase(tmp.begin(),tmp.end());
	}
      }
    tmp.push_back(vert[i]);
    prev=id;
  }//endfor
  
  //-- last list wouldn't've been added in loop
  if (  prev != -1  &&  tmp.size()>0  )
    byInfo.push_back(tmp);
  
  //-- return vector of vectors of vertices

  return byInfo;
}


template <class CFG, class WEIGHT>
void ObstBased<CFG,WEIGHT>::
Connect() {
  cout << "obstBased(k_other="<<k_other
       << ", k_self="<<k_self<<"): "<<flush;
  cout << "DONOTHING" <<endl;
}


/*---------------------------------------------------------------
Obst to Obst connections are attempted for the "k" closest nodes.
Each "body" of a multibody is considered an obstacle.  Obstacles
have id's.  The k-closest cfg's, one of which was generated wrt the
"first" obstacle (body_i) and the other of which was generated wrt
the "second" (body_j), will have a connection attempted between them.
This continues for all unique id pairings.

For example:
  Given obstacle id's 2,3,4 & 5,
  connection is attempted for the k-closest in each obst-obst
  (i,j) pairing below:
        2-2  2-3  2-4  2-5
             3-3  3-4  3-5
                  4-4  4-5
                       5-5
  When i=j, the "k" value may be different than otherwise
---------------------------------------------------------------*/ 
template <class CFG, class WEIGHT>
		       void ObstBased<CFG,WEIGHT>::
		       Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
			       CollisionDetection* cd , 
			       DistanceMetric * dm,
			       LocalPlanners<CFG,WEIGHT>* lp,
			       bool addPartialEdge,
			       bool addAllEdges) {


  //-- get cfg's
  vector<VID> vert; 
  _rm->m_pRoadmap->GetVerticesVID(vert); 
  vector<vector<VID> > body = Get_Cfgs_By_Obst(_rm, vert);

  ConnectVVectorsByKClosest( _rm, Stats, cd , dm, lp, 
			     addPartialEdge,addAllEdges, body);

}

template <class CFG, class WEIGHT>
void ObstBased<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<vector<CFG> >& verticesList)
{
  vector<CFG> vertCfgs;
  
  for(int i=0; i<verticesList.size(); ++i){
    copy (verticesList[i].begin(), verticesList[i].end(), back_inserter(vertCfgs));
  }

  vector<VID> vert;
  for(typename vector<CFG>::iterator C = vertCfgs.begin(); C != vertCfgs.end(); ++C)
    vert.push_back(_rm->m_pRoadmap->GetVID(*C));

  vector<vector<VID> > body = Get_Cfgs_By_Obst(_rm, vert);
  ConnectVVectorsByKClosest( _rm, Stats, cd , dm, lp, 
			     addPartialEdge,addAllEdges, body);

}


template <class CFG, class WEIGHT>
void ObstBased<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<CFG>& v1, vector<CFG>& v2) 
{
  copy (v2.begin(), v2.end(), back_inserter(v1));

  vector<VID> vert;
  for(typename vector<CFG>::iterator C = v1.begin(); C != v1.end(); ++C)
    vert.push_back(_rm->m_pRoadmap->GetVID(*C));

  vector< vector<VID> > body = Get_Cfgs_By_Obst(_rm, vert);

  ConnectVVectorsByKClosest(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges,body);
}

template <class CFG, class WEIGHT>
void ObstBased<CFG,WEIGHT>::
ConnectVVectorsByKClosest(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<vector<VID> > &body) 
{

  if (body.empty()) return;

  for(int i=0; i< body.size()-1; ++i)
    for(int j=i; j<body.size(); ++j){
      int k;
      if (i == j) 
	k = (int)min(k_self, body[i].size());
      else      
	k = (int)min(k_other,min(body[i].size(), body[j].size()));
  
      // if no pairs to find, continue to next set of bodies
      if (k==0) 
	return;

      LPOutput<CFG,WEIGHT> lpOutput;
      stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
      // get closest pairs of nodes on obst to (possibly) another obstacle
//Old Interface
//      vector<pair<VID,VID> > kp = dm->FindKClosestPairs(_rm, body[i], 
//							body[j], k);
//New Interface
vector<pair<VID,VID> > kp(k);
 vector<pair<VID,VID> >::iterator kp_iter = kp.begin();
 this->GetMPProblem()->GetNeighborhoodFinder()->KClosestPairs(_rm, body[i].begin(), body[i].end(), body[j].begin(),body[j].end(), kp_iter);

							
							
//-- check connections between pairs
      for (int m=0;m<kp.size();++m){
	cmap.reset();
	if(this->m_CheckIfSameCC && is_same_cc(*(_rm->m_pRoadmap), cmap, kp[m].first,kp[m].second)) continue;
	if (!_rm->m_pRoadmap->IsEdge(kp[m].first,kp[m].second)
	    && lp->IsConnected(_rm->GetEnvironment(),Stats,cd,dm,
			       (*(_rm->m_pRoadmap->find_vertex(kp[m].first))).property(),
			       (*(_rm->m_pRoadmap->find_vertex(kp[m].second))).property(),
			       &lpOutput,connectionPosRes, connectionOriRes, 
			       (!addAllEdges))){
	  _rm->m_pRoadmap->AddEdge(kp[m].first,kp[m].second,lpOutput.edge);
	}//endif IsConnected
      }//endfor m
      
    }
}
#endif
