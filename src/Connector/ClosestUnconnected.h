#ifndef ClosestUnconnected_h
#define ClosestUnconnected_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "GraphAlgo.h"

//Connect K ClosestUnconnected only allowed M failures
//If M is not specified in command line, it is set as same as K
/**Connect nodes in map to their k closest neighbors.
 *Following Algorithm is used:
 *   -# for evry node, cfg1, in roadmap
 *       -# find k closet neighbors for cfg1
 *       -# lp_set is a local planner set defined in info.lpsetid
 *       -# for every node, cfg2, in k-closest neighbor list for cfg1
 *           -# using local planning functions in lp_set
 *              to connect cfg1 and cfg2
 *           -# if connected, add this edge to map, _rm.
 *       -#end for
 *   -# end for
 *
 *@param info provides inforamtion other than connection, like
 *collision dection, local planner, and distance metrics.
 *@param _cn provides information for specific node connection 
 *paramters.
 *@param lp Local planner for connecting given 2 Cfgs.
 *
 *@see RoadmapGraph::AddEdge and LocalPlanners::IsConnected
 */

#define KCLOSEST 5 
#define MFAILURE 5 

template <class CFG, class WEIGHT>
class ClosestUnconnected: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  ClosestUnconnected();
  ClosestUnconnected(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ClosestUnconnected(int k);
  ClosestUnconnected(int k, int m);
  virtual ~ClosestUnconnected();
 
  //////////////////////
  // Access
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);  
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os);  
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();
  virtual void ParseXML(XMLNodeReader& in_Node);

  //////////////////////
  // Core: Connection method
  void Connect();

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
             CollisionDetection*, DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges);  

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
             CollisionDetection*, DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<VID>& v1, vector<VID>& v2);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
             CollisionDetection*, DistanceMetric*,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<vector<VID> >& verticesList);

  //function used in this class only
  void Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector< pair<VID,VID> > kp); 
  void Connect2(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<pair<pair<VID,VID>,double> > kp); 


  //private:
  //////////////////////
  // Data

  int kclosest;
  int mfailure;
};


template <class CFG, class WEIGHT>
ClosestUnconnected<CFG,WEIGHT>::ClosestUnconnected():NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closest_unconnected"; 
  SetDefault();
}

template <class CFG, class WEIGHT>
ClosestUnconnected<CFG,WEIGHT>::ClosestUnconnected(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem) { 
  LOG_DEBUG_MSG("ClosestUnconnected::ClosestUnconnected()"); 
  this->element_name = "closest_unconnected"; 
  SetDefault();
  ParseXML(in_Node);
  
  
  LOG_DEBUG_MSG("~ClosestUnconnected::ClosestUnconnected()"); 
}


//this is backward support for function call from other class
//to be cleaned
template <class CFG, class WEIGHT>
ClosestUnconnected<CFG,WEIGHT>::ClosestUnconnected(int k):NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closest_unconnected"; 
  kclosest = k;
  mfailure = k;
}

template <class CFG, class WEIGHT>
ClosestUnconnected<CFG,WEIGHT>::ClosestUnconnected(int k, int m):NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closest_unconnected"; 
  kclosest = k;
  mfailure = m;
}


template <class CFG, class WEIGHT>
ClosestUnconnected<CFG,WEIGHT>::~ClosestUnconnected() { 
}


template <class CFG, class WEIGHT>
void ClosestUnconnected<CFG,WEIGHT>::ParseXML(XMLNodeReader& in_Node) { 
  NodeConnectionMethod<CFG,WEIGHT>::ParseXML(in_Node); 
  kclosest = in_Node.numberXMLParameter(string("k"), true, 5,1,1000, 
                                  string("k-closest value"));
}



template <class CFG, class WEIGHT>
void ClosestUnconnected<CFG,WEIGHT>::SetDefault() {
  kclosest = KCLOSEST;
  mfailure = MFAILURE;
}


template <class CFG, class WEIGHT>
void
ClosestUnconnected<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  

  _os << "\n" << this->GetName() << " ";
  _os << "\tINTEGER INTEGER (default kclosest:" << KCLOSEST << ", mfailure:" << MFAILURE << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ClosestUnconnected<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << this->GetName() << " kclosest = ";
  _os << kclosest << " mfailure = " << mfailure ;
  _os << endl;
}

template <class CFG, class WEIGHT>
void
ClosestUnconnected<CFG, WEIGHT>::
PrintOptions(ostream& out_os){
  NodeConnectionMethod<CFG,WEIGHT>::PrintOptions(out_os);
  out_os << "      kclosest = " << kclosest << endl;
  out_os << "      mfailure = " << mfailure << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
ClosestUnconnected<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new ClosestUnconnected<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void ClosestUnconnected<CFG,WEIGHT>::
Connect() {
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl ;
  //cout << "DOING NOTHING" << endl;
}


template <class CFG, class WEIGHT>
void ClosestUnconnected<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
#ifndef QUIET
  cout << "closest(k="<< kclosest <<"): "<<flush;
#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<VID> vertices;
  pMap->GetVerticesVID(vertices);
  
  Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges, vertices, vertices);

}


template <class CFG, class WEIGHT>
void ClosestUnconnected<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<vector<VID> >& verticesList)
{
  for(int i=0; i<verticesList.size()-1; ++i)
    for(int j=i+1; j<verticesList.size(); ++j)
      Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges,
            verticesList[i], verticesList[j]);
}


/*
 * for each node in v1 {
 *   find k closest nodes in v2
 *   attempt connection
 * }
 */
template <class CFG, class WEIGHT>
void ClosestUnconnected<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<VID>& v1, vector<VID>& v2) 
{
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
#ifndef QUIET
  cout << "closestU*(k="<< kclosest <<"): "<<flush;
  cout << " failure*(m="<< mfailure <<"): "<<flush;
#endif
  
  if( v2.size() < kclosest) {//all pairs
    for(vector<VID>::iterator I = v1.begin(); I != v1.end(); ++I){
      vector<pair<VID,VID> > kp;
      for(vector<VID>::iterator J = v2.begin(); J != v2.end(); ++J) {
        if(*I != *J)
          kp.push_back(make_pair<VID,VID>(*I, *J));
      }
      Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges, kp);
    } 
  } else {
    RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;  
    Environment* env = _rm->GetEnvironment();
    for(vector<VID>::iterator I = v1.begin(); I != v1.end(); ++I) {
      vector<pair<pair<VID,VID>,double> > kpd;
      CFG Icfg = pMap->GetData(*I);
      for(vector<VID>::iterator J = v2.begin(); J != v2.end(); ++J) 
        if(*I != *J)
          kpd.push_back(make_pair(make_pair(*I, *J),
                        dm->Distance(env, Icfg, pMap->GetData(*J))));
      sort(kpd.begin(), kpd.end(), DIST_Compare<VID>());
      Connect2(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges, kpd);
    }
  }
}



//connection for a given set of pairs and only allow mfailure 
template <class CFG, class WEIGHT>
void ClosestUnconnected<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<pair<VID,VID> > kp) 
{
    int failure = 0;   //actual failure attemp for this node 
    // for each pair identified
    LPOutput<CFG,WEIGHT> lpOutput;
    for(vector<pair<VID,VID> >::iterator KP = kp.begin(); KP != kp.end(); ++KP) {
      if(failure >= mfailure){
        break;
      }

      if(_rm->IsCached(KP->first,KP->second)) {
	if(!_rm->GetCache(KP->first,KP->second)) {
	  continue;
	}
      }
      if(_rm->m_pRoadmap->IsEdge(KP->first, KP->second)) 
        continue;

      if(this->m_CheckIfSameCC && IsSameCC(*(_rm->m_pRoadmap), KP->first, KP->second)) 
        continue;

      if(lp->IsConnected(_rm->GetEnvironment(), Stats, cd, dm,
			 _rm->m_pRoadmap->GetData(KP->first),
			 _rm->m_pRoadmap->GetData(KP->second),
			 &lpOutput, this->connectionPosRes, this->connectionOriRes, 
			 (!addAllEdges) )) {
        _rm->m_pRoadmap->AddEdge(KP->first, KP->second, lpOutput.edge);
	Stats.IncConnections_Made();
	_rm->SetCache(KP->first,KP->second,true);
      } else {
	_rm->SetCache(KP->first,KP->second,false);
	failure++;
      }
    } 
}


//connection for a given set of pairs and only allow mfailure 
template <class CFG, class WEIGHT>
void ClosestUnconnected<CFG,WEIGHT>::
Connect2(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<pair<pair<VID,VID>,double> > kp) 
{
    int failure = 0;   //actual failure attemp for this node 
    int attempt = 0;

    // for each pair identified
    LPOutput<CFG,WEIGHT> lpOutput;
    for(vector<pair<pair<VID,VID>,double> >::iterator KP = kp.begin(); KP != kp.end(); ++KP) {
      if(failure >= mfailure) {
        break;
      }

      if(attempt >= kclosest) {
        break;
      }

      if(_rm->IsCached(KP->first.first, KP->first.second) && !_rm->GetCache(KP->first.first, KP->first.second)) {
        continue;
      }

      if(_rm->m_pRoadmap->IsEdge(KP->first.first, KP->first.second) ||
         IsSameCC(*(_rm->m_pRoadmap), KP->first.first, KP->first.second)) {
        continue;
      }

      ++attempt;
      if(lp->IsConnected(_rm->GetEnvironment(), Stats, cd, dm,
			 _rm->m_pRoadmap->GetData(KP->first.first),
			 _rm->m_pRoadmap->GetData(KP->first.second),
			 &lpOutput, this->connectionPosRes, this->connectionOriRes, 
			 (!addAllEdges) )) {
        _rm->m_pRoadmap->AddEdge(KP->first.first, KP->first.second, lpOutput.edge);
	Stats.IncConnections_Made();
	_rm->SetCache(KP->first.first,KP->first.second,true);
      } else {
	_rm->SetCache(KP->first.first,KP->first.second,false);
	++failure;
      }
    } 
}



#endif
