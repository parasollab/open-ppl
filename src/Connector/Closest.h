#ifndef Closest_h
#define Closest_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "GraphAlgo.h"

//Connect K Closest only allowed M failures
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
class Closest: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  //////////////////////
  // Constructors and Destructor
  Closest();
  Closest(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  Closest(int k);
  Closest(int k, int m);
  virtual ~Closest();
 
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
             DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges);  

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats, 
             DistanceMetric *,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<VID>& v1, vector<VID>& v2);

  void Connect(Roadmap<CFG, WEIGHT>*, Stat_Class& Stats,
             DistanceMetric*,
             LocalPlanners<CFG,WEIGHT>*,
             bool addPartialEdge, bool addAllEdges,
             vector<vector<VID> >& verticesList);

  //function used in this class only
  void Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector< pair<VID,VID> > kp); 


 private:
  //////////////////////
  // Data

  int kclosest;
  int mfailure;
};


template <class CFG, class WEIGHT>
Closest<CFG,WEIGHT>::Closest():NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closest"; 
  SetDefault();
}

template <class CFG, class WEIGHT>
Closest<CFG,WEIGHT>::Closest(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem) { 
  LOG_DEBUG_MSG("Closest::Closest()"); 
  this->element_name = "closest"; 
  SetDefault();
  ParseXML(in_Node);
  
  
  LOG_DEBUG_MSG("~Closest::Closest()"); 
}


//this is backward support for function call from other class
//to be cleaned
template <class CFG, class WEIGHT>
Closest<CFG,WEIGHT>::Closest(int k):NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closest"; 
  kclosest = k;
  mfailure = k;
}

template <class CFG, class WEIGHT>
Closest<CFG,WEIGHT>::Closest(int k, int m):NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closest"; 
  kclosest = k;
  mfailure = m;
}


template <class CFG, class WEIGHT>
Closest<CFG,WEIGHT>::~Closest() { 
}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::ParseXML(XMLNodeReader& in_Node) { 
  NodeConnectionMethod<CFG,WEIGHT>::ParseXML(in_Node);
  kclosest = in_Node.numberXMLParameter(string("k"), true, 5,1,1000, 
                                  string("k-closest value")); 
}



template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::SetDefault() {
  kclosest = KCLOSEST;
  mfailure = MFAILURE;
}


template <class CFG, class WEIGHT>
void
Closest<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  

  _os << "\n" << this->GetName() << " ";
  _os << "\tINTEGER INTEGER (default kclosest:" << KCLOSEST << ", mfailure:" << MFAILURE << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
Closest<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << this->GetName() << " kclosest = ";
  _os << kclosest << " mfailure = " << mfailure ;
  _os << endl;
}

template <class CFG, class WEIGHT>
void
Closest<CFG, WEIGHT>::
PrintOptions(ostream& out_os){
  NodeConnectionMethod<CFG,WEIGHT>::PrintOptions(out_os);
  out_os << "      kclosest = " << kclosest << endl;
  out_os << "      mfailure = " << mfailure << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
Closest<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new Closest<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
Connect() {
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl ;
  //cout << "DOING NOTHING" << endl;
}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
#ifndef QUIET
  cout << "closest(k="<< kclosest <<", mfailure=" << mfailure <<"): "<<flush;
#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<VID> vertices;
  pMap->GetVerticesVID(vertices);
  
  Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges, vertices, vertices);

}


template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<vector<VID> >& verticesList)
{
  for(int i=0; i<verticesList.size()-1; ++i)
    for(int j=i+1; j<verticesList.size(); ++j)
      Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges,
            verticesList[i], verticesList[j]);
}


/*
 * for each node in v1 {
 *   find k closest nodes in v2
 *   attempt connection
 * }
 */
template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<VID>& v1, vector<VID>& v2) 
{
  //cout << "Connecting CCs with method: closest k="<< kclosest << std::endl;
#ifndef QUIET
  cout << "closest(k="<< kclosest <<", mfailure="<< mfailure <<"): "<<flush;

#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;  
  vector< pair<VID,VID> > kp;

  if( v2.size() < kclosest) {//all pairs
    for(vector<VID>::iterator I = v1.begin(); I != v1.end(); ++I){
      kp.clear();
      for(vector<VID>::iterator J = v2.begin(); J != v2.end(); ++J){
        if(*I != *J)
          kp.push_back(make_pair<VID,VID>(*I, *J));
      }
      Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges, kp);
    } //end of for I
  } else {
    for(vector<VID>::iterator I = v1.begin(); I != v1.end(); ++I){
      kp.clear();
      vector<VID> v;
      v.push_back(*I);
      kp = dm->FindKClosestPairs(_rm,v, v2, kclosest);
      Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges, kp);
    }
  }
}



//connection for a given set of pairs and only allow mfailure 
template <class CFG, class WEIGHT>
void Closest<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector< pair<VID,VID> > kp) 
{
    int failure = 0;   //actual failure attemp for this node 
    // for each pair identified
    LPOutput<CFG,WEIGHT> lpOutput;
    for(vector<pair<VID,VID> >::iterator KP = kp.begin(); KP != kp.end(); ++KP) {
      //cout<<KP->first<<"->"<<KP->second<<endl;
      if(failure >= mfailure){
      //cout << "failure time equals the max allowed number."<<KP->first<<" fails: "<<failure<<endl;
      break;
      }
      if(_rm->IsCached(KP->first,KP->second)) {
	if(!_rm->GetCache(KP->first,KP->second)) {
	  //cout << "Connect:: using failed cache" << endl;
	  continue;
	}
      }
      if(_rm->m_pRoadmap->IsEdge(KP->first, KP->second)) 
        continue;
      if(this->m_CheckIfSameCC && IsSameCC(*(_rm->m_pRoadmap), KP->first, KP->second)) 
        continue;
      if(lp->IsConnected(_rm->GetEnvironment(), Stats, dm,
			 _rm->m_pRoadmap->GetData(KP->first),
			 _rm->m_pRoadmap->GetData(KP->second),
			 &lpOutput, this->connectionPosRes, this->connectionOriRes, 
			 (!addAllEdges) )) {
        _rm->m_pRoadmap->AddEdge(KP->first, KP->second, lpOutput.edge);
	Stats.IncConnections_Made();
	_rm->SetCache(KP->first,KP->second,true);
      }
      else {
	_rm->SetCache(KP->first,KP->second,false);
	failure++;
      }
    } 
}



#endif
