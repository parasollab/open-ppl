#ifndef ClosestSF_h
#define ClosestSF_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "RoadmapGraph.h"

//Attempt K+M Closest neighbors and only allowed K success OR M failures
//If M is not specified in command line, it is set as same as K
/**Connect nodes in map to their k+m closest neighbors.
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

#define KSUCCESS 5 
#define MFAILURE 5 

template <class CFG, class WEIGHT>
class ClosestSF: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  //////////////////////
  // Constructors and Destructor
  ClosestSF();
  ClosestSF(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ClosestSF(int k, int m);
  virtual ~ClosestSF();
 
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

  int ksuccess;
  int mfailure;
  string m_nf;
};


template <class CFG, class WEIGHT>
ClosestSF<CFG,WEIGHT>::ClosestSF():NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closestsf"; 
  SetDefault();
}

template <class CFG, class WEIGHT>
ClosestSF<CFG,WEIGHT>::ClosestSF(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem) { 
  LOG_DEBUG_MSG("ClosestSF::ClosestSF()"); 
  this->element_name = "closestsf"; 
  SetDefault();
  ParseXML(in_Node);
  
  
  LOG_DEBUG_MSG("~ClosestSF::ClosestSF()"); 
}


template <class CFG, class WEIGHT>
ClosestSF<CFG,WEIGHT>::ClosestSF(int k, int m):NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closestsf"; 
  ksuccess = k;
  mfailure = m;
}


template <class CFG, class WEIGHT>
ClosestSF<CFG,WEIGHT>::~ClosestSF() { 
}


template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::ParseXML(XMLNodeReader& in_Node) { 
  NodeConnectionMethod<CFG,WEIGHT>::ParseXML(in_Node); 
  ksuccess = in_Node.numberXMLParameter(string("success"), true, 0,1,1000, 
                                  string("k-success value")); 

  mfailure = in_Node.numberXMLParameter(string("fail"), true, 0,1,1000, 
                                  string("k-fail value"));

}



template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::SetDefault() {
  ksuccess = KSUCCESS;
  mfailure = MFAILURE;
}


template <class CFG, class WEIGHT>
void
ClosestSF<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << this->GetName() << " ";
  _os << "\tINTEGER INTEGER (default ksuccess:" << KSUCCESS << ", mfailure:" << MFAILURE << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
ClosestSF<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << this->GetName() << " ksuccess = ";
  _os << ksuccess << " mfailure = " << mfailure ;
  _os << endl;
}

template <class CFG, class WEIGHT>
void
ClosestSF<CFG, WEIGHT>::
PrintOptions(ostream& out_os){
  NodeConnectionMethod<CFG,WEIGHT>::PrintOptions(out_os);
  out_os << "      ksuccess = " << ksuccess << endl;
  out_os << "      mfailure = " << mfailure << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
ClosestSF<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new ClosestSF<CFG,WEIGHT>(*this);
  return _copy;
}

template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::
Connect() {
  //cout << "Connecting CCs with method: closest k="<< ksuccess << endl ;
  //cout << "DOING NOTHING" << endl;
}


template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  //cout << "Connecting CCs with method: closest k="<< ksuccess << endl;
#ifndef QUIET
  cout << "closest(k="<< ksuccess <<"): "<<flush;
#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<VID> vertices;
  pMap->GetVerticesVID(vertices);
  
  Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges, vertices, vertices);

}


template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::
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
 *   find (kclosest+mfailure) nodes in v2
 *   attempt connection
 * }
 */
template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<VID>& v1, vector<VID>& v2) 
{
  cout << "Connecting CCs with method: closest k="<< ksuccess << endl;
#ifndef QUIET
  cout << "closest*(k="<< ksuccess <<"): "<<flush;
  cout << "failure*(m="<< mfailure <<"): "<<flush;

#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;  
  vector< pair<VID,VID> > kp;

  if( v2.size() < ksuccess+mfailure) {//all pairs
    for(typename vector<VID>::iterator I = v1.begin(); I != v1.end(); ++I){
      kp.clear();
      for(typename vector<VID>::iterator J = v2.begin(); J != v2.end(); ++J){
        if(*I != *J)
          kp.push_back(make_pair<VID,VID>(*I, *J));
      }
      Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges, kp);
    } //end of for I
  } else {
    for(typename vector<VID>::iterator I = v1.begin(); I != v1.end(); ++I){
      kp.clear();
      vector<VID> v;
      v.push_back(*I);
      
      kp = this->GetMPProblem()->GetNeighborhoodFinder()->FindKClosestPairs(_rm,v, v2, ksuccess+mfailure);
      
      Connect(_rm, Stats, dm, lp, addPartialEdge, addAllEdges, kp);
    }
  }
}


//connection for a given set of pairs and only allow mfailure 
template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector< pair<VID,VID> > kp) 
{
    int success = 0;   //actual success attemp for this node
    int failure = 0;   //actual failure attemp for this node
    
    // for each pair identified
    LPOutput<CFG,WEIGHT> lpOutput;
    stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
    for(typename vector<pair<VID,VID> >::iterator KP = kp.begin(); KP != kp.end(); ++KP) {
      cout<<KP->first<<"->"<<KP->second<<endl;
      if(failure >= mfailure || success >= ksuccess){
      cout << "mfailure/ksuccss equals the max allowed number."<<KP->first<<" fails: "<<failure<<", success: "<<success<<endl;
      break;
      }
      if(_rm->m_pRoadmap->IsEdge(KP->first, KP->second)) {
        success++;
        continue;
      }
      cmap.reset();
      if(this->m_CheckIfSameCC && is_same_cc(*(_rm->m_pRoadmap), cmap, KP->first, KP->second)) {
        success++;
        continue;
      }
      if(lp->IsConnected(_rm->GetEnvironment(), Stats, dm,
               _rm->m_pRoadmap->find_vertex(KP->first).property(),
               _rm->m_pRoadmap->find_vertex(KP->second).property(),
               &lpOutput, this->connectionPosRes, this->connectionOriRes, 
               (!addAllEdges) )) {
        _rm->m_pRoadmap->AddEdge(KP->first, KP->second, lpOutput.edge);
	success++;
	
      }
      else
      failure++;
   } 
}


#endif
