#ifndef ClosestSF_h
#define ClosestSF_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "GraphAlgo.h"

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
  //////////////////////
  // Constructors and Destructor
  ClosestSF();
  ClosestSF(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  ClosestSF(int k, int m);
  virtual ~ClosestSF();
 
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
  virtual NodeConnectionMethod<CFG, WEIGHT>* CreateCopy();
  virtual void ParseXML(TiXmlNode* in_pNode);

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


 private:
  //////////////////////
  // Data

  int ksuccess;
  int mfailure;
};


template <class CFG, class WEIGHT>
ClosestSF<CFG,WEIGHT>::ClosestSF():NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "closestsf"; 
  SetDefault();
}

template <class CFG, class WEIGHT>
ClosestSF<CFG,WEIGHT>::ClosestSF(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_pNode, in_pProblem) { 
  LOG_DEBUG_MSG("ClosestSF::ClosestSF()"); 
  this->element_name = "closestsf"; 
  SetDefault();
  ParseXML(in_pNode);
  
  
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
void ClosestSF<CFG,WEIGHT>::ParseXML(TiXmlNode* in_pNode) { 
  
  int k;
  if(TIXML_SUCCESS == in_pNode->ToElement()->QueryIntAttribute("success",&k))
  {
    ksuccess = k;
  }
  int fail;		
  if(TIXML_SUCCESS == in_pNode->ToElement()->QueryIntAttribute("fail",&fail))
  {
    mfailure = fail;
  }
 
}

//If there are two parameters in the command line, 
//  the first one is K, the second one is M;
//If there is only one parameter, 
//  we set M equal to K (this is identical to K-closest connection method)
template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::
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
      if (is >> ksuccess) {
        if (ksuccess < 0)
  	  throw BadUsage();

        c = is.peek();
        while(c == ' ' || c == '\n') {
          is.get();
          c = is.peek();
        }
        if (c >= '0' && c <='9') {
          if (is >> mfailure) {
    	    if (mfailure < 0)
	      throw BadUsage();
          } else
	      throw BadUsage();
        } else 
          mfailure = ksuccess;  
	  //set mfailure equals to kcloest if it is not specified in the command line

      } else
        throw BadUsage();
    }

  } catch (BadUsage) {
    cerr << "Error in \'closest\' parameters" << endl;
    PrintUsage(cerr);
    exit(-1);
  }

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
  out_os << "    " << this->GetName() << "::  ksuccess = ";
  out_os << ksuccess << "  mfailure = " << mfailure ;
  out_os << endl;
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
            CollisionDetection* cd , 
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
  
  Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges, vertices, vertices);

}


template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::
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
 *   find (kclosest+mfailure) nodes in v2
 *   attempt connection
 * }
 */
template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            vector<VID>& v1, vector<VID>& v2) 
{
  //cout << "Connecting CCs with method: closest k="<< ksuccess << endl;
#ifndef QUIET
  cout << "closest*(k="<< ksuccess <<"): "<<flush;
  cout << "failure*(m="<< mfailure <<"): "<<flush;

#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;  
  vector< pair<VID,VID> > kp;

  if( v2.size() < ksuccess+mfailure) {//all pairs
    for(vector<VID>::iterator I = v1.begin(); I != v1.end(); ++I){
      kp.clear();
      for(vector<VID>::iterator J = v2.begin(); J != v2.end(); ++J){
        if(*I != *J)
          kp.push_back(make_pair<VID,VID>(*I, *J));
      }
      Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges, kp);
    } //end of for I
  } else {
    for(vector<VID>::iterator I = v1.begin(); I != v1.end(); ++I){
      kp.clear();
      vector<VID> v;
      v.push_back(*I);
      kp = dm->FindKClosestPairs(_rm,v, v2, ksuccess+mfailure);
      Connect(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges, kp);
    }
  }
}


//connection for a given set of pairs and only allow mfailure 
template <class CFG, class WEIGHT>
void ClosestSF<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            CollisionDetection* cd , 
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
    for(vector<pair<VID,VID> >::iterator KP = kp.begin(); KP != kp.end(); ++KP) {
      //cout<<KP->first<<"->"<<KP->second<<endl;
      if(failure >= mfailure || success >= ksuccess){
      //cout << "mfailure/ksuccss equals the max allowed number."<<KP->first<<" fails: "<<failure<<", success: "<<success<<endl;
      break;
      }
      if(_rm->m_pRoadmap->IsEdge(KP->first, KP->second)) {
        success++;
        continue;
      }
      #if CHECKIFSAMECC
      if(IsSameCC(*(_rm->m_pRoadmap), KP->first, KP->second)) {
        success++;
        continue;
      }
      #endif
      if(lp->IsConnected(_rm->GetEnvironment(), Stats, cd, dm,
               _rm->m_pRoadmap->GetData(KP->first),
               _rm->m_pRoadmap->GetData(KP->second),
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
