#ifndef Closest_h
#define Closest_h

#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "GraphAlgo.h"
#include "NeighborhoodFinder.h"

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
  typedef typename RoadmapGraph<CFG, WEIGHT>::vertex_descriptor VID;
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

  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges) ;

  template<typename InputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last) ;

  template<typename InputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last,
        InputIterator _itr2_first, InputIterator _itr2_last) ;



 private:
  //////////////////////
  // Data

  int kclosest;
  int mfailure;
  string m_nf;
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
  m_nf = in_Node.stringXMLParameter("nf", true, "", "nf");
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
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
    cout << "Closest<CFG,WEIGHT>::ConnectNodes() - Roadmap Only" << endl << flush;
  //cout << "Connecting CCs with method: closest k="<< kclosest << endl;
#ifndef QUIET
  cout << "closest(k="<< kclosest <<", mfailure=" << mfailure <<"): "<<flush;
#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<VID> vertices;
  pMap->GetVerticesVID(vertices);
  
  ConnectNodes(_rm, Stats, dm, lp, addPartialEdge, addAllEdges, 
        vertices.begin(), vertices.end());
}


template <class CFG, class WEIGHT>
template<typename InputIterator>
void Closest<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            InputIterator _itr1_first, InputIterator _itr1_last) 
{
  cout << "Closest<CFG,WEIGHT>::ConnectNodes() - 1 pairs InputIterator" << endl << flush;


  LPOutput<CFG,WEIGHT> lpOutput;
  for(InputIterator itr1 = _itr1_first; itr1 != _itr1_last; ++itr1) {
    CFG v_cfg = _rm->m_pRoadmap->GetData(*itr1);
    vector<VID> closest(kclosest);
    typename vector<VID>::iterator closest_iter = closest.begin();
    NeighborhoodFinder::NeighborhoodFinderPointer nfptr;
    nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf);
    this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, _rm, v_cfg, kclosest, closest_iter);
    for(typename vector<VID>::iterator itr2 = closest.begin(); itr2!= closest.end(); ++itr2) {
      if(*itr1 == *itr2) continue; //don't connect the same ones!
      if(_rm->IsCached(*itr1,*itr2)) {
        if(!_rm->GetCache(*itr1,*itr2)) {
        //cout << "Connect:: using failed cache" << endl;
        continue;
        }
      }
      Stats.IncConnections_Attempted();
      if(_rm->m_pRoadmap->IsEdge(*itr1,*itr2)) {
        Stats.IncConnections_Made();
        continue;
      }
      #if CHECKIFSAMECC
      if(IsSameCC(*(_rm->m_pRoadmap), *itr1,*itr2)) {
        Stats.IncConnections_Made();
        continue;
      }
      #endif
      if(lp->IsConnected(_rm->GetEnvironment(), Stats, dm,
       _rm->m_pRoadmap->GetData(*itr1),
       _rm->m_pRoadmap->GetData(*itr2),
       &lpOutput, this->connectionPosRes, this->connectionOriRes, 
       (!addAllEdges) )) {
        _rm->m_pRoadmap->AddEdge(*itr1,*itr2, lpOutput.edge);
        Stats.IncConnections_Made();
        _rm->SetCache(*itr1,*itr2,true);
      }
      else {
        _rm->SetCache(*itr1,*itr2,false);
      }
    } 
  }

}


/*
 * for each node in v1 {
 *   find k closest nodes in v2
 *   attempt connection
 * }
 */
template <class CFG, class WEIGHT>
template<typename InputIterator>
void Closest<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            InputIterator _itr1_first, InputIterator _itr1_last,
            InputIterator _itr2_first, InputIterator _itr2_last)
{
  cout << "Closest<CFG,WEIGHT>::ConnectNodes() - 2 pairs InputIterator" << endl << flush;

  LPOutput<CFG,WEIGHT> lpOutput;
  for(InputIterator itr1 = _itr1_first; itr1 != _itr1_last; ++itr1) {
    CFG v_cfg = _rm->m_pRoadmap->GetData(*itr1);
    vector<VID> closest(kclosest);
    typename vector<VID>::iterator closest_iter = closest.begin();
    NeighborhoodFinder::NeighborhoodFinderPointer nfptr;
    nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf);
    this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, _rm, _itr2_first, 
                                               _itr2_last, v_cfg, kclosest, closest_iter);
    for(typename vector<VID>::iterator itr2 = closest.begin(); itr2!= closest.end(); ++itr2) {
      if(*itr1 == *itr2) continue; //don't connect the same ones!
      if(_rm->IsCached(*itr1,*itr2)) {
        if(!_rm->GetCache(*itr1,*itr2)) {
        //cout << "Connect:: using failed cache" << endl;
        continue;
        }
      }

      Stats.IncConnections_Attempted();
      if(_rm->m_pRoadmap->IsEdge(*itr1,*itr2)) {
        Stats.IncConnections_Made();
        continue;
      }
      #if CHECKIFSAMECC
      if(IsSameCC(*(_rm->m_pRoadmap), *itr1,*itr2)) {
        Stats.IncConnections_Made();
        continue;
      }
      #endif
      if(lp->IsConnected(_rm->GetEnvironment(), Stats, dm,
       _rm->m_pRoadmap->GetData(*itr1),
       _rm->m_pRoadmap->GetData(*itr2),
       &lpOutput, this->connectionPosRes, this->connectionOriRes, 
       (!addAllEdges) )) {
        _rm->m_pRoadmap->AddEdge(*itr1,*itr2, lpOutput.edge);
        Stats.IncConnections_Made();
        _rm->SetCache(*itr1,*itr2,true);
      }
      else {
        _rm->SetCache(*itr1,*itr2,false);
      }
    } 
  }
}
 
#endif
