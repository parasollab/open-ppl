#ifndef NeighborhoodConnection_h
#define NeighborhoodConnection_h

#include "ConnectionMethod.h"

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
class NeighborhoodConnection: public ConnectionMethod<CFG,WEIGHT> {
  public:
    //////////////////////
    // Typedef from RoadmapGraph
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    typedef typename vector<typename RoadmapGraph<CFG,WEIGHT>::VID>::iterator VIDIT;

    //////////////////////
    // Constructors and Destructor
    NeighborhoodConnection(string _lp = "", string _nf = "", 
        int _k = KCLOSEST, int _m = MFAILURE, 
        bool _countFailures = false, bool _unconnected = false, 
        bool _random = false, bool _checkIfSameCC = false,
        MPProblem* _problem = NULL);
    NeighborhoodConnection(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~NeighborhoodConnection();

    //////////////////////
    // Used in new MPProblem framework.
    virtual void PrintOptions(ostream& _os);  
    virtual void ParseXML(XMLNodeReader& _node);

    //////////////////////
    // Core: Connection method
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
          ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
          InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision) ;

  protected:
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void ConnectNeighbors(
          Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, 
          ColorMap& _cmap, VID _vid,
          InputIterator _closestFirst, InputIterator _closestLast,
          OutputIterator _collision);

    //////////////////////
    // Utility Method
    template <typename InputIterator, typename OutputIterator>
      OutputIterator FindKNeighbors(
          Roadmap<CFG, WEIGHT>* _rm, CFG _cfg, 
          InputIterator _itrFirst, InputIterator _itrLast, 
          int _k,
          const vector<VID>& _iterNeighbors, 
          OutputIterator _closestIter);

  private:
    //////////////////////
    // Data
    int m_totalSuccess;
    int m_totalFailure;
    int m_iterSuccess;
    int m_iterFailure;
    int m_k;
    int m_fail;
    bool m_countFailures;
    bool m_unconnected;
    bool m_random;
    bool m_checkIfSameCC;
};

///////////////////////////////////////////////////////////////////////////////
  template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::NeighborhoodConnection(string _lp, string _nf, int _k, int _m, bool _countFailures, bool
    _unconnected, bool _random, bool _checkIfSameCC, MPProblem* _problem) 
  : ConnectionMethod<CFG,WEIGHT>(), m_k(_k), m_fail(_m), 
  m_countFailures(_countFailures), m_unconnected(_unconnected),
  m_random(_random), m_checkIfSameCC(_checkIfSameCC){
    this->SetName("NeighborhoodConnection"); 
    this->m_lpMethod = _lp;
    this->m_nfMethod = _nf;
    this->SetMPProblem(_problem);
  }

///////////////////////////////////////////////////////////////////////////////
  template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::NeighborhoodConnection(XMLNodeReader& _node, MPProblem* _problem) 
  : ConnectionMethod<CFG,WEIGHT>(_node, _problem), 
  m_k(KCLOSEST), m_fail(MFAILURE), m_countFailures(false), m_unconnected(false), m_random(false){
    ParseXML(_node);
  }

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::~NeighborhoodConnection(){ 
}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void NeighborhoodConnection<CFG,WEIGHT>::ParseXML(XMLNodeReader& _node){
  this->SetName("NeighborhoodConnection"); 
  m_checkIfSameCC = _node.boolXMLParameter("CheckIfSameCC",false,true,"If true, do not connect if edges are in the same CC");
  m_countFailures = _node.boolXMLParameter("count_failures", false, false, "if false, ignore failure count and just attempt k; if true, attempt k neighbors until too many failures detected");
  m_k = _node.numberXMLParameter("k", true, 0, 0, 10000, "k-value (max neighbors to find). k = 0 --> all-pairs");
  m_fail = _node.numberXMLParameter("fail", false, m_k, 0, 10000, "amount of failed connections allowed before operation terminates");
  m_unconnected = _node.boolXMLParameter("unconnected", false, false, "if true, do not count existing connections towards k");
  m_random = _node.boolXMLParameter("random", false, false, "if true, find k random configurations from destination vector");
  _node.warnUnrequestedAttributes();
}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void NeighborhoodConnection<CFG, WEIGHT>::PrintOptions(ostream& _os){
  ConnectionMethod<CFG,WEIGHT>::PrintOptions(_os);
  _os << "    " << this->GetName() << "::  k = ";
  _os << m_k << "  fail = " << m_fail ;
  _os << "  count_failures = " << this->m_countFailures;
  _os << "  unconnected = " << m_unconnected;
  _os << "  random = " << m_random;
  _os << endl;
}

///////////////////////////////////////////////////////////////////////////////
// ConnectNodes
/*
 * for each node in v1 {
 *   find k closest nodes in v2
 *   attempt connection
 * }
 */
///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template<typename ColorMap, typename InputIterator, typename OutputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, 
    ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision){

  typedef RoadmapGraph<CFG, WEIGHT> RoadmapGraphType;
  typedef pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  if(this->m_debug){
    cout << endl; 
    PrintOptions(cout);
  }
  // the vertices in this iteration are the source for the connection operation

  int iterSize = _itr2Last - _itr2First;

  // calculate the number of neighbors to retrieve at each iteration
  // if m_k is 0, we will attempt to connect all pairs
  // if this->m_countFailures is true, add m_fail to attempts since they will count
  int kToFind;
  if(m_k == 0) kToFind = iterSize;
  else {
    kToFind = (this->m_countFailures) ? m_k+m_fail : m_k;
    kToFind = min(kToFind, iterSize); // cap kToFind at iterSize
  }

  m_totalSuccess = m_totalFailure = 0;
  for(InputIterator itr1 = _itr1First; itr1 != _itr1Last; ++itr1){

    // find cfg pointed to by itr1
    CFG vCfg = GetCfg()(_rm->m_pRoadmap, itr1);
    if(this->m_debug){
      cout << (itr1 - _itr1First) << "\tAttempting connections: VID = " << *itr1 << "  --> " << vCfg << endl;
    }

    bool enoughConnected = true;
    m_iterSuccess = m_iterFailure = 0;
    vector<VID> iterNeighbors;

    do{
      if(m_unconnected) kToFind = min(2*kToFind, iterSize);
      if(this->m_debug) cout << "kToFind = " << kToFind << endl;

      _stats.StartClock("kClosest");

      vector<VID> closest;
      back_insert_iterator<vector<VID> > iterBegin(closest);
      FindKNeighbors(_rm, vCfg, _itr2First, _itr2Last, kToFind, iterNeighbors, iterBegin);   

      _stats.StopClock("kClosest");

      if(this->m_debug) copy(closest.begin(), closest.end(), ostream_iterator<VID>(cout, " "));

      ConnectNeighbors(_rm, _stats, _cmap, *itr1, closest.begin(), closest.end(), _collision);
      enoughConnected = true;

      if(m_iterSuccess < m_k){
        if(this->m_countFailures) enoughConnected = !(m_iterFailure < m_fail);
        else enoughConnected = false;
      }

    } while(m_unconnected && !enoughConnected && kToFind < iterSize);
  }

  if(this->m_debug) cout << "*** kClosest Time = " << _stats.GetSeconds("kClosest") << endl;
  if(this->m_debug) cout << "*** m_totalSuccess = " << m_totalSuccess << endl;
  if(this->m_debug) cout << "*** m_totalFailure = " << m_totalFailure << endl;
}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template <typename ColorMap, typename InputIterator, typename OutputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::ConnectNeighbors(
    Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, 
    ColorMap& _cmap, VID _vid,
    InputIterator _closestFirst, InputIterator _closestLast,
    OutputIterator _collision){
  
  typedef RoadmapGraph<CFG, WEIGHT> RoadmapGraphType;
  typedef pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetMethod(this->m_nfMethod)->GetDMMethod();
  LPOutput<CFG,WEIGHT> lpOutput;
  int success(m_iterSuccess);
  int failure(m_iterFailure);

  // connect the found k-closest to the current iteration's CFG
  for(typename vector<VID>::iterator itr2 = _closestFirst; itr2 != _closestLast; ++itr2){
    if(*itr2==INVALID_VID)
      continue;
    if(this->m_debug) cout << "\t(s,f) = (" << success << "," << failure << ")";
    if(this->m_debug) cout << " | VID = " << *itr2;
    if(this->m_debug) cout << " | dist = " << 
      dm->Distance( _rm->GetEnvironment(), 
          GetCfg()(_rm->m_pRoadmap, _vid),
          GetCfg()(_rm->m_pRoadmap, itr2));

    // stopping conditions
    if(this->m_countFailures && failure >= m_fail){
      if(this->m_debug) cout << " | stopping... failures exceeded" << endl;
      break;
    }
    if(m_k > 0 && success >= m_k){
      if(this->m_debug) cout << " | stopping... successes met" << endl;
      break;
    }

    // don't attempt an edge between the same nodes
    if(_vid == *itr2){
      if(this->m_debug) cout << " | skipping... same nodes" << endl;
      continue;
    }

    // don't attempt the connection if it already failed once before
    if(_rm->IsCached(_vid,*itr2)){
      if(!_rm->GetCache(_vid,*itr2)){
        if(this->m_debug) cout << " | skipping... this connection already failed once";
        if(this->m_debug) cout << " | failure incremented";
        ++failure;
        if(this->m_debug) cout << endl;
        continue;
      }
    }

    // the edge already exists :: no need for this, it is already done in STAPL
#ifndef _PARALLEL
    if(_rm->m_pRoadmap->IsEdge(_vid, *itr2)){
      // if we're not in "unconnected" mode, count this as a success
      if(this->m_debug) cout << " | edge already exists in roadmap";
      if(!m_unconnected){
        if(this->m_debug) cout << " | success incremented";
        ++success;
      }
      if(this->m_debug) cout << endl;
      continue;
    }

    if(m_checkIfSameCC){
      // the nodes are in the same connected component
      _cmap.reset();
      if(stapl::sequential::is_same_cc(*(_rm->m_pRoadmap), _cmap, _vid, *itr2)){
        // if we're not in "unconnected" mode, count this as a success
        if(this->m_debug) cout << " | nodes in the same connected component";
        if(!m_unconnected){
          if(this->m_debug) cout << " | success incremented";
          ++success;
        }
        if(this->m_debug) cout << endl;
        continue;
      }
    }
#endif

    // attempt connection with the local planner
    CfgType col;
    if(this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpMethod)->
        IsConnected( _rm->GetEnvironment(), _stats, dm,
          GetCfg()(_rm->m_pRoadmap, _vid),
          GetCfg()(_rm->m_pRoadmap, itr2),
          col, &lpOutput, this->m_connectionPosRes, this->m_connectionOriRes, 
          (!this->m_addAllEdges) )){

      // if connection was made, add edge and record the successful connection
      if(this->m_debug) cout << " | connection was successful";
      _rm->m_pRoadmap->AddEdge(_vid, *itr2, lpOutput.edge);
      // mark the successful connection in the roadmap's cache
      if(this->m_debug) cout << " | success incremented" << endl;
      _rm->SetCache(_vid,*itr2,true);
      ++success;
      this->m_connectionAttempts.push_back(make_pair(make_pair(_vid, *itr2), true));
    }
    else {
      // mark the failed connection in the roadmap's cache
      if(this->m_debug) cout << " | connection failed | failure incremented" << endl;
      _rm->SetCache(_vid,*itr2,false);
      ++failure;
      this->m_connectionAttempts.push_back(make_pair(make_pair(_vid, *itr2), false));
    }
    if(col != CfgType())
      *_collision++ = col;
  }

  m_totalSuccess += success - m_iterSuccess;
  m_totalFailure += failure - m_iterFailure;

  m_iterSuccess = success;
  m_iterFailure = failure;
}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template <typename InputIterator, typename OutputIterator>
OutputIterator NeighborhoodConnection<CFG, WEIGHT>::
FindKNeighbors(Roadmap<CFG, WEIGHT>* _rm, CFG cfg, 
    InputIterator _itrFirst, InputIterator _itrLast, 
    int _k, 
    const vector<VID>& _iterNeighbors, OutputIterator _closestIter){
#ifndef _PARALLEL 
  if(m_random){
    // find k random (unique) neighbors
    set<int> ids(_iterNeighbors.begin(), _iterNeighbors.end());
    if(!m_unconnected && m_k != 0)
    {
      ids.insert(_rm->m_pRoadmap->GetVID(cfg));
      _k++;
    }
    for(int i = ids.size(); i < _k && i<(_itrLast-_itrFirst); i++){
      int id = 0;
      do {
        id = (int)(LRand()%(_itrLast - _itrFirst));
      } while(ids.find(id) != ids.end());
      ids.insert(id);

      *_closestIter = *(_itrFirst + id);
      ++_closestIter;
    }
    return _closestIter;
  }
  else {
    // find the k-closest neighbors
    NeighborhoodFinder::NeighborhoodFinderPointer nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetMethod(this->m_nfMethod);
    if(_itrLast - _itrFirst == (int)_rm->m_pRoadmap->get_num_vertices()) 
      return nfptr->KClosest(_rm, cfg, _k, _closestIter);
    else 
      return nfptr->KClosest(_rm, _itrFirst, _itrLast, cfg, _k, _closestIter);
  } 
#else
  // find k-closest using just brute force
  BFNF<CFG,WEIGHT>* bf_finder = new BFNF<CFG,WEIGHT>(this->GetMPProblem()->GetNeighborhoodFinder()->GetMethod(this->m_nfMethod)->GetDMMethod());
  return bf_finder->KClosest(_rm, _itrFirst, _itrLast, cfg, _k, _closestIter);
#endif
}            

///////////////////////////////////////////////////////////////////////////////
#endif

