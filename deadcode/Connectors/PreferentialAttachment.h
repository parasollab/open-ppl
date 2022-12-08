#ifndef PreferentialAttachment_h
#define PreferentialAttachment_h

#include "ConnectionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
class PreferentialAttachment: public ConnectionMethod<CFG,WEIGHT> {
  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    typedef typename vector<typename RoadmapGraph<CFG,WEIGHT>::VID>::iterator VIDIT;

    //////////////////////
    // Constructors and Destructor
    PreferentialAttachment(string _lp = "", string _nf = "", MPProblem* _problem = NULL);
    PreferentialAttachment(XMLNode& _node, MPProblem* _problem);
    PreferentialAttachment(int _k); // constructor for preferential attachment
    PreferentialAttachment(int _k, int _m); // constructor for preferential attachment

    virtual ~PreferentialAttachment();

    //Used in new MPProblem framework.
    virtual void Print(ostream& _os) const;
    virtual void ParseXML(XMLNode& _node);

    //////////////////////
    // Core: Connection method
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
          ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
          InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision);

    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void ConnectNeighbors(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
          ColorMap& _cmap, VID _vid,
          InputIterator _closestFirst, InputIterator _closestLast,
          OutputIterator _collision);

  private:

    //////////////////////
    // Probability function
    double PrefProb(Roadmap<CFG, WEIGHT>* _rm, VID _vid, int _n) {
      int candidate_degree = _rm->m_pRoadmap->get_degree(_vid);
      int total_degree = _rm->m_pRoadmap->get_num_edges();
      if (m_debug) cout << "PrefProb(" << _vid << ", " << _n << ") = " << 1 + candidate_degree << " / " << _n + total_degree << endl;
      return ((double)(1 + candidate_degree) / (double)(_n + total_degree));
    }
    //////////////////////
    // Data

    int m_totalSuccess;
    int m_totalFailure;
    int m_k;
    int m_fail;
    bool m_unconnected;
    bool m_random;
    bool m_debug;
    bool m_checkIfSameCC;
    string m_dm;
};


template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::PreferentialAttachment(string _lp, string _nf, MPProblem* _problem){
  this->SetName("PreferentialAttachment");
  this->m_lpMethod = _lp;
  this->m_nfMethod = _nf;
  this->SetMPProblem(_problem);
  m_k = KCLOSEST;
  m_fail = MFAILURE;
  m_checkIfSameCC = true;
}

template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::PreferentialAttachment(XMLNode& _node, MPProblem* _problem) :
  ConnectionMethod<CFG,WEIGHT>(_node, _problem) {
    this->SetName("PreferentialAttachment");
    m_k = KCLOSEST;
    m_fail = MFAILURE;
    ParseXML(_node);
  }


//this is backward support for function call from other class
//to be cleaned
template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::PreferentialAttachment(int _k) :
  ConnectionMethod<CFG,WEIGHT>() {
    this->SetName("PreferentialAttachment");
    m_k = _k;
    m_fail = _k;
    m_unconnected = false;
    m_random = false;
    m_debug = false;
  }

template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::PreferentialAttachment(int _k, int _m) :
  ConnectionMethod<CFG,WEIGHT>() {
    this->SetName("PreferentialAttachment");
    m_k = _k;
    m_fail = _m;
    m_unconnected = false;
    m_random = false;
    m_debug = false;
  }


template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::~PreferentialAttachment() {
}

template <class CFG, class WEIGHT>
void PreferentialAttachment<CFG,WEIGHT>::ParseXML(XMLNode& _node) {
  m_checkIfSameCC = _node.Read("CheckIfSameCC",false,true,"If true, do not connect if edges are in the same CC");
  m_k = _node.Read("k", true, 0, 0, 10000, "k-value (max neighbors to find). k = 0 --> all-pairs");
  m_fail = _node.Read("fail", false, 5, 0, 10000, "amount of failed connections allowed before operation terminates");
  m_unconnected = _node.Read("unconnected", false, false, "if true, do not count existing connections towards k");
  m_random = _node.Read("random", false, false, "if true, find k random configurations from destination vector");
  m_debug = _node.Read("debug", false, false, "");
  m_dm =_node.Read("dm_method", false, "default", "Distance Metric Method");

  XMLNode::childiterator citr;
  for(citr = _node.children_begin(); citr!= _node.children_end(); ++citr) {
    if(citr->getName() == "Shell") {
      //m_shells.push_back(new Shell<CFG, WEIGHT>(*citr, GetMPProblem()));
    } else {
      citr->warnUnknownNode();
    }
  }

}

template <class CFG, class WEIGHT>
void
PreferentialAttachment<CFG, WEIGHT>::Print(ostream& _os) const {
  ConnectionMethod<CFG,WEIGHT>::Print(_os);
  _os << "    " << this->GetName() << "::  k = ";
  _os << m_k << "  fail = " << m_fail ;
  _os << endl;
}

/**
 * This function connects the nodes between the InputIterators to
 * themselves, using the following algorithm:
 *
 * func Preferential-K (V, k)
 * -----------
 * for n = 3 ... V.size()
 *      attempts = 0
 *      for i = 0 ... n-1
 *          if RAND(0...1) > p( i ) then
 *              attempt_connection(V[i], V[n])
 *              attempts += 1
 *          endif
 *          if (attempts == MIN(k, n)) then
 *              return
 *          endif
 *      endfor
 * endfor
 */
template <class CFG, class WEIGHT>
template<typename ColorMap, typename InputIterator, typename OutputIterator>
void PreferentialAttachment<CFG,WEIGHT>::Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision){

  vector<VID> inputVertices(_itr2First, _itr2Last);
  for(InputIterator itr1 = _itr1First; itr1 != _itr1Last; ++itr1){
    m_totalSuccess = m_totalFailure = 0;


    for (size_t n = 2; n < inputVertices.size(); n++) {
      if (m_debug) {
        cout << "***************************************************************\n";
        cout << "VID - inputVertices[" << n << "] = " << inputVertices[n] << endl;
      }
      int attempts = 0;

      _stats.StartClock("kClosest");
      while (attempts < min(m_k, n-1)) {
        for (size_t i = 0; i < n-1; i++) {
          double drand = DRand();
          double prob = PrefProb(_rm, inputVertices[i], n);
          if (m_debug) cout << "attempts = " << attempts << ", drand = " << drand << ", prob = " << prob << endl;
          if (drand < prob) {

            vector<VID> candidate;
            candidate.push_back(inputVertices[i]);

            _stats.StopClock("kClosest");
            if (m_debug) cout << "\tAttempting connections: VID = " << inputVertices[n] << "  --> " << candidate[0] << endl;
            ConnectNeighbors(_rm, _stats, _cmap, inputVertices[n], candidate.begin(), candidate.end(), _collision);
            _stats.StartClock("kClosest");

            attempts += 1;
          }
          if (attempts == min(m_k, n-1)) {
            break;
          }
        }
      }
      _stats.StopClock("kClosest");
    }
  }

  if (m_debug) cout << "*** kClosest Time = " << _stats.GetSeconds("kClosest") << endl;
  if (m_debug) cout << "*** m_totalSuccess = " << m_totalSuccess << endl;
  if (m_debug) cout << "*** m_totalFailure = " << m_totalFailure << endl;
}

template <class CFG, class WEIGHT>
template <typename ColorMap, typename InputIterator, typename OutputIterator>
void PreferentialAttachment<CFG,WEIGHT>::ConnectNeighbors(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    ColorMap& _cmap, VID _vid,
    InputIterator _closestFirst, InputIterator _closestLast,
    OutputIterator _collision){
  LPOutput<CFG,WEIGHT> lpOutput;
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);

  typedef RoadmapGraph<CFG, WEIGHT> RoadmapGraphType;
  typedef pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  // connect the found k-closest to the current iteration's CFG
  for(typename vector<VID>::iterator itr2 = _closestFirst; itr2!= _closestLast; ++itr2) {

    if (m_debug) cout << "\t(s,f) = (" << m_totalSuccess << "," << m_totalFailure << ")";
    if (m_debug) cout << " | VID = " << *itr2;
    if (m_debug) cout << " | dist = " << dm->Distance(_rm->GetEnvironment(),
        GetCfg()(_rm->m_pRoadmap, _vid),
        GetCfg()(_rm->m_pRoadmap, itr2));

    // don't attempt an edge between the same nodes
    if (_vid == *itr2) {
      if (m_debug) cout << " | skipping... same nodes" << endl;
      continue;
    }

    // don't attempt the connection if it already failed once before
    if (_rm->IsCached(_vid,*itr2)) {
      if (!_rm->GetCache(_vid,*itr2)) {
        if (m_debug) cout << " | skipping... this connection already failed once";
        if (!m_unconnected) {
          if (m_debug) cout << " | failure incremented";
          m_totalFailure++;
        }
        if (m_debug) cout << endl;
        continue;
      }
    }
#ifndef _PARALLEL
    // the edge already exists
    if (_rm->m_pRoadmap->IsEdge(_vid, *itr2)) {
      // if we're not in "unconnected" mode, count this as a success
      if (m_debug) cout << " | edge already exists in roadmap";
      if (!m_unconnected) {
        if (m_debug) cout << " | success incremented";
        m_totalSuccess++;
      }
      if (m_debug) cout << endl;
      continue;
    }
    if (m_checkIfSameCC) {
      // the nodes are in the same connected component
      _cmap.reset();
      if (is_same_cc(*(_rm->m_pRoadmap), _cmap, _vid, *itr2)) {
        // if we're not in "unconnected" mode, count this as a success
        if (m_debug) cout << " | nodes in the same connected component";
        if (!m_unconnected) {
          if (m_debug) cout << " | success incremented";
          m_totalSuccess++;
        }
        if (m_debug) cout << endl;
        continue;
      }
    }
#endif
    // attempt connection with the local planner
    CfgType _col;
    if(this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpMethod)->
        IsConnected(_rm->GetEnvironment(), _stats, dm,
          GetCfg()(_rm->m_pRoadmap, _vid),
          GetCfg()(_rm->m_pRoadmap, itr2),
          _col, &lpOutput, this->m_connectionPosRes, this->m_connectionOriRes,
          (!this->m_addAllEdges) )){
      // if connection was made, add edge and record the successful connection
      if (m_debug) cout << " | connection was successful";
      _rm->m_pRoadmap->AddEdge(_vid, *itr2, lpOutput.edge);
      //this->connection_attempts.push_back(make_pair(make_pair(_vid, *itr2), true));

      // mark the successful connection in the roadmap's cache
      if (m_debug) cout << " | success incremented" << endl;
      _rm->SetCache(_vid,*itr2,true);
      m_totalSuccess++;
    }
    else {
      // mark the failed connection in the roadmap's cache
      if (m_debug) cout << " | connection failed | failure incremented" << endl;
      _rm->SetCache(_vid,*itr2,false);
      m_totalFailure++;
      //this->connection_attempts.push_back(make_pair(make_pair(_vid, *itr2), false));
    }
    if(_col != CfgType())
      *_collision++ = _col;
  }
}

#endif
