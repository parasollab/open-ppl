#ifndef CCS_CONNECTOR_H_
#define CCS_CONNECTOR_H_

#include "ConnectorMethod.h"

#include <containers/sequential/graph/algorithms/connected_components.h>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief Attempt connection between nearby CCs
/// @tparam MPTraits Motion planning universe
///
/// Attempts connection between different connected components of the roadmap.
/// We try to connect k-closest pairs of connected components.
/// When connecting the CCs, we only attempt neighboring pairs of nodes.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class CCsConnector: public ConnectorMethod<MPTraits> {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;

    CCsConnector(string _nfLabel = "", string _lpLabel = "", size_t _k = 5);
    CCsConnector(MPProblemType* _problem, XMLNode& _node);

    virtual void Print(ostream& _os) const;

    template<typename InputIterator1, typename InputIterator2,
      typename OutputIterator>
        void Connect(RoadmapType* _rm,
            InputIterator1 _itr1First, InputIterator1 _itr1Last,
            InputIterator2 _itr2First, InputIterator2 _itr2Last,
            bool _fromFullRoadmap,
            OutputIterator _collision);

    template<typename OutputIterator>
      void ConnectCC(RoadmapType* _rm,
          vector<VID>& _cc1Vec, vector<VID>& _cc2Vec,
          OutputIterator _collision);

  protected:

    // compute all pair distance between ccs.
    // approximated using coms of ccs
    void ComputeAllPairsCCDist(RoadmapType* _rm, vector<pair<size_t, VID> >& _ccs);

    //get k closest pairs
    void GetKCCs(size_t _k, VID _ccid, vector<VID>& _kCCID);

  private:
    size_t m_k; //how many closest CCs to connect to

    map<VID, vector<pair<VID,double> > > m_ccDist;
};

template<class MPTraits>
CCsConnector<MPTraits>::CCsConnector(string _nfLabel, string _lpLabel, size_t _k)
  : ConnectorMethod<MPTraits>(_nfLabel, _lpLabel), m_k(_k) {
    this->SetName("CCsConnector");
  }

template<class MPTraits>
CCsConnector<MPTraits>::CCsConnector(MPProblemType* _problem, XMLNode& _node)
  : ConnectorMethod<MPTraits>(_problem, _node) {
    this->SetName("CCsConnector");
    m_k = _node.Read("k", true, 5, 0, 1000, "k closest CCs");
  }

template<class MPTraits>
void
CCsConnector<MPTraits>::Print(ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "\tk: " << m_k << endl;
}

template<class MPTraits>
template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
void
CCsConnector<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision) {

  GraphType* rgraph = _rm->GetGraph();

  if(this->m_debug){
    this->GetStatClass()->DisplayCCStats(cout, *rgraph);
    cout << endl;
  }

  vector<pair<size_t,VID> > ccs;

  typename GraphType::ColorMap colorMap;
  get_cc_stats(*rgraph, colorMap, ccs);

  if(ccs.size() <= 1) return;

  ///////////////////////////////////////////////////////////////////////////////////
  /// ConnectkCCs
  ///////////////////////////////////////////////////////////////////////////////////
  size_t k = m_k ? m_k : ccs.size()-1;

  if(this->m_debug)
    cout << "Connecting " << m_k << "-closest CCs" << endl;

  ComputeAllPairsCCDist(_rm, ccs);

  typedef typename vector<pair<size_t, VID> >::iterator CCIT;
  for(CCIT itr1 = ccs.begin(); itr1 != ccs.end(); ++itr1) {

    //grab the k-closest CCs to connect to
    vector<VID> kCCID;
    GetKCCs(k, itr1->second, kCCID);

    typedef typename vector<VID>::iterator VIDIT;
    for(VIDIT itr2 = kCCID.begin(); itr2 != kCCID.end(); ++itr2) {

      //even though this might be inefficient, double check to make sure the CCs
      //are different
      if(!stapl::sequential::is_same_cc(*rgraph, colorMap, itr1->second, *itr2)) {

        vector<VID> cc1, cc2;

        get_cc(*rgraph, colorMap, itr1->second, cc1);
        get_cc(*rgraph, colorMap, *itr2, cc2);

        ConnectCC(_rm, cc1, cc2, _collision);
        colorMap.reset();
      }
      if(this->m_debug)
        cout << " ...done\n";
    }
  }

  if(this->m_debug) {
    this->GetStatClass()->DisplayCCStats(cout, *rgraph);
    cout << endl;
  }
}

template<class MPTraits>
template<typename OutputIterator>
void
CCsConnector<MPTraits>::ConnectCC(RoadmapType* _rm,
    vector<VID>& _cc1Vec, vector<VID>& _cc2Vec, OutputIterator _collision) {

  Environment* env = this->GetEnvironment();
  GraphType* rgraph = _rm->GetGraph();
  LPOutput<MPTraits> lpOutput;
  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  LocalPlannerPointer lp = this->GetLocalPlanner(this->m_lpLabel);

  typedef vector<pair<pair<VID, VID>, double> > NeighborPairs;
  typedef typename NeighborPairs::iterator NPIT;

  NeighborPairs neighborPairs;
  nf->FindNeighborPairs(_rm, _cc1Vec.begin(), _cc1Vec.end(), _cc2Vec.begin(), _cc2Vec.end(), back_inserter(neighborPairs));

  // Begin the connection attempts
  for(NPIT npit = neighborPairs.begin(); npit!=neighborPairs.end(); ++npit){

    VID cc1Elem = npit->first.first;
    VID cc2Elem = npit->first.second;

    CfgType _col;
    if (lp->IsConnected(
          rgraph->GetVertex(cc1Elem),
          rgraph->GetVertex(cc2Elem),
          _col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true)) {
      rgraph->AddEdge(cc1Elem, cc2Elem, lpOutput.m_edge);
      return;
    }
    if(_col != CfgType())
      *_collision++ = _col;
  }
}

template<class MPTraits>
void
CCsConnector<MPTraits>::
ComputeAllPairsCCDist(RoadmapType* _rm, vector<pair<size_t, VID> >& _ccs) {

  GraphType* rgraph=_rm->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel)->GetDMMethod();
  typename GraphType::ColorMap colorMap;

  //compute com of ccs
  map<VID, CfgType> coms;
  typedef typename vector<pair<size_t, VID> >::iterator CCIT;
  for(CCIT cc = _ccs.begin(); cc != _ccs.end(); ++cc){
    vector<VID> ccvids;
    get_cc(*rgraph, colorMap, cc->second, ccvids);
    coms[cc->second] = GetCentroid(rgraph, ccvids);
  }

  //dist between ccs
  m_ccDist.clear();
  typedef typename map<VID, CfgType>::iterator IT;
  for(IT i = coms.begin(); i != coms.end(); ++i)
    for(IT j = i; j != coms.end(); ++j)
      //dont track the CC distance if i and j are the same
      if(i != j){
        double dist = dmm->Distance(i->second, j->second);
        m_ccDist[i->first].push_back(make_pair(j->first, dist));
        m_ccDist[j->first].push_back(make_pair(i->first, dist));
      }
}

//get m_k closest pairs of CCs
template<class MPTraits>
void
CCsConnector<MPTraits>::GetKCCs(size_t _k, VID _ccid, vector<VID>& _kCCID){
  typedef vector<double>::iterator IT;
  vector<pair<VID, double> >& dis2CCs = m_ccDist[_ccid];
  partial_sort(dis2CCs.begin(), dis2CCs.begin() + _k, dis2CCs.end(), CompareSecond<VID, double>());

  //copy
  for(typename vector<pair<VID, double> >::iterator i=dis2CCs.begin(); i != dis2CCs.begin() + _k; ++i)
    _kCCID.push_back(i->first);
}

#endif
