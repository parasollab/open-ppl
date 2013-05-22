#ifndef CCSCONNECTOR_H_
#define CCSCONNECTOR_H_

#include "ConnectorMethod.h"

//CCsConnector
/**Try to connect different connected components of the roadmap.
 *We try to connect all pairs of connected components. If both
 *components are small (less than "m_smallcc" nodes), then we try to 
 *connect all pairs of nodes.  If at least one of the components is 
 *large, we try to connect the "m_kPairs" closest pairs of nodes.
 *
 *@see WeightedGraph ::Get CC_stats, WeightedGraph ::Get CC, 
 *WeightedGraph ::Is SameCC for information about connected 
 *component in graph,and ConnectNeighbors for 
 *connecting between connected component.
 */


#define KPAIRS        4       // default for connectCCs: k-pairs of closest cfgs between CCs
#define SMALL_CC      3       // default for connectCCs: all-pairs connection between CCs
#define K2_CLOSEST    0       // m_k2 closest CCs, default 0 means connect all CCs

template <class MPTraits>
class CCsConnector: public ConnectorMethod<MPTraits> {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename MPProblemType::VID VID; 
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename vector<VID>::iterator VIDIT;

    CCsConnector(MPProblemType* _problem = NULL, string _lp = "", string _nf = ""); 
    CCsConnector(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~CCsConnector();

    virtual void PrintOptions(ostream& _os);
    virtual void ParseXML(XMLNodeReader& _node);

    template <typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats,
          ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
          InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision);

    template<typename OutputIterator>
      void ConnectSmallCC(RoadmapType* _rm, StatClass& _stats,
          vector<VID>& _cc1Vec, vector<VID>& _cc2Vec,
          OutputIterator _collision);  

    template<typename OutputIterator>
      void ConnectBigCC(RoadmapType* _rm, StatClass& _stats,
          vector<VID>& _cc1Vec, vector<VID>& _cc2Vec,
          OutputIterator _collision);  

  protected:
    // compute all pair distance between ccs.
    // approximated using coms of ccs
    template<typename ColorMap, typename InputIterator>
      void ComputeAllPairsCCDist(RoadmapType* _rm,
          ColorMap& _cmap, InputIterator _ccs1First, InputIterator _ccs1Last,
          InputIterator _ccs2First, InputIterator _ccs2Last);

    //get k closest pairs
    void GetK2Pairs(int _ccid, vector<VID>& _kCCID);

    // get closest dist between two given CCs
    double ClosestInterCCDist(RoadmapType* _rm, vector<VID>& _cc1, vector<VID>& _cc2);

  private:
    size_t m_kPairs;
    size_t m_smallcc;
    size_t m_k2;

    map<VID, vector<pair<VID,double> > > m_ccDist;
};

template<class MPTraits>
CCsConnector<MPTraits>::CCsConnector(MPProblemType* _problem, string _lp, string _nf)
  : ConnectorMethod<MPTraits>() { 
    this->SetName("CCsConnector"); 
    this->m_lpMethod = _lp;
    this->m_nfMethod = _nf;
    this->SetMPProblem(_problem);
    m_kPairs = KPAIRS;
    m_smallcc = SMALL_CC;
    m_k2 = K2_CLOSEST;
  }

template<class MPTraits>
CCsConnector<MPTraits>::CCsConnector(MPProblemType* _problem, XMLNodeReader& _node) : ConnectorMethod<MPTraits>(_problem, _node) { 
  this->SetName("CCsConnector"); 
  ParseXML(_node);
}

template<class MPTraits>
void
CCsConnector<MPTraits>::ParseXML(XMLNodeReader& _node){
  m_kPairs = _node.numberXMLParameter("kpairs", true, 5,1,1000, "kpairs value"); 
  m_smallcc = _node.numberXMLParameter("smallcc", true, 5,1,1000,  "smallcc value"); 
  m_k2 = _node.numberXMLParameter("kclosest", true, 5,0,1000, "k closest CCs");
}

template<class MPTraits>
CCsConnector<MPTraits>::~CCsConnector(){}

template<class MPTraits>
void
CCsConnector<MPTraits>::PrintOptions(ostream& _os){
  ConnectorMethod<MPTraits>::PrintOptions(_os);
  _os << "    " << this->GetName() << "::  m_kPairs = ";
  _os << m_kPairs << "  m_smallcc = " << m_smallcc ;
  _os << "  m_k2 = " << m_k2;
  _os << endl;
}

template<class MPTraits>
template<typename ColorMap, typename InputIterator>
void
CCsConnector<MPTraits>::ComputeAllPairsCCDist(RoadmapType* _rm,
    ColorMap& _cmap, InputIterator _ccs1First, InputIterator _ccs1Last,
    InputIterator _ccs2First, InputIterator _ccs2Last){
  
  DistanceMetricPointer dmm; 
  dmm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  GraphType* rgraph=_rm->GetGraph();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  _cmap.reset();

  //compute com of ccs
  map<VID, CfgType> com1,com2;
  vector<VID> ccvids;
  for (InputIterator i = _ccs1First; i != _ccs1Last; i++){
    _cmap.reset();
    ccvids.clear();
    get_cc(*rgraph, _cmap, *i, ccvids);
    com1[*i] = GetCentroid(rgraph, ccvids);
  }
  for (InputIterator i = _ccs2First; i != _ccs2Last; i++){
    _cmap.reset();
    ccvids.clear();
    get_cc(*rgraph, _cmap, *i, ccvids);
    com2[*i] = GetCentroid(rgraph, ccvids);
  }

  //dist between ccs
  m_ccDist.clear();

  typedef typename map<VID, CfgType>::iterator IT;
  for(IT i = com1.begin(); i != com1.end(); ++i){
    for(IT j = com2.begin(); j != com2.end(); ++j){
      if (i->first != j->first){
        m_ccDist[i->first].push_back(make_pair(j->first, dmm->Distance(env,i->second,j->second)));
      }
      else{ //same cc
        m_ccDist[i->first].push_back(make_pair(j->first, MAX_DBL));
      }
    }
  }//end for i
}

//get m_k2 closest pairs of CCs
template<class MPTraits>
void
CCsConnector<MPTraits>::GetK2Pairs(int _ccid, vector<VID>& _k2CCID){
  typedef vector<double>::iterator IT;

  vector<pair<VID, double> >& dis2CCs = m_ccDist[_ccid];
  partial_sort(dis2CCs.begin(), dis2CCs.begin()+m_k2, dis2CCs.end(),
      CompareSecond<VID, double>());  

  //copy
  _k2CCID.clear();
  for(typename vector<pair<VID, double> >::iterator i=dis2CCs.begin(); i != (dis2CCs.begin() + m_k2); i++){
    _k2CCID.push_back(i->second);
  }
}

// Find the closest inter CC distance
template<class MPTraits>
double
CCsConnector<MPTraits>::ClosestInterCCDist(RoadmapType* _rm, 
    vector<VID>& _cc1, vector<VID>& _cc2){
  DistanceMetricPointer dmm;
  dmm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();
  Environment * env=_rm->GetEnvironment();

  double min_dist=1e20, dist = 0;
  const CfgType& cfg1;
  const CfgType& cfg2;

  for(VIDIT i =_cc1.begin(); i != _cc1.end(); ++i){
    cfg1 = _rm->GetGraph()->GetVertex(*i);
    for(VIDIT j = _cc2.begin(); j != _cc2.end(); ++j){
      cfg2 = _rm->GetGraph()->GetVertex(*j);
      dist = dmm->Distance(env,cfg1,cfg2);
      if(dist<min_dist){
        min_dist=dist;
      }
    }//end j
  }//end i
  return min_dist;
}

template<class MPTraits>
template<typename ColorMap, typename InputIterator, typename OutputIterator>
void
CCsConnector<MPTraits>::Connect( RoadmapType* _rm, StatClass& _stats, 
    ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision) {

  if(this->m_debug){
    cout << "components(m_kPairs="<< m_kPairs ;
    cout << ", m_smallcc="<<m_smallcc <<", m_k2=" << m_k2 << "): "<<flush;
    _stats.DisplayCCStats(cout, *(_rm->GetGraph())); 
    cout << endl;
  }

  GraphType* rgraph = _rm->GetGraph();
  vector< pair<size_t,VID> > ccs1;
  vector<VID> ccid,cc1,cc2;

  stapl::sequential::vector_property_map< GraphType,size_t > cmap;
  get_cc_stats(*rgraph,cmap,ccs1);

  if(ccs1.size() <= 1) return;

  VIDIT itr1,itr2;
  for(size_t i = 0; i < ccs1.size(); ++i){
    ccid.push_back(ccs1[i].second);
  }

  ///////////////////////////////////////////////////////////////////////////////////
  /// ConnectkCCs
  ///////////////////////////////////////////////////////////////////////////////////
  if(m_k2 != 0){ // Attempt to Connect K2-Closest CCs

    if(this->m_debug) 
      cout << "Connecting " << m_k2 << "-closest CCs" << endl;
    ComputeAllPairsCCDist(_rm, _cmap, ccid.begin(), ccid.end(), ccid.begin(), ccid.end());

    for (itr1 = ccid.begin(); itr1 != ccid.end(); ++itr1) {
      vector<VID> k2CCID;
      GetK2Pairs(*itr1,k2CCID);
      for (VIDIT itr2 = k2CCID.begin(); itr2 != k2CCID.end(); ++itr2) {
        _cmap.reset();
        if ( !stapl::sequential::is_same_cc(*rgraph,_cmap,*itr1,*itr2) ) {

          _cmap.reset();
          get_cc(*rgraph,_cmap,*itr1,cc1);
          _cmap.reset();
          get_cc(*rgraph,_cmap,*itr2,cc2);

          if(cc1.size() < m_smallcc && cc2.size() < m_smallcc){
            ConnectSmallCC(_rm, _stats, cc1, cc2, _collision);
          }
          else if(cc1.size() <= cc2.size()){
            ConnectBigCC(_rm, _stats, cc1, cc2, _collision);
          }
          else{
            ConnectBigCC(_rm, _stats, cc2, cc1, _collision);
          }
        }
        if(this->m_debug)
          cout << " ...done\n";
      }
    }
  } 

  ///////////////////////////////////////////////////////////////////////////////////
  /// CCsConnector
  ///////////////////////////////////////////////////////////////////////////////////
  else { // Attempt to Connect CCs, All-Pairs for all CCs
    if(this->m_debug) 
      cout << "Connecting CCs" << endl;
    for (itr1 = ccid.begin(); itr1 != ccid.end(); ++itr1) {
      for (itr2 = itr1+1; itr2 != ccid.end(); ++itr2) {
        _cmap.reset();
        if ( !stapl::sequential::is_same_cc(*rgraph,_cmap,*itr1,*itr2) ) {

          _cmap.reset();
          get_cc(*rgraph,_cmap,*itr1,cc1);
          _cmap.reset();
          get_cc(*rgraph,_cmap,*itr2,cc2);

          if(cc1.size() < m_smallcc && cc2.size() < m_smallcc){
            ConnectSmallCC(_rm, _stats, cc1, cc2, _collision);
          }
          else if(cc1.size() <= cc2.size()){
            ConnectBigCC(_rm, _stats, cc1, cc2, _collision);
          }
          else{
            ConnectBigCC(_rm, _stats, cc2, cc1, _collision);
          }
        }
        if(this->m_debug)
          cout << " ...done\n";
      }
    }
  }
  if(this->m_debug) {
    _stats.DisplayCCStats(cout, *(rgraph)); 
    cout << endl;
  }
}

template<class MPTraits>
template<typename OutputIterator>
void
CCsConnector<MPTraits>::ConnectSmallCC( RoadmapType* _rm, StatClass& _stats,
    vector<VID>& _cc1Vec, vector<VID>& _cc2Vec, OutputIterator _collision) {

  GraphType* rgraph = _rm->GetGraph();
  LPOutput<MPTraits> lpOutput;
  DistanceMetricPointer dmm;
  dmm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  // Begin the connection attempts
  VID cc1Elem, cc2Elem;
  for (size_t i = 0; i < _cc1Vec.size(); ++i){
    cc1Elem = _cc1Vec[i];
    for (size_t j = 0; j < _cc2Vec.size(); ++j){
      cc2Elem = _cc2Vec[j];
      if(_rm->IsCached(cc1Elem, cc2Elem)) {
        continue;
      }
      CfgType _col;
      if (!rgraph->IsEdge(cc1Elem, cc2Elem) 
          && this->GetMPProblem()->GetLocalPlanner(this->m_lpMethod)->
          IsConnected(this->GetMPProblem()->GetEnvironment(),_stats,dmm,
            rgraph->GetVertex(cc1Elem),
            rgraph->GetVertex(cc2Elem),
            _col, &lpOutput, this->m_connectionPosRes, 
            this->m_connectionOriRes, !this->m_addAllEdges)) {
        rgraph->AddEdge(cc1Elem, cc2Elem, lpOutput.edge);
        return;
      }
      else if(this->m_addPartialEdge) {
        typename vector<typename LPOutput<MPTraits>::LPSavedEdge>::iterator eit;
        for(eit=lpOutput.savedEdge.begin(); eit!=lpOutput.savedEdge.end(); eit++) {
          CfgType tmp = eit->first.second;
          if(tmp != rgraph->GetVertex(cc1Elem)){
            VID tmpVID = rgraph->AddVertex(tmp);
            rgraph->AddEdge(cc1Elem, tmpVID, eit->second);
          }
        }
      }
      if(_col != CfgType()){
        *_collision++ = _col;
      }
    }//end for c2
  }//end for c1
}

template<class MPTraits>
template<typename OutputIterator>
void
CCsConnector<MPTraits>::ConnectBigCC( RoadmapType* _rm, StatClass& _stats,
    vector<VID>& _cc1Vec, vector<VID>& _cc2Vec, OutputIterator _collision) {

  GraphType* rgraph = _rm->GetGraph();
  LPOutput<MPTraits> lpOutput;
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod);
  DistanceMetricPointer dmm = nf->GetDMMethod();

  size_t k = min(m_kPairs, _cc2Vec.size()); // for connecting k-closest pairs of nodes between CCs

  vector<pair<VID,VID> > kp(k);
  typename vector<pair<VID,VID> >::iterator kpIter = kp.begin();

  nf->KClosestPairs(_rm, _cc1Vec.begin(), _cc1Vec.end(), _cc2Vec.begin(), _cc2Vec.end(), k, kpIter);

  // Begin the connection attempts
  VID cc1Elem, cc2Elem;
  for (size_t i = 0; i < kp.size(); ++i){

    cc1Elem = kp[i].first;
    cc2Elem = kp[i].second;

    //_stats.IncConnections_Attempted();
    if(_rm->IsCached(cc1Elem, cc2Elem)) {
      continue;
    }
    CfgType _col;
    if (!rgraph->IsEdge(cc1Elem,cc2Elem) 
        && this->GetMPProblem()->GetLocalPlanner(this->m_lpMethod)->
        IsConnected(this->GetMPProblem()->GetEnvironment(),_stats,dmm,
          rgraph->GetVertex(cc1Elem),
          rgraph->GetVertex(cc2Elem),
          _col, &lpOutput, this->m_connectionPosRes, 
          this->m_connectionOriRes, !this->m_addAllEdges)) {
      rgraph->AddEdge(cc1Elem, cc2Elem, lpOutput.edge);
      return;
    }
    else if(this->m_addPartialEdge) {
      typename vector<typename LPOutput<MPTraits>::LPSavedEdge>::iterator eit;
      for(eit = lpOutput.savedEdge.begin(); eit != lpOutput.savedEdge.end(); eit++) {
        CfgType tmp = eit->first.second;
        if(tmp != rgraph->GetVertex(cc1Elem)){
          VID tmpVID = rgraph->AddVertex(tmp);
          rgraph->AddEdge(cc1Elem, tmpVID, eit->second);
        }
      }
    }
    if(_col != CfgType()){
      *_collision++ = _col;
    }
  }
}

#endif

