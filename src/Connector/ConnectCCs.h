#ifndef CONNECTCCS_H_
#define CONNECTCCS_H_

#include "ConnectionMethod.h"

//ConnectCCs
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

template <class CFG, class WEIGHT>
class ConnectCCs: public ConnectionMethod<CFG,WEIGHT> {
  public:
    //////////////////////
    // Typedefs from RoadmapGraph
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    typedef typename vector<typename RoadmapGraph<CFG,WEIGHT>::VID>::iterator VIDIT;

    //////////////////////
    // Constructors and Destructor
    ConnectCCs();
    ConnectCCs(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~ConnectCCs();

    //////////////////////
    ///Used in new MPProblem framework.
    virtual void PrintOptions(ostream& _os);
    virtual void ParseXML(XMLNodeReader& _node);

    template <typename ColorMap, typename InputIterator, typename OutputIterator>
      void Connect(Roadmap<CFG, WEIGHT>*, StatClass& _stats,
          ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
          InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision);

    template<typename OutputIterator>
      void ConnectSmallCC(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
          vector<VID>& _cc1Vec, vector<VID>& _cc2Vec,
          OutputIterator _collision);  

    template<typename OutputIterator>
      void ConnectBigCC(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
          vector<VID>& _cc1Vec, vector<VID>& _cc2Vec,
          OutputIterator _collision);  

  protected:
    // compute all pair distance between ccs.
    // approximated using coms of ccs
    template<typename ColorMap, typename InputIterator>
      void ComputeAllPairsCCDist(Roadmap<CFG, WEIGHT>* _rm,
          ColorMap& _cmap, InputIterator _ccs1First, InputIterator _ccs1Last,
          InputIterator _ccs2First, InputIterator _ccs2Last);

    //get k closest pairs
    void GetK2Pairs(int _ccid, vector<VID>& _kCCID);

    // get closest dist between two given CCs
    double ClosestInterCCDist(Roadmap<CFG, WEIGHT>* _rm, vector<VID>& _cc1, vector<VID>& _cc2);

  private:
    //////////////////////
    // Data from ConnectCCs
    size_t m_kPairs;
    size_t m_smallcc;
    size_t m_k2;

    // Data from ConnectkCCs
    map<VID, vector<pair<VID,double> > > m_ccDist;
};


///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  ConnectCCs
template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::ConnectCCs() : ConnectionMethod<CFG,WEIGHT>() { 
  this->SetName("ConnectCCs"); 
  m_kPairs = KPAIRS;
  m_smallcc = SMALL_CC;
  m_k2 = K2_CLOSEST;
}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::ConnectCCs(XMLNodeReader& _node, MPProblem* _problem) : ConnectionMethod<CFG,WEIGHT>(_node, _problem) { 
  ParseXML(_node);
}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void ConnectCCs<CFG,WEIGHT>::ParseXML(XMLNodeReader& _node){
  this->SetName("ConnectCCs"); 

  m_kPairs = KPAIRS;
  m_smallcc = SMALL_CC;
  m_k2 = K2_CLOSEST;

  m_kPairs = _node.numberXMLParameter("kpairs", true, 5,1,1000, "kpairs value"); 
  m_smallcc = _node.numberXMLParameter("smallcc", true, 5,1,1000,  "smallcc value"); 
  m_k2 = _node.numberXMLParameter("kclosest", true, 5,0,1000, "k closest CCs");

}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
ConnectCCs<CFG,WEIGHT>::~ConnectCCs(){}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::PrintOptions(ostream& _os){
  ConnectionMethod<CFG,WEIGHT>::PrintOptions(_os);
  _os << "    " << this->GetName() << "::  m_kPairs = ";
  _os << m_kPairs << "  m_smallcc = " << m_smallcc ;
  _os << "  m_k2 = " << m_k2;
  _os << endl;
}

///////////////////////////////////////////////////////////////////////////////
template <typename CFG, typename WEIGHT>
template <typename ColorMap, typename InputIterator>
void ConnectCCs<CFG,WEIGHT>::ComputeAllPairsCCDist(Roadmap<CFG, WEIGHT>* _rm,
    ColorMap& _cmap, InputIterator _ccs1First, InputIterator _ccs1Last,
    InputIterator _ccs2First, InputIterator _ccs2Last){

  shared_ptr<DistanceMetricMethod> dm = 
    this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod)->GetDMMethod();

  RoadmapGraph<CFG,WEIGHT>* rmapG=_rm->m_pRoadmap;
  Environment* env=_rm->GetEnvironment();
  _cmap.reset();

  //compute com of ccs
  map<VID, CFG> com1,com2;
  vector<VID> ccvids;
  for (InputIterator i = _ccs1First; i != _ccs1Last; i++){
    _cmap.reset();
    ccvids.clear();
    get_cc(*rmapG, _cmap, *i, ccvids);
    com1[*i] = GetCentroid(rmapG, ccvids);
  }
  for (InputIterator i = _ccs2First; i != _ccs2Last; i++){
    _cmap.reset();
    ccvids.clear();
    get_cc(*rmapG, _cmap, *i, ccvids);
    com2[*i] = GetCentroid(rmapG, ccvids);
  }

  //dist between ccs
  m_ccDist.clear();

  typedef typename map<VID, CFG>::iterator IT;
  for(IT i = com1.begin(); i != com1.end(); ++i){
    for(IT j = com2.begin(); j != com2.end(); ++j){
      if (i->first != j->first){
        m_ccDist[i->first].push_back(make_pair(j->first, dm->Distance(env,i->second,j->second)));
      }
      else{ //same cc
        m_ccDist[i->first].push_back(make_pair(j->first, MAX_DBL));
      }
    }
  }//end for i
}

///////////////////////////////////////////////////////////////////////////////
//get m_k2 closest pairs of CCs
template <class CFG, class WEIGHT>
void ConnectCCs<CFG,WEIGHT>::GetK2Pairs(int _ccid, vector<VID>& _k2CCID){
  typedef vector<double>::iterator IT;

  vector<pair<VID, double> >& dis2CCs = m_ccDist[_ccid];
  partial_sort(dis2CCs.begin(), dis2CCs.begin()+m_k2, dis2CCs.end(),
      compare_second<VID, double>());  

  //copy
  _k2CCID.clear();
  for(typename vector<pair<VID, double> >::iterator i=dis2CCs.begin(); i != (dis2CCs.begin() + m_k2); i++){
    _k2CCID.push_back(i->second);
  }
}


///////////////////////////////////////////////////////////////////////////////
// Find the closest inter CC distance
template <class CFG, class WEIGHT>
double ConnectCCs<CFG,WEIGHT>::ClosestInterCCDist(Roadmap<CFG, WEIGHT>* _rm, 
    vector<VID>& _cc1, vector<VID>& _cc2){
  shared_ptr<DistanceMetricMethod> dm = 
    this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod)->GetDMMethod();

  RoadmapGraph<CFG,WEIGHT> * rmapG=_rm->m_pRoadmap;
  Environment * env=_rm->GetEnvironment();

  double min_dist=1e20, dist = 0;
  const CFG& cfg1;
  const CFG& cfg2;

  for(VIDIT i =_cc1.begin(); i != _cc1.end(); ++i){
    cfg1 = pmpl_detail::GetCfg<VIDIT>(_rm->m_pRoadmap)(i);
    for(VIDIT j = _cc2.begin(); j != _cc2.end(); ++j){
      cfg2 = pmpl_detail::GetCfg<VIDIT>(_rm->m_pRoadmap)(j);
      dist = dm->Distance(env,cfg1,cfg2);
      if(dist<min_dist){
        min_dist=dist;
      }
    }//end j
  }//end i
  return min_dist;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template<typename ColorMap, typename InputIterator, typename OutputIterator>
void ConnectCCs<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, 
    ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision) {

  if(this->m_debug){
    cout << "components(m_kPairs="<< m_kPairs ;
    cout << ", m_smallcc="<<m_smallcc <<", m_k2=" << m_k2 << "): "<<flush;
    _stats.DisplayCCStats(cout, *(_rm->m_pRoadmap)); 
    cout << endl;
  }

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector< pair<size_t,VID> > ccs1;
  vector<VID> ccid,cc1,cc2;

  stapl::sequential::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
  get_cc_stats(*(_rm->m_pRoadmap),cmap,ccs1);
  
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
        if ( !stapl::sequential::is_same_cc(*pMap,_cmap,*itr1,*itr2) ) {

          _cmap.reset();
          get_cc(*pMap,_cmap,*itr1,cc1);
          _cmap.reset();
          get_cc(*pMap,_cmap,*itr2,cc2);

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
  /// ConnectCCs
  ///////////////////////////////////////////////////////////////////////////////////
  else { // Attempt to Connect CCs, All-Pairs for all CCs
    if(this->m_debug) 
      cout << "Connecting CCs" << endl;
    for (itr1 = ccid.begin(); itr1 != ccid.end(); ++itr1) {
      for (itr2 = itr1+1; itr2 != ccid.end(); ++itr2) {
        _cmap.reset();
        if ( !stapl::sequential::is_same_cc(*pMap,_cmap,*itr1,*itr2) ) {

          _cmap.reset();
          get_cc(*pMap,_cmap,*itr1,cc1);
          _cmap.reset();
          get_cc(*pMap,_cmap,*itr2,cc2);

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
    _stats.DisplayCCStats(cout, *(_rm->m_pRoadmap)); 
    cout << endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template <typename OutputIterator>
void ConnectCCs<CFG, WEIGHT>::ConnectSmallCC( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    vector<VID>& _cc1Vec, vector<VID>& _cc2Vec, OutputIterator _collision) {

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  LPOutput<CFG, WEIGHT> lpOutput;
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod)->GetDMMethod();

  // Begin the connection attempts
  VID cc1Elem, cc2Elem;
  for (size_t i = 0; i < _cc1Vec.size(); ++i){
    cc1Elem = _cc1Vec[i];
    for (size_t j = 0; j < _cc2Vec.size(); ++j){
      cc2Elem = _cc2Vec[j];
      if(_rm->IsCached(cc1Elem, cc2Elem) && !_rm->GetCache(cc1Elem, cc2Elem)) {
        continue;
      }
      CfgType _col;
      if (!_rm->m_pRoadmap->IsEdge(cc1Elem,cc2Elem) 
          && this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpMethod)->
          IsConnected( _rm->GetEnvironment(),_stats,dm,
            pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(cc1Elem),
            pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(cc2Elem),
            _col, &lpOutput, this->m_connectionPosRes, 
            this->m_connectionOriRes, !this->m_addAllEdges) ) {
        pMap->AddEdge(cc1Elem, cc2Elem, lpOutput.edge);
        return;
      }
      else if(this->m_addPartialEdge) {
        typename vector<pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > >::iterator I;
        for(I=lpOutput.savedEdge.begin(); I!=lpOutput.savedEdge.end(); I++) {
          CFG tmp = I->first.second;
          if(!tmp.AlmostEqual(pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(cc1Elem))){
            VID tmpVID = pMap->AddVertex(tmp);
            pMap->AddEdge(cc1Elem, tmpVID, I->second);
          }
        }
      }
      if(_col != CfgType()){
        *_collision++ = _col;
      }
    }//end for c2
  }//end for c1
}

///////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT>
template <typename OutputIterator>
void ConnectCCs<CFG, WEIGHT>::ConnectBigCC( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    vector<VID>& _cc1Vec, vector<VID>& _cc2Vec, OutputIterator _collision) {

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  LPOutput<CFG, WEIGHT> lpOutput;
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod)->GetDMMethod();

  size_t k = min(m_kPairs, _cc2Vec.size()); // for connecting k-closest pairs of nodes between CCs

  vector<pair<VID,VID> > kp(k);
  typename vector<pair<VID,VID> >::iterator kpIter = kp.begin();

  this->GetMPProblem()->GetNeighborhoodFinder()->KClosestPairs(
      this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod),
      _rm, 
      _cc1Vec.begin(), _cc1Vec.end(), 
      _cc2Vec.begin(), _cc2Vec.end(), 
      k, kpIter);

  // Begin the connection attempts
  VID cc1Elem, cc2Elem;
  for (size_t i = 0; i < kp.size(); ++i){

    cc1Elem = kp[i].first;
    cc2Elem = kp[i].second;

    //_stats.IncConnections_Attempted();
    if(_rm->IsCached(cc1Elem, cc2Elem) && !_rm->GetCache(cc1Elem, cc2Elem)) {
      continue;
    }
    CfgType _col;
    if (!_rm->m_pRoadmap->IsEdge(cc1Elem,cc2Elem) 
        && this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpMethod)->
        IsConnected( _rm->GetEnvironment(),_stats,dm,
          pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(cc1Elem),
          pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(cc2Elem),
          _col, &lpOutput, this->m_connectionPosRes, 
          this->m_connectionOriRes, !this->m_addAllEdges) ) {
      pMap->AddEdge(cc1Elem, cc2Elem, lpOutput.edge);
      return;
    }
    else if(this->m_addPartialEdge) {
      typename vector<pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > >::iterator I;
      for(I=lpOutput.savedEdge.begin(); I!=lpOutput.savedEdge.end(); I++) {
        CFG tmp = I->first.second;
        if(!tmp.AlmostEqual(pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(cc1Elem))){
          VID tmpVID = pMap->AddVertex(tmp);
          pMap->AddEdge(cc1Elem, tmpVID, I->second);
        }
      }
    }
    if(_col != CfgType()){
      *_collision++ = _col;
    }
  }
}

#endif

