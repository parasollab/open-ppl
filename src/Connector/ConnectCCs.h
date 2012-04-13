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

template <typename CFG, typename WEIGHT>
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

    template <typename ColorMap>
    void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& cmap);

    template <typename OutputIterator, typename ColorMap>
      void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& cmap,
          OutputIterator _collision);

    template <typename InputIterator, typename OutputIterator, typename ColorMap>
      void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& cmap, 
          InputIterator _itr1First, InputIterator _itr1Last,
          OutputIterator _collision);

    template <typename InputIterator, typename OutputIterator, typename ColorMap>
      void Connect(Roadmap<CFG, WEIGHT>*, StatClass& _stats, ColorMap& cmap,
          InputIterator _itr1First, InputIterator _itr1Last,
          InputIterator _itr2First, InputIterator _itr2Last, 
          OutputIterator _collision);

  protected:
    template<typename OutputIterator, typename ColorMap>
      void ConnectNeighbors(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& cmap,
          vector<VID>& _cc1Vec, vector<VID>& _cc2Vec,
          OutputIterator _collision);  

    // compute all pair distance between ccs.
    // approximated using coms of ccs
    template <typename InputIterator, typename ColorMap>
      void ComputeAllPairsCCDist(Roadmap<CFG, WEIGHT>* _rm, ColorMap& cmap,
          InputIterator _ccs1First, InputIterator _ccs1Last,
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
    vector< vector<double> > m_ccDist;
};


///////////////////////////////////////////////////////////////////////////////
//   Connection Method:  ConnectCCs
template <typename CFG, typename WEIGHT>
ConnectCCs<CFG,WEIGHT>::ConnectCCs() : ConnectionMethod<CFG,WEIGHT>() { 
  this->SetName("ConnectCCs"); 
  cerr << "ConnectCCs default constructor" << endl;
  m_kPairs = KPAIRS;
  m_smallcc = SMALL_CC;
  m_k2 = K2_CLOSEST;
}

///////////////////////////////////////////////////////////////////////////////
template <typename CFG, typename WEIGHT>
ConnectCCs<CFG,WEIGHT>::ConnectCCs(XMLNodeReader& _node, MPProblem* _problem) : ConnectionMethod<CFG,WEIGHT>(_node, _problem) { 
  cerr << "ConnectCCs secondary constructor" << endl;
  ParseXML(_node);
}

///////////////////////////////////////////////////////////////////////////////
template <typename CFG, typename WEIGHT>
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
template <typename CFG, typename WEIGHT>
ConnectCCs<CFG,WEIGHT>::~ConnectCCs(){}

///////////////////////////////////////////////////////////////////////////////
template <typename CFG, typename WEIGHT>
void
ConnectCCs<CFG, WEIGHT>::PrintOptions(ostream& _os){
  ConnectionMethod<CFG,WEIGHT>::PrintOptions(_os);
  _os << "    " << this->GetName() << "::  m_kPairs = ";
  _os << m_kPairs << "  m_smallcc = " << m_smallcc ;
  _os << endl;
}

///////////////////////////////////////////////////////////////////////////////
template <typename CFG, typename WEIGHT>
template <typename InputIterator, typename ColorMap>
void ConnectCCs<CFG,WEIGHT>::ComputeAllPairsCCDist(Roadmap<CFG, WEIGHT>* _rm, ColorMap& cmap,
    InputIterator _ccs1First, InputIterator _ccs1Last,
    InputIterator _ccs2First, InputIterator _ccs2Last){

  shared_ptr<DistanceMetricMethod> dm = 
    this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod)->GetDMMethod();

  RoadmapGraph<CFG,WEIGHT>* rmapG=_rm->m_pRoadmap;
  Environment* env=_rm->GetEnvironment();
  cmap.reset();

  //compute com of ccs
  vector<CFG> com1,com2;
  vector<VID> ccvids;
  for (InputIterator i = _ccs1First; i != _ccs1Last; i++){
    cmap.reset();
    ccvids.clear();
    get_cc(*rmapG, cmap, *i, ccvids);
    com1.push_back(GetCentroid(rmapG, ccvids));
  }
  for (InputIterator i = _ccs2First; i != _ccs2Last; i++){
    cmap.reset();
    ccvids.clear();
    get_cc(*rmapG, cmap, *i, ccvids);
    com2.push_back(GetCentroid(rmapG, ccvids));
  }

  //dist between ccs
  m_ccDist.clear();

  typedef typename vector<CFG>::iterator IT;
  for(IT i = com1.begin(); i != com1.end(); ++i){
    m_ccDist.push_back(vector<double>());
    int id1 = i - com1.begin();
    for(IT j = com2.begin(); j != com2.end(); ++j){
      int id2=j-com2.begin();
      if (*(_ccs1First + id1) != *(_ccs2First + id2)){
        m_ccDist.back().push_back(dm->Distance(env,*i,*j));
      }
      else{ //same cc
        m_ccDist.back().push_back(1e20);
      }
    }
  }//end for i
}

///////////////////////////////////////////////////////////////////////////////
//get m_k2 closest pairs of CCs
template <typename CFG, typename WEIGHT>
void ConnectCCs<CFG,WEIGHT>::GetK2Pairs(int _ccid, vector<VID>& _k2CCID){
  typedef pair<double,int> CCD; //dist to cc
  typedef vector<double>::iterator IT;

  vector<double>& dis2CCs = m_ccDist[_ccid];
  vector<CCD> _ccids;
  _ccids.reserve(m_k2);

  // go through each cc
  for(IT it = dis2CCs.begin(); it != dis2CCs.end(); it++){
    int id = it - dis2CCs.begin();
    if(_ccids.size() < m_k2){ //not yet enough
      _ccids.push_back(CCD(*it,id));
      push_heap(_ccids.begin(),_ccids.end());
    } 
    else { //start to check
      pop_heap(_ccids.begin(),_ccids.end());
      CCD& back=_ccids.back();
      if(back.first>*it){
        back.first=*it;
        back.second=id;
      }  
      push_heap(_ccids.begin(),_ccids.end());
    }
  }//end for it

  //copy
  _k2CCID.reserve(m_k2);
  for(vector<CCD>::iterator i=_ccids.begin();i!=_ccids.end();++i){
    _k2CCID.push_back(i->second);
  }
}


///////////////////////////////////////////////////////////////////////////////
// Find the closest inter CC distance
template <typename CFG, typename WEIGHT>
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
template <typename CFG, typename WEIGHT>
template <typename ColorMap>
void ConnectCCs<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& cmap){
  vector<CFG> collision;
  Connect(_rm, _stats, cmap, back_inserter(collision));
}

///////////////////////////////////////////////////////////////////////////////
template <typename CFG, typename WEIGHT>
template <typename OutputIterator, typename ColorMap>
void ConnectCCs<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& cmap, 
    OutputIterator _collision) {

  if(this->m_debug){
    cout << "components(m_kPairs="<< m_kPairs ;
    cout << ", m_smallcc="<<m_smallcc <<"): "<<flush;
  }

  cmap.reset();
  vector< pair<size_t,VID> > ccs1;
  get_cc_stats(*(_rm->m_pRoadmap),cmap,ccs1);

  vector<VID> ccs(ccs1.size());

  for(size_t i = 0; i < ccs1.size(); ++i){
    ccs[i]=ccs1[i].second;
  }

  if(ccs1.size() > 1){
    Connect(_rm, _stats, cmap, ccs.rbegin(), ccs.rend(), _collision);
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename CFG, typename WEIGHT>
template <typename InputIterator, typename OutputIterator, typename ColorMap>
void ConnectCCs<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& cmap, 
    InputIterator _itr1First, InputIterator _itr1Last,
    OutputIterator _collision) {

  if(this->m_debug){
    cout << "components(m_kPairs="<< m_kPairs ;
    cout << ", m_smallcc="<<m_smallcc <<"): "<<flush;
  }

  if(_itr1Last - _itr1First > 1){
    Connect(_rm, _stats, cmap, _itr1First, _itr1Last, _itr1First+1, _itr1Last, _collision);
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator, typename ColorMap>
void ConnectCCs<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats, ColorMap& cmap, 
    InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last, 
    OutputIterator _collision) {

  if(this->m_debug){
    cout << "components(m_kPairs="<< m_kPairs ;
    cout << ", m_smallcc="<<m_smallcc <<"): "<<flush;
  }

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  cmap.reset();

  if(_itr1Last - _itr1First <= 1) return;

  if(m_k2 != 0){ // Attempt to Connect K2-Closest CCs
    vector< vector<VID> > ccset1, ccset2;

    for(int j = 0; j < 2; ++j){
      InputIterator vidsFirst = (j==0) ? _itr1First : _itr2First;
      InputIterator vidsLast = (j==0) ? _itr1Last : _itr2Last;
      vector< vector<VID> >& ccset = (j==0) ? ccset1 : ccset2;
      for(InputIterator i = vidsFirst; i != vidsLast; ++i){
        ccset.push_back(vector<VID>());  
        cmap.reset();
        get_cc(*pMap, cmap, *i, ccset.back());
      }//end for i
    }//end for j

    ComputeAllPairsCCDist(_rm, cmap, _itr1First, _itr1Last, _itr2First, _itr2Last);

    for(InputIterator itr1 = _itr1Last; itr1 != _itr1First; --itr1) {
      int id = itr1 - _itr1First - 1;
      vector<VID> k2CCID;
      GetK2Pairs(id,k2CCID);
      for(VIDIT v2=k2CCID.begin();v2!=k2CCID.end();v2++){
        ConnectNeighbors(_rm, _stats, cmap, ccset1[id], ccset2[*v2], _collision);
      }
    }/*endfor V1*/
  } 

  else { // Attempt to Connect CCs, All-Pairs for both CCs
    // process components from smallest to biggest  
    for (InputIterator itr1 = _itr1First; itr1 != _itr1Last; ++itr1) {
      for (InputIterator itr2 = _itr2First; itr2 != _itr2Last; ++itr2) {
        cmap.reset();
        // if V1 & V2 not already connected, try to connect them 
        if ( !stapl::sequential::is_same_cc(*pMap,cmap,*itr1,*itr2) ) {
          vector<VID> cc1,cc2;

          cmap.reset();
          get_cc(*pMap,cmap,*itr1,cc1);

          cmap.reset();
          get_cc(*pMap,cmap,*itr2,cc2);

          if(cc1.size() <= cc2.size())
            ConnectNeighbors(_rm, _stats, cmap, cc1, cc2, _collision);
          else
            ConnectNeighbors(_rm, _stats, cmap, cc2, cc1, _collision);
        } 
      }/*endfor V2*/ 
    }/*endfor V1*/
  }
  _stats.ComputeIntraCCFeatures(_rm,this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod)->GetDMMethod());
}

///////////////////////////////////////////////////////////////////////////////
template <typename CFG, typename WEIGHT>
template <typename OutputIterator, typename ColorMap>
void ConnectCCs<CFG, WEIGHT>::ConnectNeighbors( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    ColorMap& cmap,
    vector<VID>& _cc1Vec, 
    vector<VID>& _cc2Vec,
    OutputIterator _collision) {

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  LPOutput<CFG, WEIGHT> lpOutput;
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod)->GetDMMethod();

  size_t cc1Size = _cc1Vec.size();
  size_t cc2Size = _cc2Vec.size();
  size_t k = min(m_kPairs, cc2Size); // for connecting k-closest pairs of nodes between CCs
  bool smallCCs = ( (cc1Size < m_smallcc) && (cc2Size < m_smallcc) );

  vector<pair<VID,VID> > kp(k);
  typename vector<pair<VID,VID> >::iterator kpIter = kp.begin();

  VID cc1Elem, cc2Elem;
  size_t count1, count2;

  if(smallCCs) { // If Connecting Small CCs, attempt an all-pairs connection between nodes in both CCs
    count1 = cc1Size;
    count2 = cc2Size;
  } 
  else { // Else, find the k-nearest nodes in both CCs and attempt connections between them
    this->GetMPProblem()->GetNeighborhoodFinder()->KClosestPairs(
        this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(this->m_nfMethod),
        _rm, 
        _cc1Vec.begin(), _cc1Vec.end(), 
        _cc2Vec.begin(), _cc2Vec.end(), 
        k, kpIter);
    count1 = count2 = k;
  }

  // Begin the connection attempts
  for (size_t i = 0; i != count1; ++i){
    for (size_t j = 0; j != count2; ++j){
      if(smallCCs){ // grab the appropriate nodes if we are doing all-pairs between nodes
        cc1Elem = _cc1Vec[i];
        cc2Elem = _cc2Vec[j];
      }
      else{ // Only attempting to connect k-nearest pairs of nodes, this handles double forloop
        j = i;
        cc1Elem = kp[i].first;
        cc2Elem = kp[j].second;
      }
      //_stats.IncConnections_Attempted();
      if(_rm->IsCached(cc1Elem, cc2Elem) && !_rm->GetCache(cc1Elem, cc2Elem)) {
        if(!smallCCs) {break;}
        else {continue;}
      }
      CfgType _col;
      if (!_rm->m_pRoadmap->IsEdge(cc1Elem,cc2Elem) 
          && this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpMethod)->
          IsConnected( _rm->GetEnvironment(),_stats,dm,
            pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(cc1Elem),
            pmpl_detail::GetCfg<VID>(_rm->m_pRoadmap)(cc2Elem),
            _col, &lpOutput, this->m_connectionPosRes, 
            this->m_connectionOriRes, !this->m_addAllEdges) ) {
        //_stats.IncConnections_Made();
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

      // Recall, if not smallCCs, we are attempting connections between k-nearest.
      // Hence, we must exit the inner loop every time we reach the end.
      if(!smallCCs) {
        break;
      }

    }//end for c2
  }//end for c1
}

#endif

