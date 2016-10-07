#ifndef CLOSEST_VE_H
#define CLOSEST_VE_H

#include "ConnectorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief Information on vertex or edge connection
/// @tparam MPTraits Motion planning universe
///
/// CfgVEType is a class used to store information about connecting to what may
/// be either another configuration or a new configuration along an edge of the
/// map.
///
/// In order to keep the same structure (for sorting by distance) and yet not
/// allocate space for endpoints when they are not needed (ie, connecting to
/// another existing map node - aka,Cfg), endpoints are stored as a vector.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class CfgVEType {
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;

  public:

    CfgVEType();
    CfgVEType(VID _vid1, VID _vid2);
    CfgVEType(VID _vid1, VID _vid2, VID _endpt1, VID _endpt2, CfgType _cfgOnEdge);
    virtual ~CfgVEType();

    bool        m_cfg2IsOnEdge;
    CfgType         m_cfgOnEdge;
    VID         m_vid1, m_vid2; // VIDs for cfgs belonging to a potential new edge
    VID         m_endpt1, m_endpt2; // VIDs for cfgs along existing edge
};

template<class MPTraits>
CfgVEType<MPTraits>::
CfgVEType() {
  m_vid1 = INVALID_VID;
  m_vid2 = INVALID_VID;
  m_cfgOnEdge = CfgType();
  m_cfg2IsOnEdge = false;
}

template<class MPTraits>
CfgVEType<MPTraits>::
CfgVEType(VID _vid1, VID _vid2){
  m_vid1 = _vid1;
  m_vid2 = _vid2;
  m_cfgOnEdge = CfgType();
  m_cfg2IsOnEdge = false;
}

template<class MPTraits>
CfgVEType<MPTraits>::
CfgVEType(VID _vid1, VID _vid2, VID _endpt1, VID _endpt2, CfgType _cfgOnEdge){
  m_vid1 = _vid1;
  m_vid2 = _vid2;
  m_endpt1 = _endpt1;
  m_endpt2 = _endpt2;
  m_cfgOnEdge = _cfgOnEdge;
  m_cfg2IsOnEdge = true;
}

template<class MPTraits>
CfgVEType<MPTraits>::
~CfgVEType() {}

#define KCLOSESTVE 5

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief Connect nodes to other close vertices or point on edges in roadmap.
/// @tparam MPTraits Motion planning universe
///
/// Connect nodes in map to their k closest neighbors, which could be vertex or
/// point on edges in roadmap. This method not only creates edges, but creates
/// new verteices also. New vertices are always on the existing edges.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ClosestVE: public ConnectorMethod<MPTraits> {
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;

  public:
    ClosestVE(string _lp = "", string _nf = "", MPProblemType* _problem = NULL);
    ClosestVE(MPProblemType* _problem, XMLNode& _node);
    ~ClosestVE();

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    /**Find k pairs of closest Cfgs from a given Cfg to Cfgs in a Cfg vector
     *or on the edges in edge vector.
     */
    template <typename InputIterator>
      vector<CfgVEType<MPTraits> > FindKClosestPairs(RoadmapType* _rm,
          VID _vid,
          InputIterator _verts1,
          InputIterator _verts2,
          vector<pair<pair<VID,VID>,WeightType> >& _edges);

    /**For each Cfg in newV, find the k closest Cfgs
     *(which could be Cfgs in oldV and/or points on edges of roadmap)
     *and try to connect with them.
     *
     *Algorithm:
     *   -# Get all edges in roadmap
     *   -# for every Cfg, c1, in newV
     *       -# Find k closest Cfgs (which could be Cfgs in oldV
     *          and/or points on edges of roadmap) for c1
     *       -# for each Cfg, c2, in k-closest Cfgs
     *           -# if c1 and c2 are conncteced.
     *               -# add new edge to roadmap
     *           -# end if
     *       -# end for
     *   -# end for
     *
     *it is possilbe that c2 in algorithm is nor in roadmap, (i.e.
     *created from edges), c2 will be added to roadmap and relavant edges
     *will be added too.
     *
     *newV contains Cfgs. This method tries to connect these Cfgs to those
     *contained in oldV.
     *oldV contains Cfgs to be connected.
     *RoadmapGraph::GetEdges
     */
    template<typename InputIterator1, typename InputIterator2,
      typename OutputIterator>
        void Connect(RoadmapType* rm,
            InputIterator1 _itr1First, InputIterator1 _itr1Last,
            InputIterator2 _itr2First, InputIterator2 _itr2Last,
            bool _fromFullRoadmap,
            OutputIterator _collision);

    int m_kClosest;
};

template <class MPTraits>
ClosestVE<MPTraits>::ClosestVE(string _lp, string _nf, MPProblemType* _problem) :
  ConnectorMethod<MPTraits>() {
  this->SetName("ClosestVE");
  m_kClosest = KCLOSESTVE;
  this->m_lpLabel = _lp;
  this->m_nfLabel = _nf;
  this->SetMPProblem(_problem);
}

/////////////////////////////////////////////////////////////////////////////

template <class MPTraits>
ClosestVE<MPTraits>::ClosestVE(MPProblemType* _problem, XMLNode& _node) :
  ConnectorMethod<MPTraits>(_problem, _node) {
    this->SetName("ClosestVE");
    m_kClosest = KCLOSESTVE;
    ParseXML(_node);
  }
/////////////////////////////////////////////////////////////////////////////

template <class MPTraits>
ClosestVE<MPTraits>::~ClosestVE() {
}

/////////////////////////////////////////////////////////////////////////////

template <class MPTraits>
void
ClosestVE<MPTraits>::ParseXML(XMLNode& _node) {
  m_kClosest = _node.Read("kClosest", true, 5,1,1000, "K Closest Connections");
}

/////////////////////////////////////////////////////////////////////////////

template <class MPTraits>
void
ClosestVE<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
  _os << "\tm_kClosest = " << m_kClosest << endl;
}

//----------------------------------------------------------------------
// Given: k, ONE Cfg and ONE vector of vertices and ONE vector of edges
// Find : find k pairs of closest cfg from "cfg" to "vector"
//----------------------------------------------------------------------

template <class MPTraits>
template <typename InputIterator>
vector<CfgVEType<MPTraits> >
ClosestVE<MPTraits>::FindKClosestPairs(RoadmapType* _rm,
    VID _vid,
    InputIterator _verts1,
    InputIterator _verts2,
    vector< pair<pair<VID,VID>,WeightType> >& _edges) {

  vector<CfgVEType<MPTraits> > pairs;
  if (m_kClosest<=0) {
    return pairs;
  }

  typedef typename MPProblemType::RoadmapType RoadmapType;
  typedef typename RoadmapType::GraphType GraphType;

  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel)->GetDMMethod();
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel);

  CfgType cfg = _rm->GetGraph()->GetVertex(_vid);
  vector<pair<CfgVEType<MPTraits>,double> > kp;
  pair<CfgVEType<MPTraits>,double> tmp2;

  //-- VERTICES:
  //   Note: need to keep the distances so can't just call one of
  //         the other versions of vertices compatable FindKClosestPairs

  // now go through all kp and find closest m_kClosest
  for (InputIterator v1 = _verts1; v1 != _verts2; v1++) {
    CfgType c1 = _rm->GetGraph()->GetVertex(v1);
    if (cfg != c1 ) {
      VID vid1 = _rm->GetGraph()->GetVID(v1);
      tmp2.first = CfgVEType<MPTraits>(_vid, vid1);
      tmp2.second = dm->Distance(cfg, c1);
      kp.push_back(tmp2);
    }
  }

  //-- EDGES:
  for (size_t e1 = 0; e1 < _edges.size(); e1++) {
    VID vd1 = _edges[e1].first.first;
    VID vd2 = _edges[e1].first.second;


    CfgType endpt1 = _rm->GetGraph()->GetVertex(vd1);
    CfgType endpt2 = _rm->GetGraph()->GetVertex(vd2);

    CfgType tmp;
    tmp = ClosestPtOnLineSegment(cfg,endpt1,endpt2);

    if(tmp == endpt1 || tmp == endpt2){
      continue;
    }

    tmp2.first = CfgVEType<MPTraits>(_vid, INVALID_VID, vd1, vd2, tmp);
    tmp2.second = dm->Distance(cfg, tmp);
    kp.push_back(tmp2);
  }

  int kp_size = kp.size()-1;

  size_t k = min(m_kClosest, kp_size);
  partial_sort(kp.begin(), kp.begin()+k, kp.end(), CompareSecond<CfgVEType<MPTraits>,double>());

  for (size_t p = 0; p < k; p++){
    pairs.push_back( kp[p].first );
  }

  return pairs;
}

// ------------------------------------------------------------------
// ClosestVE:
//
// For each cfg find the k closest CFG or PT_on_EDGE
// and try to connect with them.
//
// ------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////

template<class MPTraits>
template<typename InputIterator, typename InputIterator2, typename OutputIterator>
void
ClosestVE<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap,
    OutputIterator _collision){

  typedef typename MPProblemType::RoadmapType RoadmapType;
  typedef typename RoadmapType::GraphType GraphType;
  typedef typename GraphType::VI VI;

  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel)->GetDMMethod();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel);

  if(this->m_debug){
    cout << "closestVE(k="<< m_kClosest <<"): "<<flush;
  }

  if(this->m_lpLabel != "sl"){
    if(this->m_debug){
      cout <<"\n\nWARNING: Skipping call to ClosestVE .. only 'sl' lp allowed!" << endl;
    }
    return;
  }

  vector< pair<pair<VID,VID>,WeightType> > edges;

  //Get all Vertices on the Graph
   vector< VID > vids;

  for(VI vit = _rm->GetGraph()->begin(); vit!=_rm->GetGraph()->end(); ++vit){
    vids.push_back(_rm->GetGraph()->AddVertex(_rm->GetGraph()->GetVertex(vit)));
  }

  typename GraphType::vertex_iterator vi;
  typename GraphType::adj_edge_iterator ei;

  for(size_t i = 0; i < vids.size(); i++) {
    vi = _rm->GetGraph()->find_vertex(vids[i]);
    ei = (*vi).begin();
    while (ei != (*vi).end()){
      pair<pair<VID,VID>,WeightType> single_edge;
      single_edge.first.first = (*ei).source();
      single_edge.first.second = (*ei).target();
      single_edge.second = (*ei).property();
      edges.push_back(single_edge);
    }
  }

  // for each "real" cfg in roadmap
  LPOutput<MPTraits> lpOutput;
  for (InputIterator2 v = _itr2First; v != _itr2Last; ++v) {
    // Find k closest cfgs in the roadmap
    VID curVID = _rm->GetGraph()->GetVID(v);
    vector<CfgVEType<MPTraits> > KP = FindKClosestPairs(_rm, curVID, _itr1First, _itr1Last, edges);

    // for each pair identified
    typename vector<CfgVEType<MPTraits> >::iterator kp;
    for (kp=KP.begin();kp<KP.end();++kp){
      if(kp->m_vid1 != INVALID_VID || kp->m_vid2 != INVALID_VID){
        typename GraphType::ColorMap colorMap;
        if(stapl::sequential::is_same_cc(*_rm->GetGraph(), colorMap, kp->m_vid1, kp->m_vid2))
          continue;
      }

      CfgType cfg1 = _rm->GetGraph()->GetVertex(kp->m_vid1);
      CfgType cfg2;

      if ( kp->m_cfg2IsOnEdge ) {
        cfg2 = kp->m_cfgOnEdge;
      }
      else{
        cfg2 = _rm->GetGraph()->GetVertex(kp->m_vid2);
      }

      CfgType col;
      bool test1 = !_rm->GetGraph()->IsEdge(kp->m_vid1, kp->m_vid2);
      bool test2 = lp->IsConnected(cfg1, cfg2, col,
          &lpOutput, env->GetPositionRes(), env->GetOrientationRes());

      if(test1 && test2){
        //-- may have to add a new node in "middle" of existing edge
        if ( kp->m_cfg2IsOnEdge ) {
          kp->m_vid2 = _rm->GetGraph()->AddVertex(cfg2);
        }

        //-- add new edge to map
        _rm->GetGraph()->AddEdge(kp->m_vid1, kp->m_vid2, lpOutput.m_edge);

        //-- if did add an explicit interior node to edge(endpt0,endpt1)
        if ( kp->m_cfg2IsOnEdge ) {
          // Add edges from endpt nodes to new node on the edge
          _rm->GetGraph()->AddEdge(kp->m_endpt1, kp->m_vid2, lpOutput.m_edge);
          _rm->GetGraph()->AddEdge(kp->m_endpt2, kp->m_vid2, lpOutput.m_edge);

          // Remove original edge connecting endpt nodes
          _rm->GetGraph()->delete_edge(kp->m_endpt1, kp->m_endpt2);
          _rm->GetGraph()->delete_edge(kp->m_endpt2, kp->m_endpt1);
        }
      }
    } //endfor kp
  } //endfor v
}
#endif

