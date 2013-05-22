#ifndef CLOSESTVE_H
#define CLOSESTVE_H

#include "ConnectionMethod.h"

/**Connect nodes in map to their k closest neighbors, which
 *could be vertex or point on edges in roadmap.
 *
 *This method not only creates edges, but creates new verteices 
 *also. New vertices are always on the existing edges.
 *
 *@param info provides inforamtion other than connection, like
 *collision dection, local planner, and distance metrics.
 *@param _cn provides information for specific node connection 
 *paramters.
 *@param lp Local planner for connecting given 2 Cfgs.
 *
 *@see ClosestVE for more information.
 */

/**Cfg VE Type.
 *@todo Not well documented. I don't know what this class for.
 */
template <typename CFG, typename WEIGHT>
class CfgVEType {
  private:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

  public:

    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{

    /**Default Constructor.
    */
    CfgVEType();

    CfgVEType(VID _vid1, VID _vid2);

    CfgVEType(VID _vid1, VID _vid2, VID _endpt1, VID _endpt2, CFG _cfgOnEdge);

    ///Destructor. Do nothing.
    ~CfgVEType();

    //@}

    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Data
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    bool        m_cfg2IsOnEdge;
    CFG         m_cfgOnEdge;
    VID         m_vid1, m_vid2; // VIDs for cfgs belonging to a potential new edge
    VID         m_endpt1, m_endpt2; // VIDs for cfgs along existing edge
}; //End of CfgVEType

// ------------------------------------------------------------------
// CfgVEType is a private class used by ConnectMapNodes class
// to store information about connecting to what may either be
// another Cfg _OR_ a new Cfg generated in the "middle" of an existing
// edge in the map.  
//
// In order to keep the same structure (for sorting by distance) and 
// yet not allocate space for endpoints when they are not needed
// (ie, connecting to another existing map node--aka,Cfg), endpoints
// are stored as a vector.
// ------------------------------------------------------------------

template <typename CFG, typename WEIGHT>
CfgVEType<CFG,WEIGHT>::
CfgVEType(){
  m_vid1 = INVALID_VID;
  m_vid2 = INVALID_VID;
  m_cfgOnEdge = CFG();
  m_cfg2IsOnEdge = false;
}

template <typename CFG, typename WEIGHT>
CfgVEType<CFG,WEIGHT>::
CfgVEType(VID _vid1, VID _vid2){
  m_vid1 = _vid1;
  m_vid2 = _vid2;
  m_cfgOnEdge = CFG();
  m_cfg2IsOnEdge = false;
}

template <typename CFG, typename WEIGHT>
CfgVEType<CFG,WEIGHT>::
CfgVEType(VID _vid1, VID _vid2, VID _endpt1, VID _endpt2, CFG _cfgOnEdge){
  m_vid1 = _vid1;
  m_vid2 = _vid2;
  m_endpt1 = _endpt1;
  m_endpt2 = _endpt2;
  m_cfgOnEdge = _cfgOnEdge;
  m_cfg2IsOnEdge = true;
}

template <typename CFG, typename WEIGHT>
CfgVEType<CFG,WEIGHT>::
~CfgVEType(){
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

#define KCLOSESTVE 5 

template <typename CFG, typename WEIGHT>
class ClosestVE: public ConnectionMethod<CFG,WEIGHT>{
  private:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

  public:
    //////////////////////
    // Constructors and Destructor
    ClosestVE(string _lp = "", string _nf = "", MPProblem* _problem = NULL);
    ClosestVE(XMLNodeReader& _node, MPProblem* _problem);
    ~ClosestVE();

    //////////////////////
    // I/O methods

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os);
    //////////////////////
    // Core: Connection method
    /**Find k pairs of closest Cfgs from a given Cfg to Cfgs in a Cfg vector
     *or on the edges in edge vector.
     */
    template <typename InputIterator>
      vector<CfgVEType<CFG,WEIGHT> > FindKClosestPairs(Roadmap<CFG, WEIGHT>* _rm, 
          VID _vid,
          InputIterator _verts1,
          InputIterator _verts2,
          vector<pair<pair<VID,VID>,WEIGHT> >& _edges);

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
     *@note it is possilbe that c2 in algorithm is nor in roadmap, (i.e.
     *created from edges), c2 will be added to roadmap and relavant edges
     *will be added too.
     *
     *@param newV contains Cfgs. This method tries to connect these Cfgs to those
     *contained in oldV.
     *@param oldV contains Cfgs to be connected.
     *@see RoadmapGraph::GetEdges
     */
    //@}

template <typename ColorMap, typename InputIterator, typename OutputIterator>
void Connect( Roadmap<CFG,WEIGHT>* rm, StatClass& _stats,
    ColorMap& _cmap,
    InputIterator _oldV1, InputIterator _oldV2,
    InputIterator _newV1, InputIterator _newV2,
    OutputIterator _collision);

//////////////////////
// Data
int m_kClosest;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

template <typename CFG, typename WEIGHT>
ClosestVE<CFG,WEIGHT>::ClosestVE(string _lp, string _nf, MPProblem* _problem)
  :ConnectionMethod<CFG,WEIGHT>() { 
  this->SetName("ClosestVE"); 
  m_kClosest = KCLOSESTVE;
  this->m_lpMethod = _lp;
  this->m_nfMethod = _nf;
  this->SetMPProblem(_problem);
}

/////////////////////////////////////////////////////////////////////////////

  template <typename CFG, typename WEIGHT>
ClosestVE<CFG,WEIGHT>::ClosestVE(XMLNodeReader& _node, MPProblem* _problem)
  :ConnectionMethod<CFG,WEIGHT>(_node, _problem) { 
    this->SetName("ClosestVE"); 
    m_kClosest = KCLOSESTVE;
    ParseXML(_node);
  }

/////////////////////////////////////////////////////////////////////////////

template <typename CFG, typename WEIGHT>
ClosestVE<CFG,WEIGHT>::~ClosestVE() { 
}

/////////////////////////////////////////////////////////////////////////////

template <typename CFG, typename WEIGHT>
void 
ClosestVE<CFG,WEIGHT>::ParseXML(XMLNodeReader& _node){
  m_kClosest = _node.numberXMLParameter("kClosest", true, 5,1,1000, "K Closest Connections"); 
}

/////////////////////////////////////////////////////////////////////////////

template <typename CFG, typename WEIGHT>
void
ClosestVE<CFG, WEIGHT>::
PrintOptions(ostream& _os){
  _os << "    " << this->GetName() << "::  m_kClosest = ";
  _os << m_kClosest;
  _os << endl;
}


/////////////////////////////////////////////////////////////////////////////

//----------------------------------------------------------------------
// Given: k, ONE Cfg and ONE vector of vertices and ONE vector of edges
// Find : find k pairs of closest cfg from "cfg" to "vector"
//----------------------------------------------------------------------
template <typename CFG, typename WEIGHT>
template <typename InputIterator>
vector<CfgVEType<CFG,WEIGHT> > 
ClosestVE<CFG, WEIGHT>::FindKClosestPairs(Roadmap<CFG, WEIGHT>* _rm,
    VID _vid,
    InputIterator _verts1,
    InputIterator _verts2,
    vector< pair<pair<VID,VID>,WEIGHT> >& _edges) {  

  vector<CfgVEType<CFG,WEIGHT> > pairs;
  if (m_kClosest<=0) {
    return pairs;
  }

  typedef RoadmapGraph<CFG, WEIGHT> RoadmapGraphType;
  typedef pmpl_detail::GetVertex<RoadmapGraphType> GetVertex;
  
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetMethod(this->m_nfMethod)->GetDMMethod();
  CFG cfg = GetVertex()(_rm->m_pRoadmap, _vid);
  vector<pair<CfgVEType<CFG,WEIGHT>,double> > kp;
  pair<CfgVEType<CFG,WEIGHT>,double> tmp2;

  //-- VERTICES:
  //   Note: need to keep the distances so can't just call one of 
  //         the other versions of vertices compatable FindKClosestPairs

  // now go through all kp and find closest m_kClosest
  for (InputIterator v1 = _verts1; v1 != _verts2; v1++) {
    CFG c1 = GetVertex()(_rm->m_pRoadmap, v1);
    if (cfg != c1 ) { 
      tmp2.first = CfgVEType<CFG,WEIGHT>(_vid, *v1);
      tmp2.second = dm->Distance(_rm->GetEnvironment(), cfg, c1);
      kp.push_back(tmp2);
    }
  }

  //-- EDGES:
  for (size_t e1 = 0; e1 < _edges.size(); e1++) {    
    VID vd1 = _edges[e1].first.first;
    VID vd2 = _edges[e1].first.second;

    CFG endpt1 = GetVertex()(_rm->m_pRoadmap, vd1);
    CFG endpt2 = GetVertex()(_rm->m_pRoadmap, vd2);
    CFG tmp;
    tmp.ClosestPtOnLineSegment(cfg,endpt1,endpt2);

    if(tmp == endpt1 || tmp == endpt2){
      continue;
    }

    tmp2.first = CfgVEType<CFG,WEIGHT>(_vid, INVALID_VID, vd1, vd2, tmp);
    tmp2.second = dm->Distance(_rm->GetEnvironment(), cfg, tmp);
    kp.push_back(tmp2);
  }

  size_t k = min(m_kClosest, kp.size()-1);
  partial_sort(kp.begin(), kp.begin()+k, kp.end(), CompareSecond<CfgVEType<CFG,WEIGHT>,double>());

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

template <typename CFG, typename WEIGHT>
template <typename ColorMap, typename InputIterator, typename OutputIterator>
void 
ClosestVE<CFG,WEIGHT>::Connect(Roadmap<CFG,WEIGHT>* _rm, StatClass& _stats,
    ColorMap& _cmap,
    InputIterator _oldV1, InputIterator _oldV2,
    InputIterator _newV1, InputIterator _newV2,
    OutputIterator _collision){

  typedef RoadmapGraph<CFG, WEIGHT> RoadmapGraphType;
  typedef pmpl_detail::GetVertex<RoadmapGraphType> GetVertex;
  
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetMethod(this->m_nfMethod)->GetDMMethod();
  typename LocalPlanners<CFG, WEIGHT>::LocalPlannerPointer lp =
    this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpMethod);

  if(this->m_debug){
    cout << "closestVE(k="<< m_kClosest <<"): "<<flush;
  }

  if(this->m_lpMethod != "sl"){
    if(this->m_debug){
      cout <<"\n\nWARNING: Skipping call to ClosestVE .. only 'sl' lp allowed!" << endl;
    }
    return;
  }
  
  vector< pair<pair<VID,VID>,WEIGHT> > edges;
  /*
  typename RoadmapGraph<CFG, WEIGHT>::edge_iterator ei; 
  for(ei = _rm->m_pRoadmap->edges_begin(); ei != _rm->m_pRoadmap->edges_end(); ++ei){
    pair<pair<VID,VID>,WEIGHT> single_edge;
    single_edge.first.first = (*ei).source();
    single_edge.first.second = (*ei).target();
    single_edge.second = (*ei).property();
    edges.push_back(single_edge);
  }
*/
  
   vector< VID > vids;
  _rm->m_pRoadmap->GetVerticesVID(vids);
  
  typename RoadmapGraph<CFG, WEIGHT>::vertex_iterator vi; 
  typename RoadmapGraph<CFG, WEIGHT>::adj_edge_iterator ei;
  
  for(size_t i = 0; i < vids.size(); i++) {
    vi = _rm->m_pRoadmap->find_vertex(vids[i]);
    ei = (*vi).begin();
    while (ei != (*vi).end()){
      pair<pair<VID,VID>,WEIGHT> single_edge;
      single_edge.first.first = (*ei).source();
      single_edge.first.second = (*ei).target();
      single_edge.second = (*ei).property();
      edges.push_back(single_edge);
    }
  }/**/



  // for each "real" cfg in roadmap
  LPOutput<CFG,WEIGHT> lpOutput;
  for (InputIterator v = _newV1; v != _newV2; ++v) {
    // Find k closest cfgs in the roadmap
    vector<CfgVEType<CFG,WEIGHT> > KP = FindKClosestPairs(_rm, *v, _oldV1, _oldV2, edges);

    // for each pair identified	
    typename vector<CfgVEType<CFG,WEIGHT> >::iterator kp;
    for (kp=KP.begin();kp<KP.end();++kp){
      if(kp->m_vid1 != INVALID_VID || kp->m_vid2 != INVALID_VID){
        _cmap.reset();
        if(stapl::sequential::is_same_cc(*(_rm->m_pRoadmap), _cmap, kp->m_vid1, kp->m_vid2)){
          continue;
        }
      }

      CFG cfg1 = GetVertex()(_rm->m_pRoadmap, kp->m_vid1);
      CFG cfg2;

      if ( kp->m_cfg2IsOnEdge ) {
        cfg2 = kp->m_cfgOnEdge;
      }
      else{
        cfg2 = GetVertex()(_rm->m_pRoadmap, kp->m_vid2);
      }

      bool test1 = !_rm->m_pRoadmap->IsEdge(cfg1, cfg2);
      bool test2 = lp->IsConnected(_rm->GetEnvironment(), _stats, dm, cfg1, cfg2, 
          &lpOutput, this->m_connectionPosRes, this->m_connectionOriRes, !this->m_addAllEdges);

      if(test1 && test2){
        //-- may have to add a new node in "middle" of existing edge
        if ( kp->m_cfg2IsOnEdge ) {
          kp->m_vid2 = _rm->m_pRoadmap->AddVertex(cfg2);
        }

        //-- add new edge to map
        _rm->m_pRoadmap->AddEdge(kp->m_vid1, kp->m_vid2, lpOutput.edge);

        //-- if did add an explicit interior node to edge(endpt0,endpt1)
        if ( kp->m_cfg2IsOnEdge ) {
          // Add edges from endpt nodes to new node on the edge
          _rm->m_pRoadmap->AddEdge(kp->m_endpt1, kp->m_vid2, lpOutput.edge);
          _rm->m_pRoadmap->AddEdge(kp->m_endpt2, kp->m_vid2, lpOutput.edge);

          // Remove original edge connecting endpt nodes
          _rm->m_pRoadmap->delete_edge(kp->m_endpt1, kp->m_endpt2);
          _rm->m_pRoadmap->delete_edge(kp->m_endpt2, kp->m_endpt1);
        }
      }
    } //endfor kp
  } //endfor v
}

#endif

