#ifndef ClosestVE_h
#define ClosestVE_h
#include "ConnectionMethod.h"
#include "LocalPlanners.h"
#include "MetricUtils.h"
#include "GraphAlgo.h"
#include "NeighborhoodFinder.h"
#include "MPStrategy.h"


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
template <class CFG>
class CfgVEType {
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
     *   -# #cfg1 = Cfg::InvalidData
     *   -# #cfg2 = Cfg::InvalidData
     *   -# #cfg2IsOnEdge = false
     */
    CfgVEType();

    /**Constructor.
     *   -# #cfg1 = m_cfg1
     *   -# #cfg2 = m_cfg2
     *   -# #cfg2IsOnEdge = false
     */
    CfgVEType(CFG& _cfg1, CFG& _cfg2);

    /**Constructor.
     *   -# #cfg1 = m_cfg1
     *   -# #cfg2 = m_cfg2
     *   -# #cfg2IsOnEdge = true
     *   -# #endpt = (_endpt1,_endpt2)
     */
    CfgVEType(CFG& _cfg1, CFG& _cfg2, CFG& _endpt1, CFG& _endpt2);

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

    CFG         m_cfg1, ///< Start of an edge
                m_cfg2; ///< End of an edge
    ///True if cfg2 is on line segment defined by #endpt.
    bool        m_cfg2IsOnEdge;
    ///Defines a line segment where cfg2 is. (Defined only if #cfg2IsOnEdge is true.)
    vector<CFG> m_endpt;

}; //End of CfgVEType


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
#define KCLOSESTVE 5 

template <class CFG, class WEIGHT>
class ClosestVE: public ConnectionMethod<CFG,WEIGHT>{ //public ConnectionMethod<CFG,WEIGHT> {
  private:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    typedef typename vector<typename RoadmapGraph<CFG,WEIGHT>::VID>::iterator VIDIT;

  public:
    //////////////////////
    // Constructors and Destructor
    ClosestVE();
    ClosestVE(XMLNodeReader& _node, MPProblem* _problem);
    ~ClosestVE();

    //////////////////////
    // I/O methods

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os);
    //////////////////////
    // Core: Connection method
    static bool VE_DIST_Compare(const pair<CfgVEType<CFG>,double> _cc1, const pair<CfgVEType<CFG>,double> _cc2);
    /**Find k pairs of closest Cfgs from a given Cfg to Cfgs in a Cfg vector
     *or on the edges in edge vector.
     *
     *This method check distances from given Cfg to every Cfg in verts
     *and distances from given Cfg to every edge in edges.
     *
     *The first k cloeset pair will be returned among all pairs.
     *
     *Following is short Alg for calculating result list:
     *   -# for every Cfg, c1, in Cfg vector
     *       -# if distance(cfg,c1)< largest distance in return list.
     *           -# then replace largest distance by this path (cfg->c1)
     *           -# sort return list.
     *       -# end if
     *   -# end for
     *   -# for every edge, e1, in edge vector
     *       -# if distance(cfg,e1)< largest distance in return list.
     *           -# then let e1_c be closest point on e1 to cfg.
     *           -# replace largest distance by this path (cfg->e1_c)
     *           -# sort return list.
     *   -# end for
     *
     *@param verts A list of Cfgs.
     *@param edges A list of edge with eight.
     *@param cfg Distances are calculated from this cfg.
     *@param k Find k pairs.
     *@param midpoint Not used currently.
     *
     *@return A (k-elemet) list of CfgVEType.
     *@see VE_DIST_Compare for sorting.
     */
    vector<CfgVEType<CFG> > FindKClosestPairs(Roadmap<CFG, WEIGHT>* _rm, 
        CFG _cfg, vector<CFG>& _verts,
        vector<pair<pair<VID,VID>,WEIGHT> >& _edges, 
        int _k, bool _midPoint);

    //@}
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

void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats);

template <typename OutputIterator>
void Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    OutputIterator _collision);

template <typename InputIterator, typename OutputIterator>
void Connect( Roadmap<CFG,WEIGHT>* rm, StatClass& _stats,
    InputIterator _itr1First, InputIterator _itr1Last,
    OutputIterator _collision);

template <typename InputIterator, typename OutputIterator>
void Connect( Roadmap<CFG,WEIGHT>* rm, StatClass& _stats,
    InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last,
    OutputIterator _collision);

private:
//////////////////////
// Data
int m_tag;                            ///< Nodes can be marked by user
int m_dupeNodes, m_dupeEdges;            ///< used for acct'ing w/ closestVE
int m_kClosest;
};


template <class CFG, class WEIGHT>
ClosestVE<CFG,WEIGHT>::ClosestVE():ConnectionMethod<CFG,WEIGHT>() { 
  this->SetName("ClosestVE"); 
  m_kClosest = KCLOSESTVE;
  m_tag = -999;                            ///< Nodes can be marked by user
  m_dupeNodes = 0;
  m_dupeEdges = 0; 
}

template <class CFG, class WEIGHT>
ClosestVE<CFG,WEIGHT>::ClosestVE(XMLNodeReader& _node, MPProblem* _problem):ConnectionMethod<CFG,WEIGHT>() { 
  this->SetName("ClosestVE"); 
  m_kClosest = KCLOSESTVE;
  m_tag = -999;                            ///< Nodes can be marked by user
  m_dupeNodes = 0;
  m_dupeEdges = 0; 
  ParseXML(_node);
}

template <class CFG, class WEIGHT>
ClosestVE<CFG,WEIGHT>::~ClosestVE() { 
}

template <typename CFG, typename WEIGHT>
void ClosestVE<CFG,WEIGHT>::ParseXML(XMLNodeReader& _node){
  // Do something...
  cout << "ClosestVE<CFG,WEIGHT>::ParseXML(..) does nothing at the moment!" << endl;
  exit(-1);
}

template <class CFG, class WEIGHT>
void
ClosestVE<CFG, WEIGHT>::
PrintOptions(ostream& _os){
  _os << "\n" << this->GetName() << " kclosest = ";
  _os << kclosest;
  _os << endl;
}


//
// used to "sort" by distance
//
template <class CFG, class WEIGHT>
bool
ClosestVE<CFG, WEIGHT>::
VE_DIST_Compare (const pair<CfgVEType<CFG>,double> _cc1, const pair<CfgVEType<CFG>,double> _cc2) {
  return (_cc1.second < _cc2.second ) ;
}


//----------------------------------------------------------------------
// Given: k, ONE Cfg and ONE vector of vertices and ONE vector of edges
// Find : find k pairs of closest cfg from "cfg" to "vector"
//----------------------------------------------------------------------
template <class CFG, class WEIGHT>
vector<CfgVEType<CFG> > 
ClosestVE<CFG, WEIGHT>::FindKClosestPairs(Roadmap<CFG, WEIGHT>* _rm,
    CFG _cfg, vector<CFG>& _verts, 
    vector< pair<pair<VID,VID>,WEIGHT> >& _edges, 
    int _k, bool _midPoint) {  

  vector<CfgVEType<CFG> > pairs;
  pair<CfgVEType<CFG>,double> tmp2;

  // if valid number of pairs requested
  if (_k<=0) return pairs;

  // initialize w/ _k elements each with huge distance...
  vector<pair<CfgVEType<CFG>,double> > kp;
  for (int i=0; i < _k; i++) {
    tmp2.first = CfgVEType<CFG>();
    tmp2.second = MAX_DIST;
    kp.push_back(tmp2);
  }

  //-- VERTICES:
  //   Note: need to keep the distances so can't just call one of 
  //         the other versions of vertices compatable FindKClosestPairs

  // now go through all kp and find closest _k
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(dm_label);
  for (int c1 = 0; c1 < _verts.size(); c1++) {
    if (_cfg != _verts[c1] ) { 
      double dist = dm->Distance(_rm->GetEnvironment(), _cfg, _verts[c1]);
      if ( dist < kp[_k-1].second) {
        tmp2.first = CfgVEType<CFG>(_cfg,_verts[c1]);
        tmp2.second = dist;
        kp[_k-1] = tmp2;
        sort (kp.begin(), kp.end(), ptr_fun(VE_DIST_Compare) );
      }
    }// if (_cfg != _verts[c1])
  }//endfor c1

  //-- EDGES:
  for (int e1 = 0; e1 < _edges.size(); e1++) {    

    pmpl_detail::GetCfg<InputIterator>(_rm->m_pRoadmap)(itr1);

    CFG endpt1 = pmpl_detail::GetCfg<InputIterator>(_rm->m_pRoadmap)(_edges[e1].first.first);
    CFG endpt2 = pmpl_detail::GetCfg<InputIterator>(_rm->m_pRoadmap)(_edges[e1].first.second);
    CFG tmp;
    tmp.ClosestPtOnLineSegment(_cfg,endpt1,endpt2);

    if (tmp != endpt1 && tmp != endpt2){
      double dist = dm->Distance(_rm->GetEnvironment(), _cfg, tmp);

      if ( dist < kp[_k-1].second) {
        tmp2.first = CfgVEType<CFG>(_cfg, tmp, endpt1, endpt2);
        tmp2.second = dist;
        kp[_k-1] = tmp2;
        sort (kp.begin(), kp.end(), ptr_fun(VE_DIST_Compare) );
      } //endif dist

    } // endif (tmp != endpt1 && tmp != endpt2)
  } //endfor e1

  // now construct vector of _k pairs to return (don't need distances...)
  CFG invalid;
  invalid.InvalidData();
  for (int p=0; p < _k && p<kp.size(); p++)
    if (kp[p].first.m_cfg1 != invalid && 
        kp[p].first.m_cfg2 != invalid)
      pairs.push_back( kp[p].first );

  return pairs;
}


// ------------------------------------------------------------------
// ClosestVE:
//
// For each cfg find the k closest CFG or PT_on_EDGE 
// and try to connect with them.
//
// ------------------------------------------------------------------

template <class CFG, class WEIGHT>
void ClosestVE<CFG,WEIGHT>::Connect( Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats){
  vector<CFG> collision;
  Connect(_rm, _stats, back_inserter(collision));
}

template <class CFG, class WEIGHT>
template <typename OutputIterator>
void ClosestVE<CFG,WEIGHT>::Connect(Roadmap<CFG, WEIGHT>* _rm, StatClass& _stats,
    OutputIterator _collision){

  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(dm_label);
  LocalPlanners<CFG,WEIGHT>* lp = this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lp);

  cout << "closestVE(k="<< kclosest <<"): "<<flush;
  if (lp->UsesPlannerOtherThan("straightline")){
    cout <<"\n\nWARNING: Skipping call to ClosestVE."
      <<  "\n         'straightline' ONLY local planner "
      <<"for which ClosestVE works.\n\n";
    return;
  }

  vector<CFG> oldV,newV,verts;
  _rm->m_pRoadmap->GetVerticesData(verts);
  // if separation of vertices into two sets is desired
  if (tag != -1) {
    // separate on tag values
    for( int iV=0;iV<verts.size();++iV ) {
      if(verts[iV].tag == tag) 
        newV.push_back(verts[iV]);
      else                         
        oldV.push_back(verts[iV]);
    }
  }
  // only one set desired
  else { oldV = newV = verts; }

  // Get edges	 
  // reserve extra space ...will update local copy for efficiency
  vector< pair<pair<VID,VID>,WEIGHT> > edges;
  //vector<GRAPH::edge_descriptor> v_ed;
  edges.reserve(_rm->m_pRoadmap->get_num_edges() + newV.size()*kclosest);
  //_rm->m_pRoadmap->GetEdges(edges); //replace it with following loop fix_lantao
  for(typename RoadmapGraph<CFG, WEIGHT>::edge_iterator ei = _rm->m_pRoadmap.edges_begin(); 
      ei != _rm->m_pRoadmap.edges_end(); ++ei){
    //all_edges_a.push_back(ei_a.property()); 
    pair<pair<VID,VID>,WEIGHT> single_edge;
    single_edge.first.first = (*ei).source();
    single_edge.first.second = (*ei).target();
    single_edge.second = pmpl_detail::GetCfg<InputIterator>(_rm->m_pRoadmap)(ei);
    edges.push_back(single_edge);
  }

  // May have to adjust user's desired k wrt what is actually possible
  int k = (int)min(kclosest, oldV.size()+edges.size());  

  ///Modified for VC
#if defined(_WIN32)
  using namespace std;
#endif

  // for each "real" cfg in roadmap
  LPOutput<CFG,WEIGHT> lpOutput;
  stapl::sequential::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
  for (typename vector<CFG>::iterator v=newV.begin();v<newV.end();++v) {
    // Find k closest cfgs in the roadmap
    bool midpt_approx_of_closestPt = false;
    vector<CfgVEType<CFG> > KP = FindKClosestPairs(_rm,
        *v, oldV, edges,
        k, midpt_approx_of_closestPt);
    // for each pair identified	
    for (typename vector<CfgVEType<CFG> >::iterator kp=KP.begin();kp<KP.end();++kp){
      cmap.reset();
      if(this->m_CheckIfSameCC && is_same_cc(*(_rm->m_pRoadmap), cmap, kp->m_cfg1,kp->m_cfg2)) continue;
      //-- if new edge is collision free
      CFG _col;
      if (!_rm->m_pRoadmap->IsEdge(kp->m_cfg1,kp->m_cfg2) 
          //          && lp->IsConnected(_rm->GetEnvironment(),_stats, cd,dm, 
        && lp->IsConnected(_rm->GetEnvironment(),_stats, dm, 
            kp->m_cfg1,kp->m_cfg2, _col,
            &lpOutput,
            this->m_connectionPosRes, this->m_connectionOriRes, 
            (!this->m_addAllEdges) )) {
          //-- may have to add a new node in "middle" of existing edge
          if ( kp->m_cfg2IsOnEdge ) {
          _rm->m_pRoadmap->AddVertex(kp->m_cfg2);

          ++dupeNodes;  // keep count of duplicated nodes
          }//endif cfg2IsOnEdge

          //-- add new edge to map
          _rm->m_pRoadmap->AddEdge(kp->m_cfg1, kp->m_cfg2, lpOutput.edge);

          //-- if did add an explicit interior node to edge(endpt0,endpt1)
          if ( kp->m_cfg2IsOnEdge ) {
          _rm->m_pRoadmap->AddEdge(kp->_endpt[0],kp->m_cfg2,lpOutput.edge);
          _rm->m_pRoadmap->AddEdge(kp->m_cfg2,kp->_endpt[1],lpOutput.edge);

          m_dupeEdges += 2;  // keep count of duplicated edges <--for what??
          }//endif cfg2IsOnEdge
            } //endif lp->IsConnected
      if(_col != CfgType()){
        *_collision++ = _col;
      }
    } //endfor kp
  } //endfor v

}


template <class CFG, class WEIGHT>
template <typename InputIterator, typename OutputIterator>
void ClosestVE<CFG,WEIGHT>::
Connect(Roadmap<CFG,WEIGHT>* rm, StatClass& _stats,
    InputIterator _itr1First, InputIterator _itr1Last,
    OutputIterator _collision){

  // Original Author: Left empty because I didn't know how to implement it...
  cout << "ClosestVE<CFG,WEIGHT>::Connect() - 1 pair InputIterator" << endl;
  cout << "*** ClosestVE for 1 pair InputIterator isn't supported. ***" << endl;
  exit(-1);

}

template <class CFG, class WEIGHT>
template <typename InputIterator, typename OutputIterator>
void ClosestVE<CFG,WEIGHT>::
Connect(Roadmap<CFG,WEIGHT>* rm, StatClass& _stats,
    InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last,
    OutputIterator _collision){

  cout << "ClosestVE<CFG,WEIGHT>::Connect() - 2 pairs InputIterator" << endl;
  cout << "*** ClosestVE for 2 pairs InputIterator isn't supported. ***" << endl;
  exit(-1);

}
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
template <class CFG>
CfgVEType<CFG>::
~CfgVEType() {
}


template <class CFG>
CfgVEType<CFG>::
CfgVEType() {
  CFG invalid;
  invalid.InvalidData();
  m_cfg1 = invalid;
  m_cfg2 = invalid;
  m_cfg2IsOnEdge = false;
}


template <class CFG>
CfgVEType<CFG>::
CfgVEType(CFG& _cfg1, CFG& _cfg2){
  m_cfg1 = m_cfg1;
  m_cfg2 = m_cfg2;
  m_cfg2IsOnEdge = false;
}


template <class CFG>
CfgVEType<CFG>::
CfgVEType(CFG& _cfg1, CFG& _cfg2, CFG& _endpt1, CFG& _endpt2){
  m_cfg1   = _cfg1;
  m_cfg2   = _cfg2;
  m_cfg2IsOnEdge = true;
  _endpt.push_back(_endpt1);
  _endpt.push_back(_endpt2);
}


#endif
