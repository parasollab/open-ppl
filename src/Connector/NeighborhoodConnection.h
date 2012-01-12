#ifndef NeighborhoodConnection_h
#define NeighborhoodConnection_h

#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "MetricUtils.h"
#include "GraphAlgo.h"
#include "NeighborhoodFinder.h"
#include "MPStrategy.h"
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
class NeighborhoodConnection: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  typedef typename RoadmapGraph<CFG, WEIGHT>::CVI CVI;
  typedef typename RoadmapGraph<CFG, WEIGHT>::VI VI;
  
  
  //////////////////////
  // Constructors and Destructor
  NeighborhoodConnection(string _nf, int k = KCLOSEST, int m = MFAILURE, bool _count_failures = false, bool _unconnected = false, bool _random = false);
  NeighborhoodConnection(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~NeighborhoodConnection();
 
  //////////////////////
  // Access
  void SetDefault();

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


  //
  // these methods are from Closest.h
  //
  
  // operates over all nodes in a roadmap
  template <typename OutputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
        bool addPartialEdge, bool addAllEdges, OutputIterator collision) ;

  template<typename InputIterator, typename OutputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last, OutputIterator collision) ;


  template<typename InputIterator, typename OutputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last,
        InputIterator _itr2_first, InputIterator _itr2_last, OutputIterator collision) ;
  
  template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
  void pConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
        bool addPartialEdge, bool addAllEdges,
        InputIterator1 _itr1_first, InputIterator1 _itr1_last,
        InputIterator2 _itr2_first, InputIterator2 _itr2_last, OutputIterator collision) ;
    
   template<typename OutputIterator>
  void ConnectNeighbors(
        Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats, 
        bool addAllEdges, 
        int &iter_success, int &iter_failure,
        int &total_success, int &total_failure,
        VID _vid, vector<VID>& closest, OutputIterator collision) ;
  
  template<typename OutputIterator>
  void pConnectNeighbors(
        Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats, 
        bool addAllEdges, 
        int &iter_success, int &iter_failure,
        int &total_success, int &total_failure,
        VID _vid, vector<VID>& closest, CFG _cfg, OutputIterator collision) ;

  template <typename InputIterator, typename OutputIterator>
  OutputIterator FindKNeighbors(
        Roadmap<CFG, WEIGHT>* _rm, CFG cfg, 
        InputIterator _itr2_first, InputIterator _itr2_last, 
        int k, const vector<VID>& iter_neighbors, OutputIterator closest_iter) ;
            
 private:
     
  //////////////////////
  // Data

  int m_k;
  int m_fail;
  bool m_count_failures;
  bool m_unconnected;
  bool m_random;
  string m_nf;
  string m_lp;
};


template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::NeighborhoodConnection(string _nf, int k, int m, bool _count_failures, bool _unconnected, bool _random) : 
    NodeConnectionMethod<CFG,WEIGHT>(), 
    m_k(k), m_fail(m), m_count_failures(_count_failures), m_unconnected(_unconnected), m_random(_random), m_nf(_nf)
{
  this->SetName("NeighborhoodConnection"); 
}


template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::NeighborhoodConnection(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem), 
    m_k(KCLOSEST), m_fail(MFAILURE), m_count_failures(false), m_unconnected(false), m_random(false)
{
  this->SetName("NeighborhoodConnection"); 
  ParseXML(in_Node);
}


template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::~NeighborhoodConnection() { 
}

template <class CFG, class WEIGHT>
void NeighborhoodConnection<CFG,WEIGHT>::ParseXML(XMLNodeReader& in_Node) { 
  m_nf = in_Node.stringXMLParameter("nf", true, "", "nf");
  m_lp = in_Node.stringXMLParameter("lp_method", true, "", "Local Planner");
  
  m_k = in_Node.numberXMLParameter(string("k"), true, 0, 0, 10000, string("k-value (max neighbors to find). k = 0 --> all-pairs"));

  m_count_failures = in_Node.boolXMLParameter("count_failures", false, false, "if false, ignore failure count and just attempt k; if true, attempt k neighbors until too many failures detected");
  m_fail = in_Node.numberXMLParameter(string("fail"), false, m_k, 0, 10000, string("amount of failed connections allowed before operation terminates"));
  
  m_unconnected = in_Node.boolXMLParameter(string("unconnected"), false, false, string("if true, do not count existing connections towards k"));
  
  m_random = in_Node.boolXMLParameter(string("random"), false, false, string("if true, find k random configurations from destination vector"));
 
  in_Node.warnUnrequestedAttributes();
}


template <class CFG, class WEIGHT>
void NeighborhoodConnection<CFG,WEIGHT>::SetDefault() {
  m_k = KCLOSEST;
  m_fail = MFAILURE;
  m_count_failures = false;
  m_unconnected = false;
  m_random = false;
}


template <class CFG, class WEIGHT>
void
NeighborhoodConnection<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  PrintOptions(_os);
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
NeighborhoodConnection<CFG, WEIGHT>::
PrintValues(ostream& _os){
  PrintOptions(_os);
}


template <class CFG, class WEIGHT>
void
NeighborhoodConnection<CFG, WEIGHT>::
PrintOptions(ostream& out_os){
  out_os << "    " << this->GetName() << "::  k = ";
  out_os << m_k << "  fail = " << m_fail ;
  out_os << "  count_failures = " << m_count_failures;
  out_os << "  unconnected = " << m_unconnected;
  out_os << "  random = " << m_random;
  out_os << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
NeighborhoodConnection<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = new NeighborhoodConnection<CFG,WEIGHT>(*this);
  return _copy;
}


// ConnectNodes
//    This method is a wrapper for the two-iterator function, and handles
//    the special case where we want to perform connections over all nodes
//    in the roadmap.
template <class CFG, class WEIGHT>
template<typename OutputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats, 
            bool addPartialEdge,
            bool addAllEdges, OutputIterator collision) 
{
   vector<VID> vertices;
  _rm->m_pRoadmap->GetVerticesVID(vertices);
  
  ConnectNodes(_rm, Stats,
               addPartialEdge, addAllEdges, 
        vertices.begin(), vertices.end(),
        vertices.begin(), vertices.end(), collision);
}


template <class CFG, class WEIGHT>
template<typename InputIterator, typename OutputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats,
            bool addPartialEdge,
            bool addAllEdges,
            InputIterator _itr1_first, InputIterator _itr1_last, OutputIterator collision) 
{
  vector<VID> vertices;
  _rm->m_pRoadmap->GetVerticesVID(vertices);
        
  ConnectNodes(_rm, Stats, addPartialEdge, addAllEdges, 
               _itr1_first, _itr1_last,
               vertices.begin(), vertices.end(), collision);
}


/*
 * for each node in v1 {
 *   find k closest nodes in v2
 *   attempt connection
 * }
 */
template <class CFG, class WEIGHT>
template<typename InputIterator, typename OutputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats, 
            bool addPartialEdge,
            bool addAllEdges,
            InputIterator _itr1_first, InputIterator _itr1_last,
            InputIterator _itr2_first, InputIterator _itr2_last, OutputIterator collision)
{   
    if (this->m_debug) { cout << endl; PrintOptions(cout); }
    // the vertices in this iteration are the source for the connection operation
    ClockClass KClosestClock;
    
    int total_success = 0;
    int total_failure = 0;
    
    for(InputIterator itr1 = _itr1_first; itr1 != _itr1_last; ++itr1) {

      int iter_size = _itr2_last - _itr2_first;
      // calculate the number of neighbors to retrieve at each iteration
      int k_to_find = m_k;
      if(m_count_failures) //add m_fail to attempts since they will be counted
        k_to_find += m_fail;
      if(m_k == 0) //all pairs
        k_to_find = iter_size;
      k_to_find = min(k_to_find, iter_size); //cap k_to_find at iter_size
      
      
      // find cfg pointed to by itr1
      CFG v_cfg = (*(_rm->m_pRoadmap->find_vertex(*itr1))).property();
      
      if (this->m_debug) cout << (itr1 - _itr1_first) << "\tAttempting connections: VID = " << *itr1 << "  --> " << v_cfg << endl;
      
      bool enough_connected = true;
      int iter_success = 0;
      int iter_failure = 0;
      vector<VID> iter_neighbors;
      do 
      {
        if (m_unconnected)
          k_to_find = min(2 * k_to_find, iter_size);

        if (this->m_debug) cout << "k_to_find = " << k_to_find << endl;

        KClosestClock.StartClock("kClosest");
        vector<VID> closest;
	back_insert_iterator<vector<VID> > iter_begin(closest);
        back_insert_iterator<vector<VID> > iter_end = FindKNeighbors(_rm, v_cfg, _itr2_first, _itr2_last, k_to_find, iter_neighbors, iter_begin);   
	
        //copy(closest.begin(), closest.end(), iter_end);
 
        KClosestClock.StopClock();
        if (this->m_debug)
        {
          
          copy(closest.begin(), closest.end(), ostream_iterator<VID>(cout, " "));
          
        }
        
        ConnectNeighbors(_rm, Stats, addAllEdges, iter_success, iter_failure, total_success,
                         total_failure, *itr1, closest, collision);
        enough_connected = true;
        if(iter_success < m_k)
        {
          if(m_count_failures)
            enough_connected = !(iter_failure < m_fail);
          else 
            enough_connected = false;
        }
      } while (m_unconnected && !enough_connected && k_to_find < iter_size);
    }
  
    if (this->m_debug) cout << "*** kClosest Time = " << KClosestClock.GetSeconds() << endl;
    if (this->m_debug) cout << "*** total_success = " << total_success << endl;
    if (this->m_debug) cout << "*** total_failure = " << total_failure << endl;
}


 /* This version is used in parallel PRM where iterator 1 and 2 are of different types */
template <class CFG, class WEIGHT>
template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::
pConnectNodes(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats, 
            bool addPartialEdge,
            bool addAllEdges,
            InputIterator1 _itr1_first, InputIterator1 _itr1_last,
            InputIterator2 _itr2_first, InputIterator2 _itr2_last, OutputIterator collision)
{ 
    if (this->m_debug) { cout << endl; PrintOptions(cout); }
    // the vertices in this iteration are the source for the connection operation
    ClockClass KClosestClock;
    
    int total_success = 0;
    int total_failure = 0;
    
    for(InputIterator1 itr1 = _itr1_first; itr1 != _itr1_last; ++itr1) {
	    
      int iter_size = _itr2_last - _itr2_first;
      // calculate the number of neighbors to retrieve at each iteration
      int k_to_find = m_k;
      if(m_count_failures) //add m_fail to attempts since they will be counted
        k_to_find += m_fail;
      if(m_k == 0) //all pairs
        k_to_find = iter_size;
      k_to_find = min(k_to_find, iter_size); //cap k_to_find at iter_size
      
      
      // just grab the cfg & vid from the STAPL view (p_graph iterator), find_vertex() is expensive in parallel
      CFG v_cfg = (*(itr1)).property();
      VID v_vid = (*(itr1)).descriptor();
      
      //cout << stapl::get_location_id() << "> " << " pConnectNodes vid: " << v_vid <<"cfg:" << v_cfg << endl;
      
      bool enough_connected = true;
      int iter_success = 0;
      int iter_failure = 0;
      vector<VID> iter_neighbors;
      do 
      {
        if (m_unconnected)
          k_to_find = min(2 * k_to_find, iter_size);

        if (this->m_debug) cout << "k_to_find = " << k_to_find << endl;

        KClosestClock.StartClock("kClosest");
        vector<VID> closest;
	back_insert_iterator<vector<VID> > iter_begin(closest);
        back_insert_iterator<vector<VID> > iter_end = FindKNeighbors(_rm, v_cfg, _itr2_first, _itr2_last, k_to_find, iter_neighbors, iter_begin);   
	
        KClosestClock.StopClock();
        if (this->m_debug)
        {
          copy(closest.begin(), closest.end(), ostream_iterator<VID>(cout, " "));
        }
        
        pConnectNeighbors(_rm, Stats, addAllEdges, iter_success, iter_failure, total_success,
                          total_failure, v_vid, closest,v_cfg, collision);
        enough_connected = true;
        if(iter_success < m_k)
        {
          if(m_count_failures)
            enough_connected = !(iter_failure < m_fail);
          else 
            enough_connected = false;
        }
      } while (m_unconnected && !enough_connected && k_to_find < iter_size);
    }
  
    if (this->m_debug) cout << "*** kClosest Time = " << KClosestClock.GetSeconds() << endl;
    if (this->m_debug) cout << "*** total_success = " << total_success << endl;
    if (this->m_debug) cout << "*** total_failure = " << total_failure << endl;
}

template <class CFG, class WEIGHT>
template <typename OutputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::
ConnectNeighbors(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats, 
            bool addAllEdges,
            int &iter_success, int &iter_failure, 
            int &total_success, int &total_failure,
            VID _vid, vector<VID>& closest, OutputIterator collision)
{
  shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf)->GetDMMethod();
  
  LPOutput<CFG,WEIGHT> lpOutput;
  
  int success(iter_success);
  int failure(iter_failure);
                                           
  // connect the found k-closest to the current iteration's CFG
  for(typename vector<VID>::iterator itr2 = closest.begin(); itr2!= closest.end(); ++itr2) {
    if(*itr2==INVALID_VID)
      continue;
    if (this->m_debug) cout << "\t(s,f) = (" << success << "," << failure << ")";
    if (this->m_debug) cout << " | VID = " << *itr2;
    if (this->m_debug) cout << " | dist = " << dm->Distance(_rm->GetEnvironment(), (*(_rm->m_pRoadmap->find_vertex(_vid))).property(), (*(_rm->m_pRoadmap->find_vertex(*itr2))).property());
    
    // stopping conditions
    if (m_count_failures && failure >= m_fail) {
      if (this->m_debug) cout << " | stopping... failures exceeded" << endl;
      break;
    }
    if (m_k > 0 && success >= m_k) {
      if (this->m_debug) cout << " | stopping... successes met" << endl;
      break;
    }
    
    // don't attempt an edge between the same nodes
    if (_vid == *itr2) {
      if (this->m_debug) cout << " | skipping... same nodes" << endl;
      continue;
    }
  
    // don't attempt the connection if it already failed once before
    if (_rm->IsCached(_vid,*itr2)) {
      if (!_rm->GetCache(_vid,*itr2)) {
        if (this->m_debug) cout << " | skipping... this connection already failed once";
          if (this->m_debug) cout << " | failure incremented";
          failure++;
        if (this->m_debug) cout << endl;
        continue;
      }
    }
  
    // the edge already exists
    if (_rm->m_pRoadmap->IsEdge(_vid, *itr2)) {
      // if we're not in "unconnected" mode, count this as a success
      if (this->m_debug) cout << " | edge already exists in roadmap";
      if (!m_unconnected) {
        if (this->m_debug) cout << " | success incremented";
        success++;
      }
      if (this->m_debug) cout << endl;
      continue;
    }
    
    if (this->m_CheckIfSameCC) {
      // the nodes are in the same connected component
      stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
      if (is_same_cc(*(_rm->m_pRoadmap), cmap, _vid, *itr2)) {
        // if we're not in "unconnected" mode, count this as a success
        if (this->m_debug) cout << " | nodes in the same connected component";
        if (!m_unconnected) {
          if (this->m_debug) cout << " | success incremented";
          success++;
        }
        if (this->m_debug) cout << endl;
        continue;
      }
    }

    // record the attempted connection
    Stats.IncConnections_Attempted();
  
    // attempt connection with the local planner
    CfgType _col;
    if(this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetLocalPlannerMethod(m_lp)->
         IsConnected(_rm->GetEnvironment(), Stats, dm,
                     (*(_rm->m_pRoadmap->find_vertex(_vid))).property(),
                     (*(_rm->m_pRoadmap->find_vertex(*itr2))).property(),
                     _col, &lpOutput, this->connectionPosRes, this->connectionOriRes, 
                     (!addAllEdges) ))
    {
      // if connection was made, add edge and record the successful connection
      if (this->m_debug) cout << " | connection was successful";
      _rm->m_pRoadmap->AddEdge(_vid, *itr2, lpOutput.edge);
      
      // mark the successful connection in the roadmap's cache
      if (this->m_debug) cout << " | success incremented" << endl;
      _rm->SetCache(_vid,*itr2,true);
      success++;
      Stats.IncConnections_Made();
      this->connection_attempts.push_back(make_pair(make_pair(_vid, *itr2), true));
    }
    else {
      // mark the failed connection in the roadmap's cache
      if (this->m_debug) cout << " | connection failed | failure incremented" << endl;
      _rm->SetCache(_vid,*itr2,false);
      failure++;
      this->connection_attempts.push_back(make_pair(make_pair(_vid, *itr2), false));
    }
    if(_col != CfgType())
       *collision++ = _col;
  }
  
  total_success += (success - iter_success);
  total_failure += (failure - iter_failure);
  
  iter_success = success;
  iter_failure = failure;
}

template <class CFG, class WEIGHT>
template <typename OutputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::
pConnectNeighbors(Roadmap<CFG, WEIGHT>* _rm, StatClass& Stats, 
            bool addAllEdges,
            int &iter_success, int &iter_failure, 
            int &total_success, int &total_failure,
            VID _vid, vector<VID>& closest, CFG _cfg, OutputIterator collision)
{
  shared_ptr<DistanceMetricMethod> dm =  this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf)->GetDMMethod();
  
  LPOutput<CFG,WEIGHT> lpOutput;
  
  int success(iter_success);
  int failure(iter_failure);
                                           
  // connect the found k-closest to the current iteration's CFG
  for(typename vector<VID>::iterator itr2 = closest.begin(); itr2!= closest.end(); ++itr2) {
    if(*itr2==INVALID_VID)
      continue;
    //cout << stapl::get_location_id() << "> " << "pConnectNeighbors vid: " << *itr2  << endl;
    // stopping conditions
    if (m_count_failures && failure >= m_fail) {
      if (this->m_debug) cout << " | stopping... failures exceeded" << endl;
      break;
    }
    if (m_k > 0 && success >= m_k) {
      if (this->m_debug) cout << " | stopping... successes met" << endl;
      break;
    }
    
    // don't attempt an edge between the same nodes
    if (_vid == *itr2) {
      if (this->m_debug) cout << " | skipping... same nodes" << endl;
      continue;
    }
  
    // don't attempt the connection if it already failed once before
    if (_rm->IsCached(_vid,*itr2)) {
      if (!_rm->GetCache(_vid,*itr2)) {
        if (this->m_debug) cout << " | skipping... this connection already failed once";
          if (this->m_debug) cout << " | failure incremented";
          failure++;
        if (this->m_debug) cout << endl;
        continue;
      }
    }
  
    // the edge already exists, but why are we checking in two places?
    if (_rm->m_pRoadmap->IsEdge(_vid, *itr2)) {
      // if we're not in "unconnected" mode, count this as a success
      if (this->m_debug) cout << " | edge already exists in roadmap";
      if (!m_unconnected) {
        if (this->m_debug) cout << " | success incremented";
        success++;
      }
      if (this->m_debug) cout << endl;
      continue;
    }

    // record the attempted connection
    Stats.IncConnections_Attempted();
  
    // attempt connection with the local planner
    CfgType _col;
    if(this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetLocalPlannerMethod(m_lp)->
         IsConnected(_rm->GetEnvironment(), Stats, dm,_cfg,
                     (*(_rm->m_pRoadmap->find_vertex(*itr2))).property(),
                     _col, &lpOutput, this->connectionPosRes, this->connectionOriRes, 
                     (!addAllEdges) ))
    {
      // if connection was made, add edge and record the successful connection
      //do we add the same edge twice?
      if (this->m_debug) cout << " | connection was successful";
      _rm->m_pRoadmap->add_edge_async(_vid, *itr2, lpOutput.edge.first);
      
      // mark the successful connection in the roadmap's cache
      if (this->m_debug) cout << " | success incremented" << endl;
      _rm->SetCache(_vid,*itr2,true);
      success++;
      Stats.IncConnections_Made();
      this->connection_attempts.push_back(make_pair(make_pair(_vid, *itr2), true));
    }
    else {
      // mark the failed connection in the roadmap's cache
      if (this->m_debug) cout << " | connection failed | failure incremented" << endl;
      _rm->SetCache(_vid,*itr2,false);
      failure++;
      this->connection_attempts.push_back(make_pair(make_pair(_vid, *itr2), false));
    }
    if(_col != CfgType())
       *collision++ = _col;
  }
  
  total_success += (success - iter_success);
  total_failure += (failure - iter_failure);
  
  iter_success = success;
  iter_failure = failure;
}


template <class CFG, class WEIGHT>
template <typename InputIterator, typename OutputIterator>
OutputIterator NeighborhoodConnection<CFG, WEIGHT>::
FindKNeighbors(Roadmap<CFG, WEIGHT>* _rm, CFG cfg, 
              InputIterator _itr2_first, InputIterator _itr2_last, 
              int k, const vector<VID>& iter_neighbors, OutputIterator closest_iter)
{
  #ifndef _PARALLEL 
  if (m_random) {
    // find k random (unique) neighbors
    set<int> ids(iter_neighbors.begin(), iter_neighbors.end());
    if(!m_unconnected && m_k != 0)
    {
      ids.insert(_rm->m_pRoadmap->GetVID(cfg));
      k++;
    }
    for (int i = ids.size(); i < k && i<(_itr2_last-_itr2_first); i++) {
      int id = 0;
      do {
        id = (int)(LRand()%(_itr2_last - _itr2_first));
      } while (ids.find(id) != ids.end());
      ids.insert(id);

      *closest_iter = *(_itr2_first + id);
      closest_iter++;
    }
    return closest_iter;
  }
  else {
    // find the k-closest neighbors
    NeighborhoodFinder::NeighborhoodFinderPointer nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf);
    if (_itr2_last - _itr2_first == (int)_rm->m_pRoadmap->get_num_vertices()) 
      return this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, _rm, cfg, k, closest_iter);
    else 
      return this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, _rm, _itr2_first, _itr2_last, cfg, k, closest_iter);
  } 
  #else
  // find k-closest using just brute force
  BFNF<CFG,WEIGHT>* bf_finder = new BFNF<CFG,WEIGHT>(this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf)->GetDMMethod());
  return bf_finder->KClosest(_rm, _itr2_first, _itr2_last, cfg, k, closest_iter);
  #endif
}            

#endif
