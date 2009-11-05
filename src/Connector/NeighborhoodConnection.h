#ifndef NeighborhoodConnection_h
#define NeighborhoodConnection_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "Clock_Class.h"
#include "GraphAlgo.h"
#include "NeighborhoodFinder.h"

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
  NeighborhoodConnection();
  NeighborhoodConnection(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  NeighborhoodConnection(int k); // constructor for "Closest"
  NeighborhoodConnection(int k, int m); // constructor for "ClosestSF"
  // here, we should have an additional constructor for each "mode" that we will use
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
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges) ;

  template<typename InputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last) ;


  template<typename InputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last,
        InputIterator _itr2_first, InputIterator _itr2_last) ;
    
  void ConnectNeighbors(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
        DistanceMetric * dm,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addAllEdges, 
        int &iter_success, int &iter_failure,
        int &total_success, int &total_failure,
        VID _vid, vector<VID> closest) ;

  template <typename InputIterator, typename OutputIterator>
  void FindKNeighbors(
        Roadmap<CFG, WEIGHT>* _rm, CFG cfg, 
        InputIterator _itr2_first, InputIterator _itr2_last, 
        int k, OutputIterator closest_iter) ;

  template <typename OutputIterator>
  void FindKNeighbors(
        Roadmap<CFG, WEIGHT>* _rm, CFG cfg, int k, OutputIterator closest_iter) ;
            
 private:
     
  //////////////////////
  // Data

  int m_k;
  int m_fail;
  bool m_unconnected;
  bool m_random;
  bool m_debug;
  
  string m_nf;
};


template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::NeighborhoodConnection():NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "NeighborhoodConnection"; 
  SetDefault();
}

template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::NeighborhoodConnection(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem) { 
  LOG_DEBUG_MSG("NeighborhoodConnection::NeighborhoodConnection()"); 
  this->element_name = "NeighborhoodConnection"; 
  SetDefault();
  ParseXML(in_Node);
  
  
  LOG_DEBUG_MSG("~NeighborhoodConnection::NeighborhoodConnection()"); 
}


//this is backward support for function call from other class
//to be cleaned
template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::NeighborhoodConnection(int k) : 
    NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "NeighborhoodConnection"; 
  m_k = k;
  m_fail = k;
  m_unconnected = false;
  m_random = false;
  m_debug = false;
}

template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::NeighborhoodConnection(int k, int m) : 
    NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "NeighborhoodConnection"; 
  m_k = k;
  m_fail = m;
  m_unconnected = false;
  m_random = false;
  m_debug = false;
}


template <class CFG, class WEIGHT>
NeighborhoodConnection<CFG,WEIGHT>::~NeighborhoodConnection() { 
}

template <class CFG, class WEIGHT>
void NeighborhoodConnection<CFG,WEIGHT>::ParseXML(XMLNodeReader& in_Node) { 
 
  m_nf = in_Node.stringXMLParameter("nf", true, "", "nf");
  
  m_k = in_Node.numberXMLParameter(string("k"), true, 0, 0, 10000, 
                                 string("k-value (max neighbors to find). k = 0 --> all-pairs"));
  m_fail = in_Node.numberXMLParameter(string("fail"), false, 0, 0, 10000, 
                                    string("amount of failed connections allowed before operation terminates"));
  m_unconnected = in_Node.boolXMLParameter(string("unconnected"), false, false,
                                         string("if true, do not count existing connections towards k"));
  m_random = in_Node.boolXMLParameter(string("random"), false, false,
                                         string("if true, find k random configurations from destination vector"));
  m_debug = in_Node.boolXMLParameter(string("debug"), false, false, string(""));
  
  
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "Shell") {
      //m_shells.push_back(new Shell<CFG, WEIGHT>(*citr, GetMPProblem()));
    } else {
      citr->warnUnknownNode();
    }
  }                                
  
}



template <class CFG, class WEIGHT>
void NeighborhoodConnection<CFG,WEIGHT>::SetDefault() {
  m_k = KCLOSEST;
  m_fail = MFAILURE;
}


template <class CFG, class WEIGHT>
void
NeighborhoodConnection<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << this->GetName() << " ";
  _os << "\tINTEGER INTEGER (default k:" << KCLOSEST << ", mfailure:" << MFAILURE << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
NeighborhoodConnection<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << this->GetName() << " k = ";
  _os << m_k << " fail = " << m_fail;
  _os << endl;
}

template <class CFG, class WEIGHT>
void
NeighborhoodConnection<CFG, WEIGHT>::
PrintOptions(ostream& out_os){
  out_os << "    " << this->GetName() << "::  k = ";
  out_os << m_k << "  fail = " << m_fail ;
  out_os << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
NeighborhoodConnection<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new NeighborhoodConnection<CFG,WEIGHT>(*this);
  return _copy;
}


// ConnectNodes
//    This method is a wrapper for the two-iterator function, and handles
//    the special case where we want to perform connections over all nodes
//    in the roadmap.
template <class CFG, class WEIGHT>
void NeighborhoodConnection<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  //cout << "NeighborhoodConnection<CFG,WEIGHT>::ConnectNodes() - Roadmap Only" << endl << flush;
  //cout << "Connecting CCs with method: closest k="<< k << endl;
#ifndef QUIET
  cout << "closest(k="<< m_k <<"): "<<flush;
#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<VID> vertices;
  pMap->GetVerticesVID(vertices);
  
  ConnectNodes(_rm, Stats, dm, lp, addPartialEdge, addAllEdges, 
        vertices.begin(), vertices.end());
}


template <class CFG, class WEIGHT>
template<typename InputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            InputIterator _itr1_first, InputIterator _itr1_last) 
{
  //cout << "NeighborhoodConnection<CFG,WEIGHT>::ConnectNodes() - 1 pairs InputIterator" << endl << flush;

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<VID> rmp_vertices;
  pMap->GetVerticesVID(rmp_vertices);
        
  // the vertices in this iteration are the source for the connection operation
  Clock_Class KClosestClock;
  
  int total_success = 0;
  int total_failure = 0;
  
  for(InputIterator itr1 = _itr1_first; itr1 != _itr1_last; ++itr1) {
    
    int iter_success = 0;
    int iter_failure = 0;

    // calculate the number of neighbors to retrieve at each iteration
    int k_to_find;
    int iter_size = rmp_vertices.end() - rmp_vertices.begin();

    if (m_k > 0) {
      k_to_find = m_k + m_fail;  // m_fail defaults to 0
      if (m_fail == 0) m_fail = 5;  // for consistency with prior defaults of m_fail = 5
    }
    else if (m_unconnected) {
      k_to_find =  m_k;  // this will be increased if we don't make enough connections 
    }
    else {
      k_to_find = iter_size;    // all pairs
    }

    // 1) get CFG pointed to by the iterator
    CFG v_cfg = _rm->m_pRoadmap->find_vertex(*itr1).property();
    
    if (m_debug) cout << (itr1 - _itr1_first) << "\tAttempting connections: VID = " << *itr1 << "  --> " << v_cfg << endl;
    
    do 
    {
    	if (m_unconnected) {      
    	  if (m_debug) cout << "k_to_find = " << k_to_find << " | iter_success = " << iter_success << endl;
        
        k_to_find *= 2;  // fix this to be more elegant in the future
        if (k_to_find > iter_size)
    	    k_to_find = iter_size;
    	}
    	
      // 2) storage to pass to the neighborhood finder for k-closest vids
      vector<VID> closest(k_to_find);
      typename vector<VID>::iterator closest_iter = closest.begin();

      // 3) find k-closest to the current iteration's CFG      
      KClosestClock.StartClock("kClosest");
      FindKNeighbors(_rm, v_cfg, rmp_vertices.begin(), rmp_vertices.end(), k_to_find, closest_iter);      
      KClosestClock.StopClock();
    
      ConnectNeighbors(_rm, Stats, dm, lp, addAllEdges, iter_success, iter_failure, 
              total_success, total_failure, *itr1, closest);
    } while (m_unconnected && iter_success < k_to_find && iter_failure < m_fail && k_to_find < iter_size);
  }

  if (m_debug) cout << "*** kClosest Time = " << KClosestClock.GetClock_SEC() << endl;
  if (m_debug) cout << "*** total_success = " << total_success << endl;
  if (m_debug) cout << "*** total_failure = " << total_failure << endl;
}


/*
 * for each node in v1 {
 *   find k closest nodes in v2
 *   attempt connection
 * }
 */
template <class CFG, class WEIGHT>
template<typename InputIterator>
void NeighborhoodConnection<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            InputIterator _itr1_first, InputIterator _itr1_last,
            InputIterator _itr2_first, InputIterator _itr2_last)
{   
    //cout << "NeighborhoodConnection<CFG,WEIGHT>::ConnectNodes() - 2 pairs InputIterator" << endl << flush;
  
    // the vertices in this iteration are the source for the connection operation
    Clock_Class KClosestClock;
    
    int total_success = 0;
    int total_failure = 0;
    
    for(InputIterator itr1 = _itr1_first; itr1 != _itr1_last; ++itr1) {
      
      int iter_success = 0;
      int iter_failure = 0;

      // calculate the number of neighbors to retrieve at each iteration
      int k_to_find;
      int iter_size = _itr2_last - _itr2_first;

      if (m_k > 0) {
        k_to_find = m_k + m_fail;  // m_fail defaults to 0
        if (m_fail == 0) m_fail = 5;  // for consistency with prior defaults of m_fail = 5
      }
      else if (m_unconnected) {
        k_to_find =  m_k;  // this will be increased if we don't make enough connections 
      }
      else {
        k_to_find = iter_size;    // all pairs
      }

      // 1) get CFG pointed to by the iterator
      CFG v_cfg = _rm->m_pRoadmap->find_vertex(*itr1).property();
      
      if (m_debug) cout << (itr1 - _itr1_first) << "\tAttempting connections: VID = " << *itr1 << "  --> " << v_cfg << endl;
      
      do 
      {
      	if (m_unconnected) {      
      	  if (m_debug) cout << "k_to_find = " << k_to_find << " | iter_success = " << iter_success << endl;
          
          k_to_find *= 2;  // fix this to be more elegant in the future
          if (k_to_find > iter_size)
      	    k_to_find = iter_size;
      	}
      	
        // 2) storage to pass to the neighborhood finder for k-closest vids
        vector<VID> closest(k_to_find);
        typename vector<VID>::iterator closest_iter = closest.begin();

        // 3) find k-closest to the current iteration's CFG      
        KClosestClock.StartClock("kClosest");
        FindKNeighbors(_rm, v_cfg, _itr2_first, _itr2_last, k_to_find, closest_iter);      
        KClosestClock.StopClock();
      
        ConnectNeighbors(_rm, Stats, dm, lp, addAllEdges, iter_success, iter_failure, 
                total_success, total_failure, *itr1, closest);
      } while (m_unconnected && iter_success < k_to_find && iter_failure < m_fail && k_to_find < iter_size);
    }
  
    if (m_debug) cout << "*** kClosest Time = " << KClosestClock.GetClock_SEC() << endl;
    if (m_debug) cout << "*** total_success = " << total_success << endl;
    if (m_debug) cout << "*** total_failure = " << total_failure << endl;
}

template <class CFG, class WEIGHT>
void NeighborhoodConnection<CFG,WEIGHT>::
ConnectNeighbors(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            DistanceMetric * dm,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addAllEdges,
            int &iter_success, int &iter_failure, 
            int &total_success, int &total_failure,
            VID _vid, vector<VID> closest)
{ 
  LPOutput<CFG,WEIGHT> lpOutput;
  
  int success = iter_success;
  int failure = iter_failure;
                                           
  // connect the found k-closest to the current iteration's CFG
  for(typename vector<VID>::iterator itr2 = closest.begin(); itr2!= closest.end(); ++itr2) {
    
    if (m_debug) cout << "\t(s,f) = (" << success << "," << failure << ")";
    if (m_debug) cout << " | VID = " << *itr2;
    if (m_debug) cout << " | dist = " << dm->Distance(
            _rm->GetEnvironment(), _rm->m_pRoadmap->find_vertex(_vid).property(), _rm->m_pRoadmap->find_vertex(*itr2).property()
            );
    
    // stopping conditions
    if (m_fail > 0 && failure >= m_fail) {
      if (m_debug) cout << " | stopping... failures exceeded" << endl;
      break;
    }
    if (m_k > 0 && success >= m_k) {
      if (m_debug) cout << " | stopping... successes met" << endl;
      break;
    }
    
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
          failure++;
        }
        if (m_debug) cout << endl;
        continue;
      }
    }
  
    // record the attempted connection
    Stats.IncConnections_Attempted();
  
    // the edge already exists
    if (_rm->m_pRoadmap->IsEdge(_vid, *itr2)) {
      // if we're not in "unconnected" mode, count this as a success
      if (m_debug) cout << " | edge already exists in roadmap";
      if (!m_unconnected) {
        if (m_debug) cout << " | success incremented";
        success++;
      }
      if (m_debug) cout << endl;
      continue;
    }
    
    if (this->m_CheckIfSameCC) {
      // the nodes are in the same connected component
      stapl::vector_property_map< stapl::stapl_color<size_t> > cmap;
      if (is_same_cc(*(_rm->m_pRoadmap), cmap, _vid, *itr2)) {
        // if we're not in "unconnected" mode, count this as a success
        if (m_debug) cout << " | nodes in the same connected component";
        if (!m_unconnected) {
          if (m_debug) cout << " | success incremented";
          success++;
        }
        if (m_debug) cout << endl;
        continue;
      }
    }

  
    // attempt connection with the local planner
    if(lp->IsConnected(_rm->GetEnvironment(), Stats, dm,
                _rm->m_pRoadmap->find_vertex(_vid).property(),
                _rm->m_pRoadmap->find_vertex(*itr2).property(),
                &lpOutput, this->connectionPosRes, this->connectionOriRes, 
                (!addAllEdges) ))
    {
      // if connection was made, add edge and record the successful connection
      if (m_debug) cout << " | connection was successful";
      _rm->m_pRoadmap->AddEdge(_vid, *itr2, lpOutput.edge);
      Stats.IncConnections_Made();
      
      // mark the successful connection in the roadmap's cache
      if (m_debug) cout << " | success incremented" << endl;
      _rm->SetCache(_vid,*itr2,true);
      success++;
    }
    else {
      // mark the failed connection in the roadmap's cache
      if (m_debug) cout << " | connection failed | failure incremented" << endl;
      _rm->SetCache(_vid,*itr2,false);
      failure++;
    }
  }
  
  iter_success += success;
  iter_failure += failure;
  
  total_success += success;
  total_failure += failure;
}


template <class CFG, class WEIGHT>
template <typename InputIterator, typename OutputIterator>
void NeighborhoodConnection<CFG, WEIGHT>::
FindKNeighbors(Roadmap<CFG, WEIGHT>* _rm, CFG cfg, 
              InputIterator _itr2_first, InputIterator _itr2_last, 
              int k, OutputIterator closest_iter)
{
  if (k == 0) {
    // we want an all-pairs comparison
    for(InputIterator itr = _itr2_first; itr != _itr2_last; ++itr) {
      *closest_iter = *itr;
      closest_iter++;
    }
  }
  else {
    
    if (m_random) {
      // find k random neighbors
      for (int i = 0; i < k; i++) {
        int id = (int)(OBPRM_lrand()%(_itr2_last - _itr2_first));
        *closest_iter = *(_itr2_first + id);
        closest_iter++;
      }
    }
    else {
      // find the k-closest neighbors
      NeighborhoodFinder::NeighborhoodFinderPointer nfptr;
      nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf);
      
      if (_itr2_last - _itr2_first == _rm->m_pRoadmap->get_num_vertices()) {
        this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, _rm, cfg, k, closest_iter);
      }
      else {
        this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, _rm, _itr2_first, 
                                                  _itr2_last, cfg, k, closest_iter);
      }
    }                                           
  }
}              

template <class CFG, class WEIGHT>
template <typename OutputIterator>
void NeighborhoodConnection<CFG, WEIGHT>::
FindKNeighbors(Roadmap<CFG, WEIGHT>* _rm, CFG cfg, 
              int k, OutputIterator closest_iter)
{
  if (k == 0) {
    // we want an all-pairs comparison
    for(CVI itr = _rm->m_pRoadmap->begin(); itr != _rm->m_pRoadmap->end(); ++itr) {
      *closest_iter = itr.descriptor();
      closest_iter++;
    }
  }
  else {
    
    // we should make a random neighborhood finder instead of doing it like this...
    if (m_random) {
      // find k random neighbors
      for (int i = 0; i < k; i++) {
        int id = (int)(OBPRM_lrand()%(_rm->m_pRoadmap->end() - _rm->m_pRoadmap->begin()));
        *closest_iter = (_rm->m_pRoadmap->begin() + id).descriptor();
        closest_iter++;
      }
    }
    else {
      // find the k-closest neighbors
      NeighborhoodFinder::NeighborhoodFinderPointer nfptr;
      nfptr = this->GetMPProblem()->GetNeighborhoodFinder()->GetNFMethod(m_nf);
      this->GetMPProblem()->GetNeighborhoodFinder()->KClosest(nfptr, _rm, cfg, k, closest_iter);
    }                                           
  }
}
 
#endif
