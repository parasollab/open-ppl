#ifndef PreferentialAttachment_h
#define PreferentialAttachment_h
#include "NodeConnectionMethod.h"
#include "LocalPlanners.h"
#include "Clock_Class.h"
#include "GraphAlgo.h"
#include "NeighborhoodFinder.h"

// Preferential attachment 

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

template <class CFG, class WEIGHT>
class PreferentialAttachment: public NodeConnectionMethod<CFG,WEIGHT> {
 public:
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  typedef typename RoadmapGraph<CFG, WEIGHT>::CVI CVI;
  typedef typename RoadmapGraph<CFG, WEIGHT>::VI VI;
  
  //////////////////////
  // Constructors and Destructor
  PreferentialAttachment();
  PreferentialAttachment(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  PreferentialAttachment(int k); // constructor for preferential attachment
  PreferentialAttachment(int k, int m); // constructor for preferential attachment
  
  virtual ~PreferentialAttachment();
 
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
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges) ;

  template<typename InputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last) ;


  template<typename InputIterator>
  void ConnectNodes(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addPartialEdge, bool addAllEdges,
        InputIterator _itr1_first, InputIterator _itr1_last,
        InputIterator _itr2_first, InputIterator _itr2_last) ; 
    
  void ConnectNeighbors(
        Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
        LocalPlanners<CFG,WEIGHT>* lp,
        bool addAllEdges, 
        int &total_success, int &total_failure,
        VID _vid, vector<VID> closest) ;

 private:
     
  //////////////////////
  // Probability function
  double pref_prob(
          Roadmap<CFG, WEIGHT>* _rm, VID _vid, int _n) {
    int candidate_degree = _rm->m_pRoadmap->get_degree(_vid);
    int total_degree = _rm->m_pRoadmap->get_num_edges(); 
    if (m_debug) cout << "pref_prob(" << _vid << ", " << _n << ") = " << 1 + candidate_degree << " / " << _n + total_degree << endl;
    return ((double)(1 + candidate_degree) / (double)(_n + total_degree));
  }
  //////////////////////
  // Data

  int m_k;
  int m_fail;
  bool m_unconnected;
  bool m_random;
  bool m_debug;
  string dm_label;
  shared_ptr<DistanceMetricMethod> dm;
};


template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::PreferentialAttachment():NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "PreferentialAttachment"; 
  SetDefault();
}

template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::PreferentialAttachment(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    NodeConnectionMethod<CFG,WEIGHT>(in_Node, in_pProblem) { 
  LOG_DEBUG_MSG("PreferentialAttachment::PreferentialAttachment()"); 
  this->element_name = "PreferentialAttachment"; 
  SetDefault();
  ParseXML(in_Node);
  
  
  LOG_DEBUG_MSG("~PreferentialAttachment::PreferentialAttachment()"); 
}


//this is backward support for function call from other class
//to be cleaned
template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::PreferentialAttachment(int k) : 
    NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "PreferentialAttachment"; 
  m_k = k;
  m_fail = k;
  m_unconnected = false;
  m_random = false;
  m_debug = false;
}

template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::PreferentialAttachment(int k, int m) : 
    NodeConnectionMethod<CFG,WEIGHT>() { 
  this->element_name = "PreferentialAttachment"; 
  m_k = k;
  m_fail = m;
  m_unconnected = false;
  m_random = false;
  m_debug = false;
}


template <class CFG, class WEIGHT>
PreferentialAttachment<CFG,WEIGHT>::~PreferentialAttachment() { 
}

template <class CFG, class WEIGHT>
void PreferentialAttachment<CFG,WEIGHT>::ParseXML(XMLNodeReader& in_Node) { 
 
  m_k = in_Node.numberXMLParameter(string("k"), true, 0, 0, 10000, 
                                 string("k-value (max neighbors to find). k = 0 --> all-pairs"));
  m_fail = in_Node.numberXMLParameter(string("fail"), false, 5, 0, 10000, 
                                    string("amount of failed connections allowed before operation terminates"));
  m_unconnected = in_Node.boolXMLParameter(string("unconnected"), false, false,
                                         string("if true, do not count existing connections towards k"));
  m_random = in_Node.boolXMLParameter(string("random"), false, false,
                                         string("if true, find k random configurations from destination vector"));
  m_debug = in_Node.boolXMLParameter(string("debug"), false, false, string(""));
  dm_label =in_Node.stringXMLParameter(string("dm_method"), false,
                                    string("default"), string("Distance Metric Method"));
  
  
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
void PreferentialAttachment<CFG,WEIGHT>::SetDefault() {
  m_k = KCLOSEST;
  m_fail = MFAILURE;
}


template <class CFG, class WEIGHT>
void
PreferentialAttachment<CFG, WEIGHT>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << this->GetName() << " ";
  _os << "\tINTEGER INTEGER (default k:" << KCLOSEST << ", mfailure:" << MFAILURE << ")";
  _os << endl;
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG, class WEIGHT>
void
PreferentialAttachment<CFG, WEIGHT>::
PrintValues(ostream& _os){
  _os << "\n" << this->GetName() << " k = ";
  _os << m_k << " fail = " << m_fail;
  _os << endl;
}

template <class CFG, class WEIGHT>
void
PreferentialAttachment<CFG, WEIGHT>::
PrintOptions(ostream& out_os){
  out_os << "    " << this->GetName() << "::  k = ";
  out_os << m_k << "  fail = " << m_fail ;
  out_os << endl;
}


template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
PreferentialAttachment<CFG,WEIGHT>::
CreateCopy() {
  NodeConnectionMethod<CFG,WEIGHT>* _copy = 
           new PreferentialAttachment<CFG,WEIGHT>(*this);
  return _copy;
}


// ConnectNodes
//    This method is a wrapper for the two-iterator function, and handles
//    the special case where we want to perform connections over all nodes
//    in the roadmap.
template <class CFG, class WEIGHT>
void PreferentialAttachment<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges) 
{
  //cout << "PreferentialAttachment<CFG,WEIGHT>::ConnectNodes() - Roadmap Only" << endl << flush;
  //cout << "Connecting CCs with method: closest k="<< k << endl;
#ifndef QUIET
  cout << "closest(k="<< m_k <<"): "<<flush;
#endif
  
  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  vector<VID> vertices;
  pMap->GetVerticesVID(vertices);
  
  ConnectNodes(_rm, Stats, lp, addPartialEdge, addAllEdges, 
        vertices.begin(), vertices.end());
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
template<typename InputIterator>
void PreferentialAttachment<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            InputIterator _itr1_first, InputIterator _itr1_last) 
{
  cout << "PreferentialAttachment<CFG,WEIGHT>::ConnectNodes() - 1 pairs InputIterator" << endl << flush;

  RoadmapGraph<CFG, WEIGHT>* pMap = _rm->m_pRoadmap;
  Clock_Class KClosestClock;
  
  int total_success = 0;
  int total_failure = 0;

  vector<VID> input_vertices(_itr1_first, _itr1_last);
  
  for (int n = 2; n < input_vertices.size(); n++) {
    if (m_debug) {
      cout << "***************************************************************\n";
      cout << "VID - input_vertices[" << n << "] = " << input_vertices[n] << endl;
    }
    int attempts = 0;

    KClosestClock.StartClock("kClosest");
    while (attempts < min(m_k, n-1)) {
      for (int i = 0; i < n-1; i++) {
        double drand = OBPRM_drand();
        double prob = pref_prob(_rm, input_vertices[i], n);
        if (m_debug) cout << "attempts = " << attempts << ", drand = " << drand << ", prob = " << prob << endl;
        if (drand < prob) {

          vector<VID> candidate;
          candidate.push_back(input_vertices[i]);

          KClosestClock.StopClock();
          if (m_debug) cout << "\tAttempting connections: VID = " << input_vertices[n] << "  --> " << candidate[0] << endl;
          ConnectNeighbors(_rm, Stats, lp, addAllEdges, total_success, total_failure, input_vertices[n], candidate);
          KClosestClock.StartClock("kClosest");

          attempts += 1;
        }
        if (attempts == min(m_k, n-1)) {
          break;
        }
      }
    }  
    KClosestClock.StopClock();
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
void PreferentialAttachment<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addPartialEdge,
            bool addAllEdges,
            InputIterator _itr1_first, InputIterator _itr1_last,
            InputIterator _itr2_first, InputIterator _itr2_last)
{   
    cout << "PreferentialAttachment<CFG,WEIGHT>::ConnectNodes() - 2 pairs InputIterator" << endl << flush;
    cout << "*** Preferential Attachment for 2 pairs InputIterator isn't supported. ***" << endl << flush;
  
}

template <class CFG, class WEIGHT>
void PreferentialAttachment<CFG,WEIGHT>::
ConnectNeighbors(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats, 
            LocalPlanners<CFG,WEIGHT>* lp,
            bool addAllEdges,
            int &total_success, int &total_failure,
            VID _vid, vector<VID> closest)
{ 
  LPOutput<CFG,WEIGHT> lpOutput;
  
  // connect the found k-closest to the current iteration's CFG
  for(typename vector<VID>::iterator itr2 = closest.begin(); itr2!= closest.end(); ++itr2) {
    
    if (m_debug) cout << "\t(s,f) = (" << total_success << "," << total_failure << ")";
    if (m_debug) cout << " | VID = " << *itr2;
    if (m_debug) cout << " | dist = " << dm->Distance(
            _rm->GetEnvironment(), (*(_rm->m_pRoadmap->find_vertex(_vid))).property(), (*(_rm->m_pRoadmap->find_vertex(*itr2))).property()
            );
    
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
          total_failure++;
        }
        if (m_debug) cout << endl;
        continue;
      }
    }
  
    // the edge already exists
    if (_rm->m_pRoadmap->IsEdge(_vid, *itr2)) {
      // if we're not in "unconnected" mode, count this as a success
      if (m_debug) cout << " | edge already exists in roadmap";
      if (!m_unconnected) {
        if (m_debug) cout << " | success incremented";
        total_success++;
      }
      if (m_debug) cout << endl;
      continue;
    }
    
    if (this->m_CheckIfSameCC) {
      // the nodes are in the same connected component
      stapl::vector_property_map< RoadmapGraph<CFG,WEIGHT>,size_t > cmap;
      if (is_same_cc(*(_rm->m_pRoadmap), cmap, _vid, *itr2)) {
        // if we're not in "unconnected" mode, count this as a success
        if (m_debug) cout << " | nodes in the same connected component";
        if (!m_unconnected) {
          if (m_debug) cout << " | success incremented";
          total_success++;
        }
        if (m_debug) cout << endl;
        continue;
      }
    }

    // record the attempted connection
    Stats.IncConnections_Attempted();

    shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetDMMethod(dm_label);
    // attempt connection with the local planner
    if(lp->IsConnected(_rm->GetEnvironment(), Stats, dm,
                (*(_rm->m_pRoadmap->find_vertex(_vid))).property(),
                (*(_rm->m_pRoadmap->find_vertex(*itr2))).property(),
                &lpOutput, this->connectionPosRes, this->connectionOriRes, 
                (!addAllEdges) ))
    {
      // if connection was made, add edge and record the successful connection
      if (m_debug) cout << " | connection was successful";
      _rm->m_pRoadmap->AddEdge(_vid, *itr2, lpOutput.edge);
      Stats.IncConnections_Made();
      this->connection_attempts.push_back(make_pair(make_pair(_vid, *itr2), true));
      
      // mark the successful connection in the roadmap's cache
      if (m_debug) cout << " | success incremented" << endl;
      _rm->SetCache(_vid,*itr2,true);
      total_success++;
    }
    else {
      // mark the failed connection in the roadmap's cache
      if (m_debug) cout << " | connection failed | failure incremented" << endl;
      _rm->SetCache(_vid,*itr2,false);
      total_failure++;
      this->connection_attempts.push_back(make_pair(make_pair(_vid, *itr2), false));
    }
  }
}

#endif
