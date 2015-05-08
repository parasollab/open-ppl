#ifndef QUERY_H_
#define QUERY_H_

#include "MapEvaluatorMethod.h"

#include "LocalPlanners/LPOutput.h"
#include "LocalPlanners/MedialAxisLP.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MedialAxisUtilities.h"

#include <containers/sequential/graph/algorithms/astar.h>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class Query : public MapEvaluatorMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::PathModifierPointer PathModifierPointer;

    Query(bool _deleteNodes=false, string _searchAlg="astar",
        string _lpLabel="", string _dmLabel="",
        string _pathModifierLabel="", bool _fullRecreatePath=true);
    Query(string _queryFileName,
        const vector<string>& _connLabels = vector<string>(),
        bool _writePaths = true);
    Query(const CfgType& _start, const CfgType& _goal, bool _writePaths = true);
    Query(MPProblemType* _problem, XMLNode& _node);
    Query(MPProblemType* _problem, CfgType _start, CfgType _goal,
        const vector<string>& _connectorLabels=vector<string>(),
        bool _deleteNodes=true, string _searchAlg="astar",
        bool _fullRecreatePath=false);
    virtual ~Query() { }

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;
    vector<CfgType>& GetQuery() { return m_query; }
    vector<CfgType>& GetPath() { return m_path; }
    vector<VID>& GetPathVIDs() { return m_pathVIDs; }
    void SetWritePath(bool _b) {m_writePaths = _b;}

    // Reads a query and calls the other PerformQuery(), then calls Smooth() if desired
    virtual bool PerformQuery(RoadmapType* _rdmp);

    // Performs the real query work
    virtual bool PerformQuery(const CfgType& _start, const CfgType& _goal, RoadmapType* _rdmp);

    // Smooths the query path
    virtual void Smooth();

    // Checks if a path is valid
    virtual bool CanRecreatePath(RoadmapType* _rdmp, vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath);

    // Node enhancement step, does nothing in Query; used in LazyPRMQuery, for example
    virtual void NodeEnhance(RoadmapType* _rdmp) { }

    virtual void ReadQuery(string _filename);

    virtual bool operator()();

  protected:
    enum GraphSearchAlg {DIJKSTRAS, ASTAR};
    vector<CfgType> m_query;    // Holds the start and goal CfgTypes
    vector<CfgType> m_path;     // The path found
    string m_queryFile;         // Where to read in the query
    string m_lpLabel;           // Local planner
    string m_dmLabel;           // Distance metric
    bool m_deleteNodes;         // Delete any added nodes?
    bool m_fullRecreatePath;    // Should the query attempt to fully recreate path (or just set VIDs)
    string m_pathModifierLabel; // Path Modifier method
    GraphSearchAlg m_searchAlg; // Shortest-path graph search algorithm
    vector<string> m_nodeConnectionLabels;   // List of connection methods for query
    bool m_writePaths;          // Should the path files be output?

  private:
    //initialize variable defaults
    void Initialize();
    void SetSearchAlgViaString(string _alg, string _where);

    vector<VID> m_pathVIDs;     // Stores path nodes for easy reference during smoothing
    vector<VID> m_toBeDeleted;  // Nodes to be deleted if m_deleteNodes is enabled.
};

// Heuristic for A* graph search
template<class MPTraits>
struct Heuristic {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;

    Heuristic(const CfgType& _goal, double _posRes, double _oriRes) :
      m_goal(_goal), m_posRes(_posRes), m_oriRes(_oriRes) { }

    WeightType operator()(const CfgType& _c1) {
      int tick;
      CfgType incr;
      incr.FindIncrement(_c1, m_goal, &tick, m_posRes, m_oriRes);
      return WeightType("", tick/2);
    }

  private:
    CfgType m_goal;
    double m_posRes;
    double m_oriRes;
};

template<class MPTraits>
Query<MPTraits>::
Query(bool _deleteNodes, string _searchAlg,
    string _lpLabel, string _dmLabel,
    string _pathModifierLabel, bool _fullRecreatePath) :
  m_lpLabel(_lpLabel), m_dmLabel(_dmLabel), m_deleteNodes(_deleteNodes),
  m_fullRecreatePath(_fullRecreatePath),
  m_pathModifierLabel(_pathModifierLabel), m_writePaths(false) {
    this->SetName("Query");
    SetSearchAlgViaString(_searchAlg, WHERE);
  }

// Reads in query from a file
template<class MPTraits>
Query<MPTraits>::
Query(string _queryFileName, const vector<string>& _connLabels,
    bool _writePaths) :
  m_nodeConnectionLabels(_connLabels), m_writePaths(_writePaths) {
    Initialize();
    ReadQuery(_queryFileName);
  }

// Uses start/goal to set up query
template<class MPTraits>
Query<MPTraits>::
Query(const CfgType& _start, const CfgType& _goal, bool _writePaths) :
  m_writePaths(_writePaths) {
    Initialize();
    m_query.push_back(_start);
    m_query.push_back(_goal);
  }

// Constructor with XML
template<class MPTraits>
Query<MPTraits>::
Query(MPProblemType* _problem, XMLNode& _node) :
  MapEvaluatorMethod<MPTraits>(_problem, _node) {
    Initialize();
    ParseXML(_node);
    ReadQuery(m_queryFile);
  }

// Uses start/goal to set up query for an existing MPProblem
template<class MPTraits>
Query<MPTraits>::
Query(MPProblemType* _problem, CfgType _start, CfgType _goal,
    const vector<string>& _connectorLabels, bool _deleteNodes,
    string _searchAlg, bool _fullRecreatePath) :
  m_deleteNodes(_deleteNodes), m_fullRecreatePath(_fullRecreatePath) {

    SetSearchAlgViaString(_searchAlg, WHERE);
    m_nodeConnectionLabels=_connectorLabels;
    m_query.push_back(_start);
    m_query.push_back(_goal);
    this->SetMPProblem(_problem);
  }

template<class MPTraits>
void
Query<MPTraits>::
ParseXML(XMLNode& _node) {
  m_queryFile = _node.Read("queryFile", true, "", "Query filename");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local planner method");
  m_dmLabel = _node.Read("dmLabel", false, "", "Distance metric method");
  string searchAlg = _node.Read("graphSearchAlg", false, "dijkstras", "Graph search algorithm");
  m_deleteNodes = _node.Read("deleteNodes", false, false, "Whether or not to delete start and goal from roadmap");
  m_fullRecreatePath = _node.Read("fullRecreatePath", false, true, "Whether or not to recreate path");
  m_pathModifierLabel = _node.Read("pmLabel", false, "", "Path modifier method");
  m_writePaths = _node.Read("writePaths", false, true, "Write path output to file?");

  for(auto& child : _node)
    if(child.Name() == "NodeConnectionMethod")
      m_nodeConnectionLabels.push_back(
          child.Read("method", true, "", "Node connection method"));

  if(m_nodeConnectionLabels.empty())
    throw ParseException(_node.Where(), "Must specify at least one connector.");

  // Ignore case for graph search algorithm
  transform(searchAlg.begin(), searchAlg.end(), searchAlg.begin(), ::tolower);
  SetSearchAlgViaString(searchAlg, _node.Where());
}

template<class MPTraits>
void
Query<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << "::";
  _os << "\n\tquery file = \"" << m_queryFile << "\"";
  _os << "\n\tdistance metric = " << m_dmLabel;
  _os << "\n\tlocal planner = " << m_lpLabel;
  _os << "\n\tsearch alg = " << m_searchAlg;
  _os << "\n\tdeleteNodes = " << m_deleteNodes ;
  _os << "\n\tfullRecreatePath = " << m_fullRecreatePath << endl;
  if(m_pathModifierLabel != "")
    _os << "\tpath modifier = \"" << m_pathModifierLabel << "\"" << endl;
  _os << "\n\twritePaths = " << m_writePaths << endl;
}

// Runs the query
template<class MPTraits>
bool
Query<MPTraits>::operator()() {

  RoadmapType* rdmp = this->GetRoadmap();

  // Perform query
  bool ans = PerformQuery(rdmp);

  return ans;
}

// Reads the query and calls the other PerformQuery method
template<class MPTraits>
bool
Query<MPTraits>::PerformQuery(RoadmapType* _rdmp) {
  if(m_query.empty())
    cerr << "Error in Query::PerformQuery() because m_query is empty.\n"
      << "Sometimes caused by reading the wrong query file in the XML." << endl;

  //clear out path before calling query to ensure that only portions relevant to
  //the query appear in the overall output.
  m_path.clear();
  m_pathVIDs.clear();

  for(typename vector<CfgType>::iterator it = m_query.begin(); it+1 != m_query.end(); it++) {
    if(this->m_debug) {
      cout << "\n*Q* query is ...     ";
      cout << *it;
      cout << "\n*Q*                  ";
      cout << *(it+1);
      cout << "\n*Q* working  ..." << endl;
    }

    if(!PerformQuery(*it, *(it+1), _rdmp)) {
      if(this->m_debug)
        cout << endl << "*Q* In Query::PerformQuery(): didn't connect";
      return false;
    }
  }

  if(m_writePaths)
    WritePath(this->GetBaseFilename() + ".full.path", m_path);

  // Path smoothing
  Smooth();

  // Delete added nodes (such as start and goal) if desired
  if(m_deleteNodes) {
    for(typename vector<VID>::iterator it = m_toBeDeleted.begin(); it != m_toBeDeleted.end(); it++)
      _rdmp->GetGraph()->delete_vertex(*it);
    m_toBeDeleted.clear();
  }

  return true;
}

// Performs the query
template<class MPTraits>
bool
Query<MPTraits>::PerformQuery(const CfgType& _start, const CfgType& _goal, RoadmapType* _rdmp) {


  if(this->m_debug)
    cout << "*Q* Begin query" << endl;
  VDComment("Begin Query");

  StatClass* stats = this->GetStatClass();
  static int graphSearchCount = 0;
  LPOutput<MPTraits> sci, gci; // Connection info for start, goal nodes
  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  stats->IncGOStat("CC Operations");
  get_cc_stats(*(_rdmp->GetGraph()), cmap, ccs);
  bool connected = false;

  if(this->m_debug)
    cout << "*Q* There are " << ccs.size() << " CCs." << endl;

  // Process node connection labels
  vector<ConnectorPointer> connectionMethods;
  if(m_nodeConnectionLabels.empty())
    m_nodeConnectionLabels.push_back("");

  for(auto&  label : m_nodeConnectionLabels)
    connectionMethods.push_back(this->GetConnector(label));

  // Add start and goal to roadmap (if not already there)
  VID sVID, gVID;
  if(_rdmp->GetGraph()->IsVertex(_start))
    sVID = _rdmp->GetGraph()->GetVID(_start);
  else {
    sVID = _rdmp->GetGraph()->AddVertex(_start);
    m_toBeDeleted.push_back(sVID);
  }
  if(_rdmp->GetGraph()->IsVertex(_goal))
    gVID = _rdmp->GetGraph()->GetVID(_goal);
  else {
    gVID = _rdmp->GetGraph()->AddVertex(_goal);
    m_toBeDeleted.push_back(gVID);
  }

  // Loop through connected components
  typename vector<pair<size_t, VID> >::const_iterator ccIt, ccsBegin = ccs.begin();
  for(ccIt = ccsBegin; ccIt != ccs.end(); ccIt++) {
    // Store cc VIDs if needed, shortest path for smoothing later if needed
    vector<VID> cc, shortestPath;

    // Try to connect start to cc
    cmap.reset();
    stats->IncGOStat("CC Operations");
    if(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, sVID, ccIt->second)) {
      if(this->m_debug)
        cout << "*Q* Start already connected to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;
    }
    else {
      cmap.reset();
      stats->IncGOStat("CC Operations");
      stapl::sequential::get_cc(*(_rdmp->GetGraph()), cmap, ccIt->second, cc);
      if(this->m_debug)
        cout << "*Q* Connecting start to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;

      for(auto&  connector : connectionMethods)
        connector->Connect(_rdmp, sVID, cc.begin(), cc.end());
    }

    // Try to connect goal to cc
    cmap.reset();
    stats->IncGOStat("CC Operations");
    if(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, gVID, ccIt->second)) {
      if(this->m_debug)
        cout << "*Q* Goal already connected to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;
    }
    else {
      if(cc.empty()) {
        cmap.reset();
        stats->IncGOStat("CC Operations");
        stapl::sequential::get_cc(*(_rdmp->GetGraph()), cmap, ccIt->second, cc);
      }
      if(this->m_debug)
        cout << "*Q* Connecting goal to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;

      for(auto&  connector : connectionMethods)
        connector->Connect(_rdmp, gVID, cc.begin(), cc.end());
    }

    // Check if start and goal are connected to the same CC
    cmap.reset();
    while(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, sVID, gVID)) {
      stats->IncGOStat("CC Operations");
      //get DSSP path
      shortestPath.clear();
      cmap.reset();
      //TO DO:: fix compilation issue in parallel
#ifndef _PARALLEL
      // Run a graph search
      graphSearchCount++;
      stats->IncGOStat("Graph Search");
      stats->StartClock("Query Graph Search");
      switch(m_searchAlg) {
        case DIJKSTRAS:
          find_path_dijkstra(*(_rdmp->GetGraph()), sVID, gVID, shortestPath, WeightType::MaxWeight());
          break;
        case ASTAR:
          Heuristic<MPTraits> heuristic(_goal,
              this->GetEnvironment()->GetPositionRes(),
              this->GetEnvironment()->GetOrientationRes());
          astar(*(_rdmp->GetGraph()), sVID, gVID, shortestPath, heuristic);
          break;
      }
      stats->StopClock("Query Graph Search");
#endif
      if(this->m_debug)
        cout << "*Q* Start(" << shortestPath[1] << ") and Goal(" << shortestPath[shortestPath.size()-2]
          << ") seem connected to same ccIt[" << distance(ccsBegin, ccIt)+1  << "]!" << endl;

      // Attempt to recreate path
      vector<CfgType> recreatedPath;
      if( m_fullRecreatePath ) {
        if(CanRecreatePath(_rdmp, shortestPath, recreatedPath)) {
          connected = true;
          m_path.insert(m_path.end(), recreatedPath.begin(), recreatedPath.end());
          m_pathVIDs.insert(m_pathVIDs.end(), shortestPath.begin(), shortestPath.end());
          break;
        }
        else if(this->m_debug)
          cout << endl << "*Q* Failed to recreate path\n";
      }
      else {
        connected = true;
        m_pathVIDs.insert(m_pathVIDs.end(), shortestPath.begin(), shortestPath.end());
        break;
      }
    }

    stats->IncGOStat("CC Operations");

    if(connected) {
      if(m_writePaths) {
        // Print out all start, all graph nodes, and goal; no "ticks" from local planners
        vector<CfgType> mapCfgs;
        for(typename vector<VID>::iterator it = shortestPath.begin(); it != shortestPath.end(); it++)
          mapCfgs.push_back(_rdmp->GetGraph()->GetVertex(*it));
        WritePath(this->GetBaseFilename() + ".rdmp.path", mapCfgs);
      }
      break;
    }
  }

  if(!connected)
    NodeEnhance(_rdmp);

  if(this->m_debug)
    cout << "*Q* Ending query with: " << _rdmp->GetGraph()->get_num_vertices() << " vertices, "
      << _rdmp->GetGraph()->get_num_edges()/2 << " edges."
      << " graphSearchCount = " << graphSearchCount << ". Returning connected = " << connected << endl;
  VDComment("End Query");
  return connected;
}

/*TODO
 * When a query has many goals, the Smoothing operations do not take that into account.
 * i.e. A smoothed path is not likely to include all the goals in the final path
 * */
// Smooth the path
template<class MPTraits>
void
Query<MPTraits>::Smooth() {
  if(m_pathModifierLabel != "") {
    vector<CfgType> pathTmp;
    this->GetPathModifier(m_pathModifierLabel)->Modify(m_path, pathTmp);
    m_path = pathTmp;
    if(m_writePaths)
      WritePath(this->GetBaseFilename() + ".smooth.path", m_path);
  }
}

// Checks validity of path, recreates path if possible
template<class MPTraits>
bool
Query<MPTraits>::
CanRecreatePath(RoadmapType* _rdmp, vector<VID>& _attemptedPath,
    vector<CfgType>& _recreatedPath) {
  _recreatedPath.push_back(
      _rdmp->GetGraph()->GetVertex(*_attemptedPath.begin()));
#ifdef _PARALLEL
  return true;
#else
  Environment* env = this->GetEnvironment();
  for(typename vector<VID>::iterator it = _attemptedPath.begin();
      it+1 != _attemptedPath.end(); it++) {
    LPOutput<MPTraits> ci;
    typename GraphType::vertex_iterator vi;
    typename GraphType::adj_edge_iterator ei;
    typename GraphType::edge_descriptor ed(*it, *(it+1));
    _rdmp->GetGraph()->find_edge(ed, vi, ei);
    CfgType col;
    CfgRef c1 = _rdmp->GetGraph()->GetVertex(*it);
    CfgRef c2 = _rdmp->GetGraph()->GetVertex(*(it+1));
    WeightType& weight = (*ei).property();
    vector<CfgType> intermediates = weight.GetIntermediates();

    if(weight.GetLPLabel() != "RRTExpand"){
      vector<CfgType> edge = this->GetLocalPlanner(weight.GetLPLabel())->
        ReconstructPath(c1, c2, intermediates,
            env->GetPositionRes(), env->GetOrientationRes());
      _recreatedPath.insert(_recreatedPath.end(), edge.begin(), edge.end());
    }
    else{
      StraightLine<MPTraits> sl;
      sl.SetMPProblem(this->GetMPProblem());
      intermediates.insert(intermediates.begin(), c1);
      intermediates.push_back(c2);
      typedef typename vector<CfgType>::iterator CIT;
      for(CIT cit = intermediates.begin();
          cit+1 != intermediates.end(); ++cit) {
        StatClass dummyStats;
        LPOutput<MPTraits> lpOutput;
        CfgType col;
        vector<CfgType> edge = sl.ReconstructPath(*cit, *(cit+1), intermediates,
            env->GetPositionRes(), env->GetOrientationRes());
        _recreatedPath.insert(_recreatedPath.end(), edge.begin(), edge.end());
      }
    }
    _recreatedPath.push_back(c2);
  }
  return true;
#endif
}

// Reads query CfgTypes from file
template<class MPTraits>
void
Query<MPTraits>::ReadQuery(string _filename) {
  CfgType tempCfg;
  ifstream in(_filename.c_str());
  if(!in) {
    cout << endl << "In ReadQuery: can't open infile: " << _filename << endl;
    return;
  }
  in >> tempCfg;
  while(in) {
    m_query.push_back(tempCfg);
    in >> tempCfg;
  }
  in.close();
}

//initialize variable defaults
template<class MPTraits>
void
Query<MPTraits>::Initialize() {
  this->SetName("Query");
  m_searchAlg = ASTAR;
  m_lpLabel = "";
  m_dmLabel = "";
  m_deleteNodes = false;
  m_fullRecreatePath = true;
}

template<class MPTraits>
void
Query<MPTraits>::
SetSearchAlgViaString(string _alg, string _where) {
  if(_alg == "dijkstras")
    m_searchAlg = DIJKSTRAS;
  else if(_alg == "astar")
    m_searchAlg = ASTAR;
  else
    throw ParseException(_where, "InValid graphSearchAlg '" + _alg +
        "'. Choices are 'dijkstras' or 'astar'.");
}

#endif
