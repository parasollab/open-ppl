// A map evaluator that runs a query. Stops running when query returns a valid path.
// If no path is found, returns to sampling again.

#ifndef QUERY_H_
#define QUERY_H_

#include "MapEvaluatorMethod.h"

template<class MPTraits>
class Query : public MapEvaluatorMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    Query();
    Query(string _queryFileName);
    Query(CfgType _start, CfgType _goal);
    Query(MPProblemType* _problem, XMLNodeReader& _node, bool _warn = true);
    virtual ~Query() { }

    void ParseXML(XMLNodeReader& _node, bool _warn);
    virtual void PrintOptions(ostream& _os); 
    vector<CfgType>& GetQuery() { return m_query; }
    vector<CfgType>& GetPath() { return m_path; }
    void SetPathFile(string _filename) {m_pathFile = _filename;}

    // Reads a query and calls the other PerformQuery(), then calls Smooth() if desired
    virtual bool PerformQuery(RoadmapType* _rdmp, StatClass& _stats);

    // Performs the real query work
    virtual bool PerformQuery(CfgType _start, CfgType _goal, RoadmapType* _rdmp, StatClass& _stats);

    // Smooths the query path by connecting path nodes to each other and rerunning the query
    virtual void Smooth();

    // Checks if a path is valid
    virtual bool CanRecreatePath(RoadmapType* _rdmp, StatClass& _stats,
        vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath);

    // Node enhancement step, does nothing in Query; used in LazyPRMQuery, for example
    virtual void NodeEnhance(RoadmapType* _rdmp, StatClass& _stats) { }

    virtual void ReadQuery(string _filename);
    virtual void WritePath(RoadmapType* _rdmp, string _filename);

    virtual bool operator()(); 

  protected:
    enum GraphSearchAlg {DIJKSTRAS, ASTAR};
    StatClass m_stats;          // Stats
    vector<CfgType> m_query;        // Holds the start and goal CfgTypes
    vector<CfgType> m_path;         // The path found
    string m_queryFile;         // Where to read in the query
    string m_pathFile;          // Where to write the initial unsmoothed path
    string m_smoothFile;        // Where to write the smoothed path
    string m_intermediateFile;  // Where to output the intermediate CfgTypes along the path if != ""
    string m_lpLabel;           // Local planner
    string m_dmLabel;           // Distance metric
    bool m_deleteNodes;         // Delete any added nodes?
    bool m_smooth;              // Perform smoothing operation?
    GraphSearchAlg m_searchAlg; // Shortest-path graph search algorithm
    vector<string> m_nodeConnectionLabels;   // List of connection methods for query
    vector<string> m_smoothConnectionLabels; // List of connection methods for smoothing

  private:
    //initialize variable defaults
    void Initialize();

    vector<VID> m_pathVIDs; // Stores path nodes for easy reference during smoothing
    bool m_doneSmoothing;   // Flag to prevent infinite recursion
    vector<VID> m_toBeDeleted; //nodes to be deleted if m_deleteNodes is enabled.
};

// Heuristic for A* graph search
template<class MPTraits>
struct Heuristic {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    
    Heuristic(CfgType& _goal, double _posRes, double _oriRes) :
      m_goal(_goal), m_posRes(_posRes), m_oriRes(_oriRes) {}

    WeightType operator()(CfgType& _c1) {
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
Query<MPTraits>::Query() {
  Initialize();
}

// Reads in query from a file
template<class MPTraits>
Query<MPTraits>::Query(string _queryFileName) {
  Initialize();
  ReadQuery(_queryFileName);
}

// Uses start/goal to set up query
template<class MPTraits>
Query<MPTraits>::Query(CfgType _start, CfgType _goal) {
  Initialize();
  m_query.push_back(_start);
  m_query.push_back(_goal);
}

// Constructor with XML
template<class MPTraits>
Query<MPTraits>::Query(MPProblemType* _problem, XMLNodeReader& _node, bool _warn) :
  MapEvaluatorMethod<MPTraits>(_problem, _node) {
    Initialize();
    ParseXML(_node, _warn);
    if(_warn)
      _node.warnUnrequestedAttributes();
    ReadQuery(m_queryFile);
  }

template<class MPTraits>
void
Query<MPTraits>::ParseXML(XMLNodeReader& _node, bool _warn) {
  m_queryFile = _node.stringXMLParameter("queryFile", true, "", "Query filename");
  m_pathFile = _node.stringXMLParameter("pathFile", false, "", "Query output path filename");
  m_smoothFile = _node.stringXMLParameter("smoothFile", false, "", "Smoothed path filename");
  m_lpLabel = _node.stringXMLParameter("lpMethod", true, "", "Local planner method");
  m_dmLabel = _node.stringXMLParameter("dmMethod", false, "default", "Distance metric method");
  string searchAlg = _node.stringXMLParameter("graphSearchAlg", false, "dijkstras", "Graph search algorithm");
  m_intermediateFile = _node.stringXMLParameter("intermediateFiles", false, "", "Determines output location of intermediate nodes.");
  m_smooth = _node.boolXMLParameter("smooth", false, false, "Whether or not to smooth the path");
  m_deleteNodes = _node.boolXMLParameter("deleteNodes", false, false, "Whether or not to delete start and goal from roadmap");

  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
    if(citr->getName() == "NodeConnectionMethod") {
      m_nodeConnectionLabels.push_back(citr->stringXMLParameter("method", true, "", "Node connection method"));
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "SmoothConnectionMethod") {
      m_smoothConnectionLabels.push_back(citr->stringXMLParameter("method", true, "", "Smooth node connection method"));
      citr->warnUnrequestedAttributes();
    }
  }

  if(m_nodeConnectionLabels.empty()) {
    cerr << "\n\nWARNING:: in Query XML constructor:: no node connection methods specified. Default will be used." << endl;
  }

  // Ignore case for graph search algorithm
  transform(searchAlg.begin(), searchAlg.end(), searchAlg.begin(), ::tolower);
  if(searchAlg == "dijkstras")
    m_searchAlg = DIJKSTRAS;
  else if(searchAlg == "astar")
    m_searchAlg = ASTAR;
  else {
    cout << "Error: invalid graphSearchAlg; valid choices are: \"dijkstras\", \"astar\". Exiting." << endl;
    exit(1);
  }
}

template<class MPTraits>
void
Query<MPTraits>::PrintOptions(ostream& _os) {
  _os << this->GetNameAndLabel() << "::";
  _os << "\n\tquery file = \"" << m_queryFile << "\"";
  _os << "\n\tpath file = \"" << m_pathFile << "\"";
  _os << "\n\tsmooth path file = \"" << m_smoothFile << "\"";
  _os << "\n\tintermediate file = \"" << m_intermediateFile << "\"";
  _os << "\n\tdistance metric = " << m_dmLabel;
  _os << "\n\tlocal planner = " << m_lpLabel;
  _os << "\n\tsearch alg = " << m_searchAlg;
  _os << "\n\tsmooth = " << m_smooth << endl;
  _os << "\n\tdeleteNodes = " << m_deleteNodes << endl;
}

// Runs the query
template<class MPTraits>
bool
Query<MPTraits>::operator()() {

  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();  

  // Perform query
  bool ans = PerformQuery(rdmp, m_stats);

  // Delete added nodes (such as start and goal) if desired
  if(m_deleteNodes){
    for(typename vector<VID>::iterator it = m_toBeDeleted.begin(); it != m_toBeDeleted.end(); it++){
      rdmp->GetGraph()->delete_vertex(*it);
    }
    m_toBeDeleted.clear();
  }

  return ans;
}

// Reads the query and calls the other PerformQuery method
template<class MPTraits>
bool
Query<MPTraits>::PerformQuery(RoadmapType* _rdmp, StatClass& _stats) {

  if(m_query.empty())
    cerr << "Error in Query::PerformQuery() because m_query is empty.\n"
      << "Sometimes caused by reading the wrong query file in the XML." << endl;

  for(typename vector<CfgType>::iterator it = m_query.begin(); it+1 != m_query.end(); it++) {
    if(this->m_debug) {
      cout << "\n*Q* query is ...     ";
      it->Write(cout);
      cout << "\n*Q*                  ";
      (it+1)->Write(cout);
      cout << "\n*Q* working  ..." << endl;
    }

    if(!PerformQuery(*it, *(it+1), _rdmp, _stats)) {
      if(this->m_debug)
        cout << endl << "*Q* In Query::PerformQuery(): didn't connect";
      return false;
    } 
  }

  if(!m_doneSmoothing) {
    if(m_pathFile == "") {
      cerr << "Warning: no path file specified. Outputting path to \"Basic.path\"." << endl;
      WritePath(_rdmp, "Basic.path");
    }
    else
      WritePath(_rdmp, m_pathFile);

    if(m_smooth)
      Smooth();
  }
  return true;
}

// Performs the query
template<class MPTraits>
bool
Query<MPTraits>::PerformQuery(CfgType _start, CfgType _goal, RoadmapType* _rdmp, StatClass& _stats) {

  if(this->m_debug)
    cout << "*Q* Begin query" << endl;  
  VDComment("Begin Query");

  StatClass* stats = this->GetMPProblem()->GetStatClass();
  static int graphSearchCount = 0;
  LPOutput<MPTraits> sci, gci; // Connection info for start, goal nodes
  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  if(this->m_recordKeep)
    stats->IncGOStat("CC Operations");
  get_cc_stats(*(_rdmp->GetGraph()), cmap, ccs);
  bool connected = false;

  if(this->m_debug)
    cout << "*Q* There are " << ccs.size() << " CCs." << endl;

  // Process node connection labels
  vector<ConnectorPointer> connectionMethods;
  if(m_nodeConnectionLabels.empty()){
    m_nodeConnectionLabels.push_back("");
  }
  
  for(vector<string>::iterator it = m_nodeConnectionLabels.begin(); it != m_nodeConnectionLabels.end(); it++)
    connectionMethods.push_back(this->GetMPProblem()->GetConnector(*it));

  // Add start and goal to roadmap (if not already there)
  VID sVID, gVID;
  if(_rdmp->GetGraph()->IsVertex(_start))
    sVID = _rdmp->GetGraph()->GetVID(_start);
  else{
    sVID = _rdmp->GetGraph()->AddVertex(_start);
    m_toBeDeleted.push_back(sVID);
  }
  if(_rdmp->GetGraph()->IsVertex(_goal))
    gVID = _rdmp->GetGraph()->GetVID(_goal);
  else{
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
    if(this->m_recordKeep)
      stats->IncGOStat("CC Operations");
    if(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, sVID, ccIt->second)) {
      if(this->m_debug)
        cout << "*Q* Start already connected to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;
    }
    else {
      cmap.reset();
      if(this->m_recordKeep)
        stats->IncGOStat("CC Operations");
      stapl::sequential::get_cc(*(_rdmp->GetGraph()), cmap, ccIt->second, cc);
      vector<VID> verticesList(1, sVID);
      if(this->m_debug)
        cout << "*Q* Connecting start to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;

      for(typename vector<ConnectorPointer>::iterator
          itr = connectionMethods.begin(); itr != connectionMethods.end(); itr++) {
        cmap.reset();
        (*itr)->Connect(_rdmp, _stats, cmap, verticesList.begin(), verticesList.end(), cc.begin(), cc.end());
      }
    }

    // Try to connect goal to cc
    cmap.reset();
    if(this->m_recordKeep)
      stats->IncGOStat("CC Operations");
    if(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, gVID, ccIt->second)) {
      if(this->m_debug)
        cout << "*Q* Goal already connected to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;
    }
    else {
      if(cc.empty()) {
        cmap.reset();
        if(this->m_recordKeep)
          stats->IncGOStat("CC Operations");
        stapl::sequential::get_cc(*(_rdmp->GetGraph()), cmap, ccIt->second, cc);
      }
      vector<VID> verticesList(1, gVID);
      if(this->m_debug)
        cout << "*Q* Connecting goal to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;

      for(typename vector<ConnectorPointer>::iterator
          itr = connectionMethods.begin(); itr != connectionMethods.end(); itr++) {
        cmap.reset();
        (*itr)->Connect(_rdmp, _stats, cmap, verticesList.begin(), verticesList.end(), cc.begin(), cc.end());
      }
    }

    // Check if start and goal are connected to the same CC
    cmap.reset();
    while(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, sVID, gVID)) {
      if(this->m_recordKeep)
        stats->IncGOStat("CC Operations");
      //get DSSP path
      shortestPath.clear();
      cmap.reset();
      //TO DO:: fix compilation issue in parallel
#ifndef _PARALLEL
      // Run a graph search
      graphSearchCount++;
      if(this->m_recordKeep) {
        stats->IncGOStat("Graph Search");
        stats->StartClock("Query Graph Search");
      }
      switch(m_searchAlg) {
        case DIJKSTRAS:
          stapl::sequential::find_path_dijkstra(*(_rdmp->GetGraph()), sVID, gVID, shortestPath, WeightType::MaxWeight());
          break;
        case ASTAR:
          Heuristic<MPTraits> heuristic(_goal, this->GetMPProblem()->GetEnvironment()->GetOrientationRes(),
              this->GetMPProblem()->GetEnvironment()->GetPositionRes());
          astar(*(_rdmp->GetGraph()), sVID, gVID, shortestPath, heuristic);
          break;
      }
      if(this->m_recordKeep)
        stats->StopClock("Query Graph Search");
#endif 
      if(this->m_debug)
        cout << "\n*Q* Start(" << shortestPath[1] << ") and Goal(" << shortestPath[shortestPath.size()-2] 
          << ") seem connected to same ccIt[" << distance(ccsBegin, ccIt)+1  << "]!" << endl;

      // Attempt to recreate path
      vector<CfgType> _recreatedPath;
      if(CanRecreatePath(_rdmp, _stats, shortestPath, _recreatedPath)) {
        connected = true;
        m_path.insert(m_path.end(), _recreatedPath.begin(), _recreatedPath.end());
        if(m_smooth)
          m_pathVIDs = shortestPath;
        break;
      }
      else if(this->m_debug)
        cout << endl << "*Q* Failed to recreate path\n";
    }

    if(this->m_recordKeep)
      stats->IncGOStat("CC Operations");

    if(connected) {
      if(m_intermediateFile != "") {
        // Print out all start, all graph nodes, and goal; no "ticks" from local planners
        vector<CfgType> mapCfgTypes;
        for(typename vector<VID>::iterator it = shortestPath.begin(); it != shortestPath.end(); it++)
          mapCfgTypes.push_back(_rdmp->GetGraph()->find_vertex(*it)->property());
        WritePathConfigurations(m_intermediateFile, mapCfgTypes, this->GetMPProblem()->GetEnvironment());
      }
      break;
    }
  }

  if(!connected)
    NodeEnhance(_rdmp, _stats);

  if(this->m_debug)
    cout << "*Q* Ending query with: " << _rdmp->GetGraph()->get_num_vertices() << " vertices, "
      << _rdmp->GetGraph()->get_num_edges()/2 << " edges."
      << " graphSearchCount = " << graphSearchCount << ". Returning connected = " << connected << endl;
  VDComment("End Query");
  return connected;
}

// Smooths the path by reconnecting path nodes and rerunning the query
template<class MPTraits>
void
Query<MPTraits>::Smooth() {

  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  vector<ConnectorPointer> methods;

  if(this->m_debug)
    cout << "\n*S* Performing Query::Smooth()" << endl;
  if(this->m_recordKeep)
    stats->StartClock("Query::Smooth");

  // Process smooth connection labels
  if(m_smoothConnectionLabels.empty()) {
    cout << "Error: Must provide a smooth connection method in XML. Exiting." << endl;
    exit(1);
  }
  for(vector<string>::iterator it = m_smoothConnectionLabels.begin(); it != m_smoothConnectionLabels.end(); it++)
    methods.push_back(this->GetMPProblem()->GetConnector(*it));

  // Redo connection among nodes in path
  if(this->m_debug)
    cout << "*S* Starting connection among path nodes" << endl;
  stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
  cmap.reset();
  for(typename vector<ConnectorPointer>::iterator itr = methods.begin();
      itr != methods.end(); itr++)
    (*itr)->Connect(rdmp, *stats, cmap, m_pathVIDs.begin(), m_pathVIDs.end(), m_pathVIDs.begin(), m_pathVIDs.end());
  m_doneSmoothing = true;

  // Rerun query
  if(this->m_debug)
    cout << "*S* Rerunning query" << endl;
  m_path.clear();
  bool smoothQueryResult = PerformQuery(rdmp, *stats);
  if(this->m_debug)
    cout << "*S* Smooth query success = " << smoothQueryResult << endl;

  // Output smoothed path
  if(smoothQueryResult) {
    if(m_smoothFile == "")
      cerr << "Warning: no smooth path file specified. Outputting smoothed path to \"Basic.smooth.path\"." << endl;
    WritePath(rdmp, m_smoothFile);
  }
  else if(this->m_debug)
    cout << "*S* Smooth query failed! (This should not happen.)" << endl;
  if(this->m_recordKeep)
    stats->StopClock("Query::Smooth");
}

// Checks validity of path, recreates path if possible
template<class MPTraits>
bool
Query<MPTraits>::CanRecreatePath(RoadmapType* _rdmp, StatClass& _stats,
    vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath) {

  _recreatedPath.push_back(_rdmp->GetGraph()->find_vertex(*(_attemptedPath.begin()))->property());
  for(typename vector<VID>::iterator it = _attemptedPath.begin(); it+1 != _attemptedPath.end(); it++) {
    LPOutput<MPTraits> ci;
    typename GraphType::vertex_iterator vi;
    typename GraphType::adj_edge_iterator ei;
    typename GraphType::edge_descriptor ed(*it, *(it+1));
    _rdmp->GetGraph()->find_edge(ed, vi, ei);
    CfgType col;

    if(this->GetMPProblem()->GetLocalPlanner(m_lpLabel)->IsConnected(
          this->GetMPProblem()->GetEnvironment(), _stats, this->GetMPProblem()->GetDistanceMetric(m_dmLabel),
          _rdmp->GetGraph()->find_vertex(*it)->property(), _rdmp->GetGraph()->find_vertex(*(it+1))->property(),
          col, &ci, this->GetMPProblem()->GetEnvironment()->GetPositionRes(), this->GetMPProblem()->GetEnvironment()->GetOrientationRes(), true, true, true)) {
      _recreatedPath.insert(_recreatedPath.end(), ci.path.begin(), ci.path.end());
      _recreatedPath.push_back(_rdmp->GetGraph()->find_vertex(*(it+1))->property());
    }
    else{
      cerr << "Error::When querying, invalid edge of graph was found between vid pair (" 
        << *it << ", " << *(it+1) << ")" << " outputing error path in error.path and exiting." << endl;
      _recreatedPath.insert(_recreatedPath.end(), ci.path.begin(), ci.path.end());
      _recreatedPath.push_back(col);
      WritePathConfigurations("error.path", _recreatedPath, this->GetMPProblem()->GetEnvironment());
      exit(1);
    }
  }
  return true;
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
  tempCfg.Read(in);
  while(in) {
    m_query.push_back(tempCfg);
    tempCfg.Read(in);
  }
  in.close();
}

// Writes path to file
template<class MPTraits>
void
Query<MPTraits>::WritePath(RoadmapType* _rdmp, string _filename) {
  vector<Cfg*> pPath;
  for(size_t i = 0; i < m_path.size(); i++)
    pPath.push_back(&m_path[i]);
  WritePathConfigurations(_filename, pPath, this->GetMPProblem()->GetEnvironment());
}

//initialize variable defaults
template<class MPTraits>
void 
Query<MPTraits>::Initialize(){
  this->SetName("Query");
  m_doneSmoothing = false;
  m_searchAlg = ASTAR;
  m_pathFile = "Basic.path";
  m_smoothFile = "Basic.smooth.path";
  m_intermediateFile = "";
  m_lpLabel = "";
  m_dmLabel = "";
  m_deleteNodes = false;
  m_smooth = false;
}

#endif
