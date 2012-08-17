// Contains data and methods to manipulate the environment with specified moving bodies
// (ie, robot(s)) and the corresponding roadmap.

#ifndef Query_H_
#define Query_H_

#include "Roadmap.h"     
#include "Connector.h"
#include "Environment.h"
#include "GraphAlgo.h"
#include <algorithm>


template <class CFG, class WEIGHT>
class Query : public MPBaseObject {

  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    
    Query();
    Query(string _queryFileName);
    Query(CFG _start, CFG _goal);
    Query(XMLNodeReader& _node, MPProblem* _problem, bool _warn);
    virtual ~Query();
    
    void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os) const;
    vector<CFG>& GetQuery() { return m_query; }
    vector<CFG>& GetPath() { return m_path; }

    // Reads a query and calls the other PerformQuery(), then calls Smooth() if desired
    virtual bool PerformQuery(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats);
    
    // Performs the real query work
    virtual bool PerformQuery(CFG _start, CFG _goal, Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats);
    
    // Smooths the query path by connecting path nodes to each other and rerunning the query
    virtual void Smooth();
    
    // Checks if a path is valid
    virtual bool CanRecreatePath(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats,
        vector<VID>& _attemptedPath, vector<CFG>& _recreatedPath);

    virtual void ReadQuery(string _filename);
    virtual void WritePath(Roadmap<CFG, WEIGHT>* _rdmp, string _filename);
   
  protected:
    enum GraphSearchAlg {DIJKSTRAS, ASTAR};
    vector<CFG> m_query;        // Holds the start and goal CFGs
    vector<CFG> m_path;         // The path found
    string m_queryFile;         // Where to read in the query
    string m_pathFile;          // Where to write the initial unsmoothed path
    string m_smoothFile;        // Where to write the smoothed path
    string m_lpLabel;           // Local planner
    string m_dmLabel;           // Distance metric
    GraphSearchAlg m_searchAlg; // Shortest-path graph search algorithm
    bool m_intermediateFiles;   // Store the intermediate CFGs?
    bool m_smooth;              // Perform smoothing operation?
    vector<string> m_nodeConnectionLabels;   // List of connection methods for query
    vector<string> m_smoothConnectionLabels; // List of connection methods for smoothing

  private:
    vector<VID> m_pathVIDs; // Stores path nodes for easy reference during smoothing
    bool m_doneSmoothing;   // Flag to prevent infinite recursion
};

// Heuristic for A* graph search
template<typename CFG, typename WEIGHT>
struct Heuristic {
  public:
    Heuristic(CFG& _goal, double _posRes, double _oriRes) :
      m_goal(_goal), m_posRes(_posRes), m_oriRes(_oriRes) {}

    WEIGHT operator()(CFG& _c1) {
      int tick;
      CFG incr;
      incr.FindIncrement(_c1, m_goal, &tick, m_posRes, m_oriRes);
      return WEIGHT("", tick/2);
    }

  private:
    CFG m_goal;
    double m_posRes;
    double m_oriRes;
};

// Default constructor, sets everything to NULL
template <class CFG, class WEIGHT>
Query<CFG, WEIGHT>::Query() {
  this->SetName("Query");
  m_doneSmoothing = false;
}

// Reads in query from a file, sets everything else to NULL
template <class CFG, class WEIGHT>
Query<CFG, WEIGHT>::Query(string _queryFileName) {
  this->SetName("Query");
  m_doneSmoothing = false;
  ReadQuery(_queryFileName);
}

// Uses start/goal to set up query, sets everything else to NULL
template <class CFG, class WEIGHT>     
Query<CFG, WEIGHT>::Query(CFG _start, CFG _goal) {
  this->SetName("Query");
  m_doneSmoothing = false;
  m_query.push_back(_start);
  m_query.push_back(_goal);
}

// Constructor with XML
template <class CFG, class WEIGHT>
Query<CFG, WEIGHT>::Query(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true) : MPBaseObject(_node, _problem) {
  this->SetName("Query");
  m_doneSmoothing = false;
  ParseXML(_node);
  if(m_debug)
    PrintOptions(cout);
  if(_warn)
    _node.warnUnrequestedAttributes();
  ReadQuery(m_queryFile);
}
  
template <class CFG, class WEIGHT>
Query<CFG, WEIGHT>::~Query() {}

template <class CFG, class WEIGHT>
void
Query<CFG, WEIGHT>::ParseXML(XMLNodeReader& _node) {
  m_queryFile = _node.stringXMLParameter("queryFile", true, "", "Query filename");
  m_pathFile = _node.stringXMLParameter("pathFile", false, "", "Query output path filename");
  m_smoothFile = _node.stringXMLParameter("smoothFile", false, "", "Smoothed path filename");
  m_lpLabel = _node.stringXMLParameter("lpMethod", true, "", "Local planner method");
  m_dmLabel = _node.stringXMLParameter("dmMethod", false, "default", "Distance metric method");
  string searchAlg = _node.stringXMLParameter("graphSearchAlg", false, "dijkstras", "Graph search algorithm");
  m_intermediateFiles = _node.boolXMLParameter("intermediateFiles", false, false, "Determines output of intermediate file mapnodes.path");
  m_smooth = _node.boolXMLParameter("smooth", false, false, "Whether or not to smooth the path");

  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
    if(citr->getName() == "NodeConnectionMethod") {
      m_nodeConnectionLabels.push_back(citr->stringXMLParameter("method", true, "", "Node connection method"));
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "SmoothConnectionMethod") {
      m_smoothConnectionLabels.push_back(citr->stringXMLParameter("method", true, "", "Smooth node connection method"));
      citr->warnUnrequestedAttributes();
    }
  }
  
  if(m_nodeConnectionLabels.empty()) {
    cerr << "\n\nError in QueryEvaluation XML constructor:: no node connection methods specified."
         << "\nUntil NeighborhoodFinder class can support a default/empty string as input, "
         << "node connection methods must be explicitly specified.\n\tExiting." << endl;
    exit(-1);
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

template <class CFG, class WEIGHT>
void
Query<CFG, WEIGHT>::PrintOptions(ostream& _os) const {
  _os << this->GetNameAndLabel() << "::";
  _os << "\n\tquery file = \"" << m_queryFile << "\"";
  _os << "\n\tpath file = \"" << m_pathFile << "\"";
  _os << "\n\tsmooth path file = \"" << m_smoothFile << "\"";
  _os << "\n\tdistance metric = " << m_dmLabel;
  _os << "\n\tlocal planner = " << m_lpLabel;
  _os << "\n\tsearch alg = " << m_searchAlg;
  _os << "\n\tintermediate files = " << m_intermediateFiles;
  _os << "\n\tsmooth = " << m_smooth << endl;
}

// Reads the query and calls the other PerformQuery method
template <class CFG, class WEIGHT>
bool
Query<CFG, WEIGHT>::PerformQuery(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats) {
  
  if(m_query.empty())
    cerr << "Error in Query::PerformQuery() because m_query is empty.\n"
         << "Sometimes caused by reading the wrong query file in the XML." << endl;

  for(typename vector<CFG>::iterator it = m_query.begin(); it+1 != m_query.end(); it++) {
    if(this->m_debug) {
      cout << "\nquery is ...     ";
      it->Write(cout);
      cout << "\n                 ";
      (it+1)->Write(cout);
      cout << "\nworking  ..." << endl;
    }

    if(!PerformQuery(*it, *(it+1), _rdmp, _stats)) {
      if(this->m_debug)
        cout << endl << "In PerformQuery(): didn't connect";
      return false;
    } 
  }

  if(!m_doneSmoothing) {
    if(m_pathFile == "") {
      cout << "Warning: no path file specified. Outputting path to \"Basic.path\"." << endl;
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
template <class CFG, class WEIGHT>
bool
Query<CFG, WEIGHT>::PerformQuery(CFG _start, CFG _goal, Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats) {

  if(this->m_debug)
    cout << "*** Begin query" << endl;  
  VDComment("Begin Query");

  static int graphSearchCount = 0;
  LPOutput<CFG,WEIGHT> sci, gci; // Connection info for start, goal nodes
  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map<RoadmapGraph<CFG, WEIGHT>, size_t> cmap;
  get_cc_stats(*(_rdmp->m_pRoadmap), cmap, ccs);  
  bool connected = false;

  // Process node connection labels
  vector<typename Connector<CFG, WEIGHT>::ConnectionPointer> connectionMethods;
  if(m_nodeConnectionLabels.empty())
    connectionMethods.push_back(typename Connector<CFG, WEIGHT>::ConnectionPointer(
        new NeighborhoodConnection<CFG, WEIGHT>("", "", 1, 1, false, true, false)));
  else
    for(vector<string>::iterator it = m_nodeConnectionLabels.begin(); it != m_nodeConnectionLabels.end(); it++)
      connectionMethods.push_back(GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*it));

  // Add start and goal to roadmap (if not already there)
  VID sVID, gVID;
  if(_rdmp->m_pRoadmap->IsVertex(_start))
    sVID = _rdmp->m_pRoadmap->GetVID(_start);
  else
    sVID = _rdmp->m_pRoadmap->AddVertex(_start);
  if(_rdmp->m_pRoadmap->IsVertex(_goal))
    gVID = _rdmp->m_pRoadmap->GetVID(_goal);
  else
    gVID = _rdmp->m_pRoadmap->AddVertex(_goal);

  // Loop through connected components
  typename vector<pair<size_t, VID> >::const_iterator ccIt, ccsBegin = ccs.begin();
  for(ccIt = ccsBegin; ccIt != ccs.end(); ccIt++) {
    // Store cc VIDs if needed, shortest path for smoothing later if needed
    vector<VID> cc, shortestPath;

    // Try to connect start to cc
    cmap.reset();
    if(stapl::sequential::is_same_cc(*(_rdmp->m_pRoadmap), cmap, sVID, ccIt->second)) {
      if(this->m_debug)
        cout << "start already connected to ccIt[" << distance(ccsBegin, ccIt)+1 << "]\n";
    }
    else {
      cmap.reset();
      stapl::sequential::get_cc(*(_rdmp->m_pRoadmap), cmap, ccIt->second, cc);
      vector<VID> verticesList(1, sVID);
      if(this->m_debug)
        cout << "connecting start to ccIt[" << distance(ccsBegin, ccIt)+1 << "]";

      for(typename vector<typename Connector<CFG,WEIGHT>::ConnectionPointer>::iterator
          itr = connectionMethods.begin(); itr != connectionMethods.end(); itr++) {
        cmap.reset();
        (*itr)->Connect(_rdmp, _stats, cmap, verticesList.begin(), verticesList.end(), cc.begin(), cc.end());
      }
    }

    // Try to connect goal to cc
    cmap.reset();
    if(stapl::sequential::is_same_cc(*(_rdmp->m_pRoadmap), cmap, gVID, ccIt->second)) {
      if(this->m_debug)
        cout << "goal already connected to ccIt[" << distance(ccsBegin, ccIt)+1 << "]\n";
    }
    else {
      if(cc.empty()) {
        cmap.reset();
        stapl::sequential::get_cc(*(_rdmp->m_pRoadmap), cmap, ccIt->second, cc);
      }
      vector<VID> verticesList(1, gVID);
      if(this->m_debug)
        cout << "connecting goal to ccIt[" << distance(ccsBegin, ccIt)+1 << "]";

      for(typename vector<typename Connector<CFG,WEIGHT>::ConnectionPointer>::iterator
          itr = connectionMethods.begin(); itr != connectionMethods.end(); itr++) {
        cmap.reset();
        (*itr)->Connect(_rdmp, _stats, cmap, verticesList.begin(), verticesList.end(), cc.begin(), cc.end());
      }
    }

    // Check if start and goal are connected to the same CC
    cmap.reset();
    while(stapl::sequential::is_same_cc(*(_rdmp->m_pRoadmap), cmap, sVID, gVID)) {
      //get DSSP path
      shortestPath.clear();
      cmap.reset();
      //TO DO:: fix compilation issue in parallel
#ifndef _PARALLEL
      // Run a graph search
      StatClass* stats = _rdmp->GetEnvironment()->GetMPProblem()->GetMPRegion(0)->GetStatClass();
      graphSearchCount++;
      if(this->m_recordKeep)
        stats->StartClock("Query Graph Search");
      switch(m_searchAlg) {
        case DIJKSTRAS:
          stapl::sequential::find_path_dijkstra(*(_rdmp->m_pRoadmap), sVID, gVID, shortestPath, WEIGHT::MaxWeight());
          break;
        case ASTAR:
          Heuristic<CFG, WEIGHT> heuristic(_goal, _rdmp->GetEnvironment()->GetOrientationRes(),
              _rdmp->GetEnvironment()->GetPositionRes());
          astar(*(_rdmp->m_pRoadmap), sVID, gVID, shortestPath, heuristic);
          break;
      }
      if(this->m_recordKeep)
        stats->StopClock("Query Graph Search");
#endif 
      if(this->m_debug)
        cout << "\nStart(" << shortestPath[1] << ") and Goal(" << shortestPath[shortestPath.size()-2] 
          << ") seem connected to same ccIt[" << distance(ccsBegin, ccIt)+1  << "]!" << endl;

      // Attempt to recreate path
      vector<CFG> _recreatedPath;
      if(CanRecreatePath(_rdmp, _stats, shortestPath, _recreatedPath)) {
        connected = true;
        m_path.insert(m_path.end(), _recreatedPath.begin(), _recreatedPath.end());
        if(m_smooth)
          m_pathVIDs = shortestPath;
        break;
      }
      else if(this->m_debug)
        cout << endl << "Failed to recreate path\n";
    }

    if(connected) {
      if(m_intermediateFiles) {
        // Print out all start, all graph nodes, and goal; no "ticks" from local planners
        vector<CFG> mapCFGs;
        for(typename vector<VID>::iterator it = shortestPath.begin(); it != shortestPath.end(); it++)
          mapCFGs.push_back(_rdmp->m_pRoadmap->find_vertex(*it)->property());
        WritePathConfigurations("mapnodes.path", mapCFGs, _rdmp->GetEnvironment());
      }
      break;
    }
  }
  if(this->m_debug)
    cout << "Ending query with: " << _rdmp->m_pRoadmap->get_num_vertices() << " vertices, "
         << _rdmp->m_pRoadmap->get_num_edges()/2 << " edges."
         << " graphSearchCount = " << graphSearchCount << ". Returning connected = " << connected << endl;
  VDComment("End Query");
  return connected;
}

// Smooths the path by reconnecting path nodes and rerunning the query
template <class CFG, class WEIGHT>
void
Query<CFG, WEIGHT>::Smooth() {

  Roadmap<CFG, WEIGHT>* rdmp = GetMPProblem()->GetMPRegion(0)->GetRoadmap();
  StatClass* stats = GetMPProblem()->GetMPRegion(0)->GetStatClass();
  vector<typename Connector<CFG, WEIGHT>::ConnectionPointer> methods;

  if(this->m_debug)
    cout << "\n=== Performing Query::Smooth()" << endl;
  if(this->m_recordKeep)
    stats->StartClock("Query::Smooth");

  // Process smooth connection labels
  if(m_smoothConnectionLabels.empty()) {
    cout << "Error: Must provide a smooth connection method in XML. Exiting." << endl;
    exit(1);
  }
  for(vector<string>::iterator it = m_smoothConnectionLabels.begin(); it != m_smoothConnectionLabels.end(); it++)
    methods.push_back(GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*it));
  
  // Redo connection among nodes in path
  if(this->m_debug)
    cout << "=== Starting connection among path nodes" << endl;
  stapl::sequential::vector_property_map<typename RoadmapGraph<CFG, WEIGHT>::GRAPH, size_t> cmap;
  cmap.reset();
  for(typename vector<typename Connector<CFG, WEIGHT>::ConnectionPointer>::iterator itr = methods.begin();
      itr != methods.end(); itr++)
    (*itr)->Connect(rdmp, *stats, cmap, m_pathVIDs.begin(), m_pathVIDs.end(), m_pathVIDs.begin(), m_pathVIDs.end());
  m_doneSmoothing = true;

  // Rerun query
  if(this->m_debug)
    cout << "=== Rerunning query" << endl;
  m_path.clear();
  bool smoothQueryResult = PerformQuery(rdmp, *stats);
  if(this->m_debug)
    cout << "=== Smooth query success = " << smoothQueryResult << endl;

  // Output smoothed path
  if(smoothQueryResult) {
    if(m_smoothFile == "") {
      cout << "Warning: no smooth path file specified. Outputting smoothed path to \"smooth.path\"." << endl;
      WritePath(rdmp, "smooth.path");
    }
    else
      WritePath(rdmp, m_smoothFile);
  }
  else if(this->m_debug)
    cout << "=== Smooth query failed! (This should not happen.)" << endl;
  if(this->m_recordKeep)
    stats->StopClock("Query::Smooth");
}

// Checks validity of path, recreates path if possible
template <class CFG, class WEIGHT>
bool
Query<CFG, WEIGHT>::CanRecreatePath(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats,
    vector<VID>& _attemptedPath, vector<CFG>& _recreatedPath) {

  _recreatedPath.push_back(_rdmp->m_pRoadmap->find_vertex(*(_attemptedPath.begin()))->property());
  for(typename vector<VID>::iterator it = _attemptedPath.begin(); it+1 != _attemptedPath.end(); it++) {
    LPOutput<CFG, WEIGHT> ci;
    typename RoadmapGraph<CFG, WEIGHT>::vertex_iterator vi;
    typename RoadmapGraph<CFG, WEIGHT>::adj_edge_iterator ei;
    typename RoadmapGraph<CFG, WEIGHT>::edge_descriptor ed(*it, *(it+1));
    _rdmp->m_pRoadmap->find_edge(ed, vi, ei);
    
    if(GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(m_lpLabel)->IsConnected(
          _rdmp->GetEnvironment(), _stats, GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel),
          _rdmp->m_pRoadmap->find_vertex(*it)->property(), _rdmp->m_pRoadmap->find_vertex(*(it+1))->property(),
          &ci, _rdmp->GetEnvironment()->GetPositionRes(), _rdmp->GetEnvironment()->GetOrientationRes(), true, true)) {
      _recreatedPath.insert(_recreatedPath.end(), ci.path.begin(), ci.path.end());
      _recreatedPath.push_back(_rdmp->m_pRoadmap->find_vertex(*(it+1))->property());
    }
    else
      return false;
  }
  return true;
}

// Reads query CFGs from file
template <class CFG, class WEIGHT>
void
Query<CFG, WEIGHT>::ReadQuery(string _filename) {
  CFG tempCfg;
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

// Write path to file
template <class CFG, class WEIGHT>
void
Query<CFG, WEIGHT>::WritePath(Roadmap<CFG, WEIGHT>* _rdmp, string _filename) {
  vector<Cfg*> pPath;
  for(size_t i = 0; i < m_path.size(); i++)
    pPath.push_back(&m_path[i]);
  WritePathConfigurations(_filename, pPath, _rdmp->GetEnvironment());
}

#endif
