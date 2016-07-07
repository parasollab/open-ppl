#ifndef RRTQUERY_H_
#define RRTQUERY_H_

#include "MapEvaluatorMethod.h"

#include "LocalPlanners/LPOutput.h"
#include "LocalPlanners/StraightLine.h"
#include "Utilities/MetricUtils.h"
#include "MapEvaluators/Query.h"
#include <containers/sequential/graph/algorithms/astar.h>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RRTQuery : public MapEvaluatorMethod<MPTraits> {

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
    
    RRTQuery(string _searchAlgo = "astar", string _dmLabel = "", 
    string _nfLabel = "", double _goalDist = 0.0);
    RRTQuery(string _queryFileName, double _goalDist = 0.0, 
        const vector<string>& _connLabels = vector<string>(),
        bool _writePaths = true);
    RRTQuery(const CfgType& _start, const CfgType& _goal, bool _writePaths = true); 
    RRTQuery(MPProblemType* _problem, XMLNode& _node);
    virtual ~RRTQuery() { }

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;
    vector<CfgType>& GetQuery() { return m_query; }
    vector<CfgType>& GetPath() { return m_path; }
    vector<VID>& GetPathVIDs() { return m_pathVIDs; }
    void SetWritePath(bool _b) {m_writePaths = _b;}

    // Reads a query and calls the other PerformQuery()
    virtual bool PerformQuery(RoadmapType* _rdmp);

    // Performs the real query work
    virtual bool PerformQuery(const CfgType& _start, const CfgType& _goal, RoadmapType* _rdmp);
    
    // Checks if a path is valid
    virtual bool CanRecreatePath(RoadmapType* _rdmp, vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath);

    virtual void ReadQuery(string _filename);

    virtual bool operator()();

  protected:
    enum GraphSearchAlg {DIJKSTRAS, ASTAR};
    vector<CfgType> m_query;    // Holds the start and goal CfgTypes
    vector<CfgType> m_path;     // The path found
    string m_queryFile;         // Where to read in the query
    string m_dmLabel;           // Distance metric
    string m_nfLabel;           // Neighborhood finder  
    bool m_fullRecreatePath;    // Should the query attempt to fully recreate path (or just set VIDs)
    GraphSearchAlg m_searchAlg; // Shortest-path graph search algorithm
    bool m_writePaths;          // Should the path files be output?

  private:
    //initialize variable defaults
    void Initialize();
    void SetSearchAlgViaString(string _alg, string _where);
    double CalculatePathLength(const vector<VID>& _shortestpath, RoadmapType* _rdmp); 
    vector<VID> m_pathVIDs;     // Stores path nodes for easy reference during smoothing
    double m_goalDist;
    vector<CfgType> m_goals;
};

// Default Constructor
template<class MPTraits>
RRTQuery<MPTraits>::
RRTQuery(string _searchAlgo, string _dmLabel, 
    string _nfLabel, double _goalDist): m_dmLabel(_dmLabel),
    m_nfLabel(_nfLabel), m_goalDist(_goalDist) {
      this->SetName("RRTQuery");
      SetSearchAlgViaString(_searchAlgo, WHERE);
}


// Reads in query from a file
template<class MPTraits>
RRTQuery<MPTraits>::
RRTQuery(string _queryFileName, double _goalDist, const vector<string>& _connLabels,
    bool _writePaths): m_writePaths(_writePaths) {
    Initialize();
    ReadQuery(_queryFileName);
  }

// Constructor with XML
template<class MPTraits>
RRTQuery<MPTraits>::
RRTQuery(MPProblemType* _problem, XMLNode& _node) :
  MapEvaluatorMethod<MPTraits>(_problem, _node) {
    Initialize();
    ParseXML(_node);
    cout << "Reading Query file\n"; 
    ReadQuery(m_queryFile);
  }

template<class MPTraits>
RRTQuery<MPTraits>::
RRTQuery(const CfgType& _start, const CfgType& _goal, bool _writePaths):m_writePaths(_writePaths) {
  Initialize();
  m_query.push_back(_start);
  m_query.push_back(_goal);
}
 
template<class MPTraits>
void
RRTQuery<MPTraits>::
ParseXML(XMLNode& _node) {
  m_queryFile = _node.Read("queryFile", true, "", "Query filename");
  m_dmLabel = _node.Read("dmLabel", false, "", "Distance metric method");
  m_nfLabel = _node.Read("nfLabel", false, "Nearest", "Neighborhood finder method"); 
  string searchAlg = _node.Read("graphSearchAlg", false, "dijkstras", "Graph search algorithm");
  m_fullRecreatePath = _node.Read("fullRecreatePath", false, true, "Whether or not to recreate path");
  m_writePaths = _node.Read("writePaths", false, true, "Write path output to file?");
  m_goalDist = stod(_node.Read("goalDist", false, "-1.0", "Minimun Distance for valid query")); 
  
   // Ignore case for graph search algorithm
  transform(searchAlg.begin(), searchAlg.end(), searchAlg.begin(), ::tolower);
  SetSearchAlgViaString(searchAlg, _node.Where());
}

template<class MPTraits>
void
RRTQuery<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << "::";
  _os << "\n\tquery file = \"" << m_queryFile << "\"";
  _os << "\n\tdistance metric = " << m_dmLabel;
  _os << "\n\tsearch alg = " << m_searchAlg;
  _os << "\n\tfullRecreatePath = " << m_fullRecreatePath << endl;
  _os << "\n\twritePaths = " << m_writePaths << endl;
}

// Runs the query
template<class MPTraits>
bool
RRTQuery<MPTraits>::
operator()() {

  RoadmapType* rdmp = this->GetRoadmap();

  // Perform query
  bool ans = PerformQuery(rdmp);

  return ans;
}

// Reads the query and calls the other PerformQuery method
template<class MPTraits>
bool
RRTQuery<MPTraits>::
PerformQuery(RoadmapType* _rdmp) {
  if(m_query.empty())
    cerr << "Error in Query::PerformQuery() because m_query is empty.\n"
      << "Sometimes caused by reading the wrong query file in the XML." << endl;

  //clear out path before calling query to ensure that only portions relevant to
  //the query appear in the overall output.
  m_path.clear();
  m_pathVIDs.clear();
  const auto& start = m_query.front();
  for(typename vector<CfgType>::iterator it = m_goals.begin(); it < m_goals.end();) {
    if(this->m_debug) {
      cout << "\n*Q* query is ...     ";
      cout << start;
      cout << "\n*Q*                  ";
      cout << *it;
      cout << "\n*Q* working  ..." << endl;
    }

    if(!PerformQuery(start, *it, _rdmp)) {
      if(this->m_debug)
        cout << endl << "*Q* In RRTQuery::PerformQuery(): failed to connect";
      return false;
    }
    else {
      it = m_goals.erase(it);
    }
  }

  if(m_writePaths)
    WritePath(this->GetBaseFilename() + ".full.path", m_path);

  
  return true;
}

// Performs the query
template<class MPTraits>
bool
RRTQuery<MPTraits>::
PerformQuery(const CfgType& _start, const CfgType& _goal, RoadmapType* _rdmp) {


  if(this->m_debug)
    cout << "*Q* Begin query" << endl;
  VDComment("Begin Query");
  StatClass* stats = this->GetStatClass();
  DistanceMetricPointer dmp = this->GetDistanceMetric(m_dmLabel);
  auto nf = this->GetNeighborhoodFinder(m_nfLabel); 
  static int graphSearchCount = 0;
  LPOutput<MPTraits> sci, gci; // Connection info for start, goal nodes
  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  stats->IncGOStat("CC Operations");
  get_cc_stats(*(_rdmp->GetGraph()), cmap, ccs);
  
  if(this->m_debug)
    cout << "*Q* There are " << ccs.size() << " CCs." << endl;
  vector<pair<VID,double>> neighbors;
  
  nf->FindNeighbors(_rdmp, _rdmp->GetGraph()->begin(), _rdmp->GetGraph()->end(), true, _goal, back_inserter(neighbors));
  for(auto iter = neighbors.begin(); iter < neighbors.end(); iter++) {
    CfgType cfg = _rdmp->GetGraph()->GetVertex(iter->first);
    double dist = dmp->Distance(_goal, cfg); 
    if(dist <= m_goalDist) {
      // perform graph search
      graphSearchCount++;
      vector<VID> shortestPath;
      stats->IncGOStat("Graph Search");
      stats->StartClock("Query Graph Search");
      switch(m_searchAlg) {
        case DIJKSTRAS:
          find_path_dijkstra(*(_rdmp->GetGraph()), _rdmp->GetGraph()->GetVID(_start), iter->first, 
                shortestPath, WeightType::MaxWeight());
          break;
        case ASTAR:
          Heuristic<MPTraits> heuristic(cfg,
              this->GetEnvironment()->GetPositionRes(),
              this->GetEnvironment()->GetOrientationRes());
          astar(*(_rdmp->GetGraph()), _rdmp->GetGraph()->GetVID(_start), iter->first, 
                shortestPath, heuristic);
          break;
      }
      stats->StopClock("Query Graph Search");
      stats->AddToHistory("pathlength", CalculatePathLength(shortestPath, _rdmp));
      return true; 
    }
  }
  return false;
}

// Checks validity of path, recreates path if possible
template<class MPTraits>
bool
RRTQuery<MPTraits>::
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
RRTQuery<MPTraits>::
ReadQuery(string _filename) {
  CfgType tempCfg;
  _filename = MPProblemType::GetPath(_filename);
  ifstream in(_filename.c_str());
  if(!in) {
    cout << endl << "In ReadQuery: can't open infile: " << _filename << endl;
    return;
  }
  //in >> tempCfg;
  while(in >> tempCfg) {
    m_query.push_back(tempCfg);
   // in >> tempCfg;
  }
  copy(m_query.begin() + 1, m_query.end(), back_inserter(m_goals)); 
  
  
  GraphType* g = this->GetRoadmap()->GetGraph(); 
  for(auto& cfg : m_query)
    g->AddVertex(cfg);
}

//initialize variable defaults
template<class MPTraits>
void
RRTQuery<MPTraits>::
Initialize() {
  this->SetName("Query");
  m_searchAlg = ASTAR;
  m_dmLabel = "";
  m_fullRecreatePath = true;
}

template<class MPTraits>
void
RRTQuery<MPTraits>::
SetSearchAlgViaString(string _alg, string _where) {
  if(_alg == "dijkstras")
    m_searchAlg = DIJKSTRAS;
  else if(_alg == "astar")
    m_searchAlg = ASTAR;
  else
    throw ParseException(_where, "InValid graphSearchAlg '" + _alg +
        "'. Choices are 'dijkstras' or 'astar'.");
}

template<class MPTraits>
double
RRTQuery<MPTraits>::
CalculatePathLength(const vector<VID>& _shortestpath, RoadmapType* _rdmp) {  
  double dist = 0; 
  for(auto start = _shortestpath.begin(); start+1 < _shortestpath.end(); start++) {
    typename GraphType::edge_descriptor ed(*start, *(start + 1));
    typename GraphType::vertex_iterator vi;
    typename GraphType::adj_edge_iterator ei;
    if(_rdmp->GetGraph()->find_edge(ed, vi, ei)) {
      dist += (*ei).property().GetWeight();
    }
  }
  return dist;
}

#endif
