#ifndef RRT_QUERY_H_
#define RRT_QUERY_H_

#include "MapEvaluatorMethod.h"

#include "LocalPlanners/LPOutput.h"
#include "LocalPlanners/StraightLine.h"
#include "MapEvaluators/Query.h"
#include "MPProblem/Path.h"
#include "Utilities/MetricUtils.h"
#include <containers/sequential/graph/algorithms/astar.h>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RRTQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::CfgRef           CfgRef;
    typedef typename MPTraits::WeightType       WeightType;
    typedef typename MPTraits::MPProblemType    MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType   GraphType;
    typedef typename MPProblemType::VID         VID;

    ///@}
    ///\name Construction
    ///@{

    RRTQuery(string _searchAlgo = "astar", string _nfLabel = "",
        double _goalDist = 0.);
    RRTQuery(string _queryFileName, double _goalDist = 0.,
        const string& _nfLabel = "",
        bool _writePaths = true);
    RRTQuery(const CfgType& _start, const CfgType& _goal,
        bool _writePaths = true);
    RRTQuery(MPProblemType* _problem, XMLNode& _node);

    virtual ~RRTQuery() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const override;
    virtual bool operator()() override;

    ///@}
    ///\name Query Interface
    ///@{

    const vector<CfgType>& GetQuery() const {return m_query;}
    const vector<CfgType>& GetGoals() const {return m_goals;}
    const Path<MPTraits>& GetPath() const {return m_path;}
    const CfgType& GetRandomGoal() const {
      if(m_goals.empty())
        throw RunTimeException(WHERE, "Random goal requested, but none are "
            "available.");
      return m_goals[LRand() % m_goals.size()];
    }

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Checks whether a path can be drawn through all query points using
    ///        the configurations in a given roadmap.
    virtual bool PerformQuery(RoadmapType* _r);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Checks whether a path can be drawn from start to goal using the
    ///        configurations in a given roadmap.
    virtual bool PerformQuery(RoadmapType* _r, const CfgType& _start,
        const CfgType& _goal);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Read a Query file.
    virtual void ReadQuery(string _filename);

    ////////////////////////////////////////////////////////////////////////////
    void WritePath() const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Reset the path and list of undiscovered goals.
    void Initialize(RoadmapType* _r = nullptr);

    ///@}

  protected:

    ///\name Query State
    ///@{

    string m_queryFile;         ///< The query file name.
    vector<CfgType> m_query;    ///< The start and all goal configurations.
    vector<CfgType> m_goals;    ///< The undiscovered goal configurations.
    double m_goalDist{0.};      ///< Getting at least this close = success.
    VID m_highestCheckedVID;    ///< The highest VID we have tried to connect to
                                ///< the goal.

    ///@}
    ///\name Path State
    ///@{

    unique_ptr<Path<MPTraits>> m_path; ///< The current path.
    bool m_fullRecreatePath{true};     ///< Create full paths or just VIDs?

    ///@}
    ///\name MP Object Labels
    ///@{

    string m_nfLabel;           ///< Neighborhood finder label.
    string m_exLabel;           ///< Extender label.
  
    ///@}
    ///\name Graph State
    ///@{

    enum GraphSearchAlg {DIJKSTRAS, ASTAR}; ///< The supported sssp algorithms.
    GraphSearchAlg m_searchAlg{ASTAR};      ///< The sssp algorithm to use.

    ///@}
    ///\name Helpers
    ///@{

    void SetSearchAlgViaString(string _alg, string _where);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Check if a start and goal node are connected by a given roadmap.
    bool SameCC(RoadmapType* _r, const VID _start, const VID _goal) const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Find the nearest node in _r to _goal, assuming that _goal is not
    ///        already in _r.
    pair<VID, double> FindNearestNeighbor(RoadmapType* _r,
        const CfgType& _goal);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Find the node in _r that is nearest to _goal and connected to
    ///        _start.
    pair<VID, double> FindNearestConnectedNeighbor(RoadmapType* _r,
        const CfgType& _start, const CfgType& _goal);

    void GeneratePath(RoadmapType* _r, const CfgType& _start,
        const pair<VID, double>& _nearest);

    pair<VID,double> ExtendToGoal(RoadmapType* _r,
        const pair<VID, double>& _nearest, const CfgType& _goal) const;

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(string _searchAlgo, string _nfLabel, double _goalDist) :
    m_goalDist(_goalDist), m_nfLabel(_nfLabel) {
  this->SetName("RRTQuery");
  SetSearchAlgViaString(_searchAlgo, WHERE);
}


template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(string _queryFileName, double _goalDist,
    const string& _nfLabel, bool _writePaths) {
  this->SetName("RRTQuery");
  ReadQuery(_queryFileName);
  m_nfLabel = _nfLabel;
}


template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(MPProblemType* _problem, XMLNode& _node) :
    MapEvaluatorMethod<MPTraits>(_problem, _node) {
  this->SetName("RRTQuery");
  ParseXML(_node);
  ReadQuery(m_queryFile);
}


template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(const CfgType& _start, const CfgType& _goal, bool _writePaths) {
  this->SetName("RRTQuery");
  m_query.push_back(_start);
  m_query.push_back(_goal);
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
RRTQuery<MPTraits>::
ParseXML(XMLNode& _node) {
  m_queryFile = _node.Read("queryFile", true, "", "Query filename");
  m_nfLabel = _node.Read("nfLabel", false, "Nearest", "Neighborhood finder "
      "method");
  m_exLabel = _node.Read("exLabel", true, "BERO", "Extender method");
  string searchAlg = _node.Read("graphSearchAlg", false, "dijkstras", "Graph "
      "search algorithm");
  m_fullRecreatePath = _node.Read("fullRecreatePath", false, true, "Whether or "
      "not to recreate path");
  m_goalDist = _node.Read("goalDist", false, 0., 0.,
      numeric_limits<double>::max(), "Minimun Distance for valid query");

  // Ignore case for graph search algorithm
  transform(searchAlg.begin(), searchAlg.end(), searchAlg.begin(), ::tolower);
  SetSearchAlgViaString(searchAlg, _node.Where());
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << "::"
      << "\n\tquery file = \"" << m_queryFile << "\""
      << "\n\tsearch alg = " << m_searchAlg
      << "\n\tfullRecreatePath = " << m_fullRecreatePath << endl;
}


template <typename MPTraits>
bool
RRTQuery<MPTraits>::
operator()() {
  return PerformQuery(this->GetRoadmap());
}

/*--------------------------- Query Interface --------------------------------*/

template <typename MPTraits>
bool
RRTQuery<MPTraits>::
PerformQuery(RoadmapType* _r) {
  if(m_query.empty())
    throw RunTimeException(WHERE, "RRTQuery::PerformQuery error: m_query is "
        "empty. This is sometimes caused by reading the wrong query file in the "
        "XML.");

  if(this->m_debug)
    cout << "Evaluating query with " << m_goals.size() << " goals not found.\n";

  // If no goals remain, then this must be a refinement step (as in optimal
  // planning). In this case, reinitialize and rebuild the whole path.
  // We also need to rebuild when using a different roadmap.
  if(m_goals.empty() || m_path->GetRoadmap() != _r)
    Initialize(_r);

  // Search for a sequential path through each query point in order.
  for(auto it = m_goals.begin(); it < m_goals.end();) {
    // Start from the last reached query point.
    const auto& start = m_query[m_query.size() - m_goals.size() - 1];
    if(!PerformQuery(_r, start, *it))
      return false;
    else
      it = m_goals.erase(it);
  }
  this->GetStatClass()->AddToHistory("pathlength", m_path->Length());

  if(this->m_debug)
    cout << "\tQuery found all goals!" << endl;

  return true;
}


template <typename MPTraits>
bool
RRTQuery<MPTraits>::
PerformQuery(RoadmapType* _r, const CfgType& _start, const CfgType& _goal) {
  if(this->m_debug)
    cout << "Evaluating sub-query:" << endl
         << "\tfrom " << _start << endl
         << "\tto   " << _goal << endl;
  VDComment("Begin Query");

  pair<VID, double> nearest;
  bool success = false;

  // Find the nearest node to _goal that is also connected to _start.
  nearest = FindNearestConnectedNeighbor(_r, _start, _goal);
  if(nearest.first == INVALID_VID)
    // If the nearest node is invalid, it means that the goal is in the map and
    // not connected to start. In this case, we can't connect.
    success = false;
  else if(nearest.second <= m_goalDist) 
    // The nearest node is within the goal distance, so we are close enough.
    success = true;
  else {
    // The nearest node is too far and the goal isn't already connected. Try to
    // extend toward goal if we are within extender's delta range. If we can't
    // extend, we can't connect.
    nearest = ExtendToGoal(_r, nearest, _goal);
    success = nearest.first != INVALID_VID && nearest.second <= m_goalDist;
  }

  if(success) {
    GeneratePath(_r, _start, nearest);
    if(this->m_debug)
      cout << "\tSuccess: found path from start to nearest node "
           << nearest.first << " at a distance of " << nearest.second
           << " from the goal." << endl;
  }
  else if(this->m_debug)
    cout << "\tFailed to connect (goal distance is " << m_goalDist << ").\n";
  return success;
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
ReadQuery(string _filename) {
  if(this->m_debug)
    cout << "Reading query file \'" << _filename << "\'..." << endl;
  CfgType tempCfg;
  _filename = MPProblemType::GetPath(_filename);
  ifstream in(_filename.c_str());
  if(!in)
    throw ParseException(WHERE, "Can't open infile: " + _filename + ".");
  while(in >> tempCfg)
    m_query.push_back(tempCfg);
  if(this->m_debug)
    cout << "\tWe read " << m_query.size() << " cfgs." << endl;
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
WritePath() const {
  if(m_path)
    ::WritePath(this->GetBaseFilename() + ".full.path", m_path->Cfgs());
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
Initialize(RoadmapType* _r) {
  m_goals.clear();
  copy(m_query.begin() + 1, m_query.end(), back_inserter(m_goals));

  if(!_r)
    _r = this->GetRoadmap();
  m_path = unique_ptr<Path<MPTraits>>(new Path<MPTraits>(_r));
  m_highestCheckedVID = 0;
}

/*------------------------------- Helpers ------------------------------------*/

template <typename MPTraits>
void
RRTQuery<MPTraits>::
SetSearchAlgViaString(string _alg, string _where) {
  if(_alg == "dijkstras")
    m_searchAlg = DIJKSTRAS;
  else if(_alg == "astar")
    m_searchAlg = ASTAR;
  else
    throw ParseException(_where, "Invalid graph search algorithm '" + _alg +
        "'. Choices are 'dijkstras' or 'astar'.");
}


template <typename MPTraits>
bool
RRTQuery<MPTraits>::
SameCC(RoadmapType* _r, const VID _start, const VID _goal) const {
  if(this->m_debug)
    cout << "\t\tChecking connectivity..." << endl;

  auto g = _r->GetGraph();
  auto stats = this->GetStatClass();

  stats->StartClock("RRTQuery::CCTesting");
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  bool connected = is_same_cc(*g, cmap, _start, _goal);
  stats->StopClock("RRTQuery::CCTesting");

  if(this->m_debug)
    cout << "\t\t\tThe start and goal are "
         << (connected ? "" : "not ") << "connected." << endl;

  return connected;
}


template <typename MPTraits>
pair<typename MPTraits::MPProblemType::VID, double>
RRTQuery<MPTraits>::
FindNearestNeighbor(RoadmapType* _r, const CfgType& _goal) {
  if(this->m_debug)
    cout << "\t\tFinding the nearest node to the goal..." << endl;

  auto g = _r->GetGraph();

  VID highestVID = g->get_num_vertices() - 1;

  // Quit if we have already tried with the current set of vertices.
  if(m_highestCheckedVID == highestVID) {
    if(this->m_debug)
      cout << "\t\t\tAll nodes have already been checked." << endl;
    return make_pair(INVALID_VID, numeric_limits<double>::max());
  }
  else if(this->m_debug)
    cout << "\t\t\tNodes 0 through " << m_highestCheckedVID
         << " have already been checked." << endl
         << "\t\t\tSearching nodes " << m_highestCheckedVID + 1 << " through "
         << highestVID << "..." << endl;

  auto stats = this->GetStatClass();

  // If we haven't tried all vertexes, find the nearest untried vertex.
  stats->StartClock("RRTQuery::NeighborhoodFinding");
  vector<pair<VID, double>> neighbors;
  this->GetNeighborhoodFinder(m_nfLabel)->FindNeighbors(_r,
      ++g->find_vertex(m_highestCheckedVID), g->end(), true, _goal,
      back_inserter(neighbors));
  m_highestCheckedVID = highestVID;
  stats->StopClock("RRTQuery::NeighborhoodFinding");

  if(this->m_debug)
    cout << "\t\t\tFound nearest node " << neighbors.back().first << " at "
         << "distance " << neighbors.back().second << "." << endl;

  return neighbors.back();
}


template <typename MPTraits>
pair<typename MPTraits::MPProblemType::VID, double>
RRTQuery<MPTraits>::
FindNearestConnectedNeighbor(RoadmapType* _r, const CfgType& _start,
    const CfgType& _goal) {
  if(this->m_debug)
    cout << "\tSearching for the nearest connected neighbor..." << endl;

  auto g = _r->GetGraph();
  pair<VID, double> nearest;

  if(g->IsVertex(_goal)) {
    // The goal is already in the roadmap. It is it's own neighbor if it shares
    // a CC with _start and disconnected otherwise.
    VID start = g->GetVID(_start);
    VID goal = g->GetVID(_goal);
    if(goal == INVALID_VID)
      throw RunTimeException(WHERE, "goal cannot be invalid");
    if(SameCC(_r, start, goal))
      nearest = make_pair(goal, 0);
    else
      nearest = make_pair(INVALID_VID, numeric_limits<double>::max());
  }
  else
    // If _goal isn't already connected, find the nearest connected node.
    nearest = FindNearestNeighbor(_r, _goal);

  if(this->m_debug) {
    if(nearest.first != INVALID_VID)
      cout << "\t\tClosest neighbor to goal is node " << nearest.first << " at "
           << "distance " << setprecision(4) << nearest.second << ".\n";
    else
      cout << "\t\tNo valid node was found." << endl;
  }
  return nearest;
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
GeneratePath(RoadmapType* _r, const CfgType& _start,
    const pair<VID, double>& _nearest) {
  if(this->m_debug)
    cout << "\tVID " << _nearest.first << " within goal distance ("
         << _nearest.second << "/" << m_goalDist << ")." << endl;

  auto g = _r->GetGraph();
  auto stats = this->GetStatClass();
  stats->IncGOStat("Graph Search");
  stats->StartClock("RRTQuery::GraphSearch");
  vector<VID> path;
  switch(m_searchAlg) {
    case DIJKSTRAS:
      find_path_dijkstra(*g, g->GetVID(_start), _nearest.first, path,
          WeightType::MaxWeight());
      break;
    case ASTAR:
      Heuristic<MPTraits> heuristic(g->GetVertex(_nearest.first),
          this->GetEnvironment()->GetPositionRes(),
          this->GetEnvironment()->GetOrientationRes());
      astar(*g, g->GetVID(_start), _nearest.first, path, heuristic);
      break;
  }
  *m_path += path;
  stats->StopClock("RRTQuery::GraphSearch");
}


template <typename MPTraits>
pair<typename MPTraits::MPProblemType::VID, double>
RRTQuery<MPTraits>::
ExtendToGoal(RoadmapType* _r, const pair<VID, double>& _nearest,
    const CfgType& _goal) const {
  auto g = _r->GetGraph();
  auto e = this->GetExtender(m_exLabel);

  VID newVID = INVALID_VID;
  double distance = numeric_limits<double>::max();

  // If the nearest node is outside the extender's range, return invalid.
  if(_nearest.second > e->GetMaxDistance())
    return make_pair(newVID, distance);

  if(this->m_debug)
    cout << "\tTrying extension from node " << _nearest.first
         << " toward goal.\n";

  // Otherwise, try to extend from _nearest to _goal.
  CfgType qNew;
  LPOutput<MPTraits> lpOutput;
  if(e->Extend(g->GetVertex(_nearest.first), _goal, qNew, lpOutput)) {
    distance = lpOutput.m_edge.first.GetWeight(); 
    // If we went far enough, add the new node and edge.
    if(distance > e->GetMinDistance() && !g->IsVertex(qNew)) {
      newVID = g->AddVertex(qNew);
      qNew.SetStat("Parent", _nearest.first);
      g->AddEdge(_nearest.first, newVID, lpOutput.m_edge);
      // Using the NeighborhoodFinder's Distance metric for consistancy in
      // distance 
      distance = this->GetNeighborhoodFinder(m_nfLabel)->GetDMMethod()->Distance(qNew, _goal); 
      if(this->m_debug)
        cout << "\t\tExtension succeeded, created new node " << newVID << " at "
             << "distance " << distance << " from goal." << endl;
    }
    else if(this->m_debug)
      cout << "\t\tExtension failed." << endl;
  }

  return make_pair(newVID, distance);
}

/*----------------------------------------------------------------------------*/

#endif
