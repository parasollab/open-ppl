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
        const vector<string>& _connLabels = vector<string>(),
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

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Reads a query and calls the other PerformQuery()
    virtual bool PerformQuery(RoadmapType* _r);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Performs the real query work
    virtual bool PerformQuery(const CfgType& _start, const CfgType& _goal,
        RoadmapType* _r);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Read a Query file.
    virtual void ReadQuery(string _filename);

    ////////////////////////////////////////////////////////////////////////////
    void WritePath() const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Reset the paht and list of undiscovered goals.
    void Initialize(RoadmapType* _r = nullptr);

    ///@}

  protected:

    ///\name Query State
    ///@{

    string m_queryFile;         ///< The query file name.
    vector<CfgType> m_query;    ///< The start and all goal configurations.
    vector<CfgType> m_goals;    ///< The undiscovered goal configurations.
    double m_goalDist{0.};      ///< Getting at least this close = success.

    ///@}
    ///\name Path State
    ///@{

    unique_ptr<Path<MPTraits>> m_path; ///< The current path.
    bool m_fullRecreatePath{true};     ///< Create full paths or just VIDs?

    ///@}
    ///\name MP Object Labels
    ///@{

    string m_nfLabel;           ///< Neighborhood finder label.

    ///@}
    ///\name Graph State
    ///@{

    enum GraphSearchAlg {DIJKSTRAS, ASTAR}; ///< The supported sssp algorithms.
    GraphSearchAlg m_searchAlg{ASTAR};      ///< The sssp algorithm to use.

    ///@}
    ///\name Helpers
    ///@{

    void SetSearchAlgViaString(string _alg, string _where);

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(string _searchAlgo, string _dmLabel, string _nfLabel, double _goalDist) :
    m_goalDist(_goalDist), m_nfLabel(_nfLabel) {
  this->SetName("RRTQuery");
  SetSearchAlgViaString(_searchAlgo, WHERE);
}


template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(string _queryFileName, double _goalDist,
    const vector<string>& _connLabels, bool _writePaths) {
  this->SetName("RRTQuery");
  ReadQuery(_queryFileName);
  Initialize();
}


template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(MPProblemType* _problem, XMLNode& _node) :
    MapEvaluatorMethod<MPTraits>(_problem, _node) {
  this->SetName("RRTQuery");
  ParseXML(_node);
  ReadQuery(m_queryFile);
  Initialize();
}


template <typename MPTraits>
RRTQuery<MPTraits>::
RRTQuery(const CfgType& _start, const CfgType& _goal, bool _writePaths) {
  this->SetName("RRTQuery");
  m_query.push_back(_start);
  m_query.push_back(_goal);
  Initialize();
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
RRTQuery<MPTraits>::
ParseXML(XMLNode& _node) {
  m_queryFile = _node.Read("queryFile", true, "", "Query filename");
  m_nfLabel = _node.Read("nfLabel", false, "Nearest", "Neighborhood finder "
      "method");
  string searchAlg = _node.Read("graphSearchAlg", false, "dijkstras", "Graph "
      "search algorithm");
  m_fullRecreatePath = _node.Read("fullRecreatePath", false, true, "Whether or "
      "not to recreate path");
  m_goalDist = stod(_node.Read("goalDist", false, "-1.0", "Minimun Distance for "
      "valid query"));

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

  // If no goals remain, then this must be a refinement step (as in optimal
  // planning). In this case, reinitialize and rebuild the whole path.
  // We also need to rebuild when using a different roadmap.
  if(m_goals.empty() || m_path->Roadmap() != _r)
    Initialize(_r);

  // Search for a sequential path through each query point in order.
  for(auto it = m_goals.begin(); it < m_goals.end();) {
    // Start from the last reached query point.
    const auto& start = m_query[m_query.size() - m_goals.size() - 1];
    if(!PerformQuery(start, *it, _r))
      return false;
    else
      it = m_goals.erase(it);
  }
  this->GetStatClass()->AddToHistory("pathlength", m_path->Length());

  return true;
}


template <typename MPTraits>
bool
RRTQuery<MPTraits>::
PerformQuery(const CfgType& _start, const CfgType& _goal, RoadmapType* _r) {
  if(this->m_debug)
    cout << "RRTQuery::PerformQuery: evaluating query from "
         << _start << " to " << _goal << "...\n";
  VDComment("Begin Query");

  StatClass* stats = this->GetStatClass();
  GraphType* g = _r->GetGraph();
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);

  // Find nearest neighbor to _goal in the roadmap.
  stats->StartClock("RRTQuery::NeighborhoodFinding");
  vector<pair<VID,double>> neighbors;
  nf->FindNeighbors(_r, g->begin(), g->end(), true, _goal,
      back_inserter(neighbors));
  const auto& nearest = neighbors.back();
  stats->StopClock("RRTQuery::NeighborhoodFinding");

  // Check nearest neighbor.
  if(nearest.second <= m_goalDist) {
    // We are close enough. Perform graph search to generate the path.
    if(this->m_debug)
      cout << "\tVID " << nearest.first << " within goal distance ("
           << nearest.second << "/" << m_goalDist << ")." << endl;

    stats->IncGOStat("Graph Search");
    stats->StartClock("RRTQuery::GraphSearch");
    vector<VID> path;
    switch(m_searchAlg) {
      case DIJKSTRAS:
        find_path_dijkstra(*g, g->GetVID(_start), nearest.first, path,
            WeightType::MaxWeight());
        break;
      case ASTAR:
        Heuristic<MPTraits> heuristic(g->GetVertex(nearest.first),
            this->GetEnvironment()->GetPositionRes(),
            this->GetEnvironment()->GetOrientationRes());
        astar(*g, g->GetVID(_start), nearest.first, path, heuristic);
        break;
    }
    *m_path += path;
    stats->StopClock("RRTQuery::GraphSearch");
    return true;
  }
  else if(this->m_debug)
    cout << "\tFailed to connect: nearest goal was " << nearest.second
         << " units away." << endl;
  return false;
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
ReadQuery(string _filename) {
  CfgType tempCfg;
  _filename = MPProblemType::GetPath(_filename);
  ifstream in(_filename.c_str());
  if(!in) {
    cerr << endl << "In ReadQuery: can't open infile: " << _filename << endl;
    return;
  }
  //in >> tempCfg;
  while(in >> tempCfg) {
    m_query.push_back(tempCfg);
   // in >> tempCfg;
  }
}


template <typename MPTraits>
void
RRTQuery<MPTraits>::
WritePath() const {
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

/*----------------------------------------------------------------------------*/

#endif
