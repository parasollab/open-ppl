#ifndef QUERY_METHOD_H_
#define QUERY_METHOD_H_

#include "MapEvaluatorMethod.h"

#include "LocalPlanners/LPOutput.h"
#include "LocalPlanners/StraightLine.h"
#include "MPProblem/Path.h"
#include "Utilities/MetricUtils.h"
#include <containers/sequential/graph/algorithms/astar.h>


////////////////////////////////////////////////////////////////////////////////
/// \brief Heuristic for A* graph search. Uses FindIncrement to estimate the
///        distance to a given goal configuration.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
struct Heuristic {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType    CfgType;
    typedef typename MPTraits::WeightType WeightType;

    ///@}
    ///\name Construction
    ///@{

    Heuristic(const CfgType& _goal, double _posRes, double _oriRes) :
        m_goal(_goal), m_posRes(_posRes), m_oriRes(_oriRes) { }

    ///@}
    ///\name Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Return the number of ticks as the estimated distance from a given
    ///        configuration to the goal.
    /// \param[in] _c The configuration of interest.
    /// \return The estimated distance from _c to m_goal.
    WeightType operator()(const CfgType& _c) {
      int tick;
      CfgType incr;
      incr.FindIncrement(_c, m_goal, &tick, m_posRes, m_oriRes);
      return WeightType("", tick / 2);
    }

    ///@}

  private:

    ///\name Internal State
    ///@{

    CfgType m_goal;  ///< The goal configuration for this search.
    double m_posRes; ///< The position resolultion to use.
    double m_oriRes; ///< The orientation resolution to use.

    ///@}
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief Base class for all query methods. These objects evaluate a roadmap
///        under construction to see if a planning query has been satisfied.
/// @tparam MPTraits Motion planning universe
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class QueryMethod : public MapEvaluatorMethod<MPTraits> {

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

    QueryMethod();
    QueryMethod(MPProblemType* _problem, XMLNode& _node);
    virtual ~QueryMethod() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const override;

    ///@}
    ///\name MapEvaluatorMethod Overrides
    ///@{

    virtual bool operator()() override;

    ///@}
    ///\name Query Interface
    ///@{

    const vector<CfgType>& GetQuery() const {return m_query;}
    const vector<CfgType>& GetGoals() const {return m_goals;}
    const Path<MPTraits>* GetPath() const {return m_path.get();}

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Check whether a path can be drawn through all query points using
    ///        the configurations in a given roadmap.
    /// \param[in] _r The roadmap to search.
    /// \return A bool indicating whether a path in _r traversing all goals was
    ///         found.
    virtual bool PerformQuery(RoadmapType* const _r);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Check whether a path connecting a given start and goal exists in
    ///        the roadmap.
    /// \param[in] _start The starting configuration to use.
    /// \param[in] _goal  The goal configuration to use.
    /// \return A bool indicating whether the path was found.
    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal) = 0;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Read a Query file.
    /// \param[in] _filename The query file to read.
    void ReadQuery(string _filename);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Output the discovered path to file.
    void WritePath() const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Reset the path and list of undiscovered goals.
    /// \param[in] _r The roadmap to use.
    virtual void Initialize(RoadmapType* _r = nullptr);

    ///@}

  protected:

    ///\name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Set the search algorithm choice from a string.
    /// \param[in] _alg The search algorithm to use ('astar' or 'dijkstras').
    /// \param[in] _where Error location info in case _alg isn't recognized.
    void SetSearchAlgViaString(string _alg, const string& _where);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Check if a two nodes are connected by the roadmap.
    /// \param[in] _start The starting node's descriptor.
    /// \param[in] _end The ending node's descriptor.
    /// \return True if _start and _goal are connected.
    bool SameCC(const VID _start, const VID _end) const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Generate a path through the roadmap from a start node to an end
    ///        node.
    /// \param[in] _start The start node.
    /// \param[in] _end The end node.
    void GeneratePath(const VID _start, const VID _end);

    ///@}
    ///\name Query State
    ///@{

    vector<CfgType> m_query;    ///< The start and all goal configurations.
    vector<CfgType> m_goals;    ///< The undiscovered goal configurations.

    ///@}
    ///\name Path State
    ///@{

    unique_ptr<Path<MPTraits>> m_path; ///< The current path.
    bool m_fullRecreatePath{true};     ///< Create full paths or just VIDs?

    ///@}
    ///\name Graph Search
    ///@{

    enum GraphSearchAlg {DIJKSTRAS, ASTAR}; ///< The supported sssp algorithms.
    GraphSearchAlg m_searchAlg{DIJKSTRAS};  ///< The sssp algorithm to use.

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
QueryMethod<MPTraits>::
QueryMethod() : MapEvaluatorMethod<MPTraits>() {
  this->SetName("QueryMethod");
}


template <typename MPTraits>
QueryMethod<MPTraits>::
QueryMethod(MPProblemType* _problem, XMLNode& _node) :
    MapEvaluatorMethod<MPTraits>(_problem, _node) {
  this->SetName("QueryMethod");
  ParseXML(_node);

  string queryFile = this->GetMPProblem()->GetQueryFilename();
  if(!queryFile.empty())
    ReadQuery(this->GetMPProblem()->GetQueryFilename());
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
QueryMethod<MPTraits>::
ParseXML(XMLNode& _node) {
  string searchAlg = _node.Read("graphSearchAlg", false, "dijkstras", "Graph "
      "search algorithm");
  m_fullRecreatePath = _node.Read("fullRecreatePath", false, true, "Whether or "
      "not to recreate path");

  SetSearchAlgViaString(searchAlg, _node.Where());
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << "::"
      << "\n\tSearch Alg: " << m_searchAlg
      << "\n\tFull Paths: " << m_fullRecreatePath << endl;
}


template <typename MPTraits>
bool
QueryMethod<MPTraits>::
operator()() {
  return this->PerformQuery(this->GetRoadmap());
}

/*--------------------------- Query Interface --------------------------------*/

template <typename MPTraits>
bool
QueryMethod<MPTraits>::
PerformQuery(RoadmapType* const _r) {
  if(m_query.empty())
    throw RunTimeException(WHERE, this->GetNameAndLabel() + "::PerformQuery "
        "error: m_query is empty. This is sometimes caused by reading the wrong "
        "query file in the XML.");

  if(this->m_debug)
    cout << "Evaluating query, " << m_goals.size() << " goals not connected.\n";

  // If no goals remain, then this must be a refinement step (as in optimal
  // planning). In this case, reinitialize and rebuild the whole path.
  // We also need to rebuild when changing roadmaps.
  if(m_goals.empty() || m_path->GetRoadmap() != _r)
    Initialize(_r);

  // Search for a sequential path through each query point in order.
  for(auto it = m_goals.begin(); it < m_goals.end();) {
    // Start from the last reached query point.
    const auto& start = m_query[m_query.size() - m_goals.size() - 1];
    if(!PerformSubQuery(start, *it))
      return false;
    else
      it = m_goals.erase(it);
  }
  this->GetStatClass()->AddToHistory("pathlength", m_path->Length());
  WritePath();

  if(this->m_debug)
    cout << "\tConnected all goals!" << endl;

  return true;
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
ReadQuery(string _filename) {
  _filename = MPProblemType::GetPath(_filename);
  if(this->m_debug)
    cout << "Reading query file \'" << _filename << "\'..." << endl;

  ifstream in(_filename);
  if(!in.good())
    throw ParseException(WHERE, "Can't open query file '" + _filename + "'.");

  m_query.clear();
  CfgType tempCfg;
  while(in >> tempCfg)
    m_query.push_back(tempCfg);

  if(this->m_debug)
    cout << "\tWe read " << m_query.size() << " cfgs." << endl;
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
WritePath() const {
  if(!m_path)
    return;
  if(m_fullRecreatePath)
    ::WritePath(this->GetBaseFilename() + ".full.path", m_path->FullCfgs());
  else
    ::WritePath(this->GetBaseFilename() + ".rdmp.path", m_path->Cfgs());
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
Initialize(RoadmapType* _r) {
  m_goals.clear();
  copy(m_query.begin() + 1, m_query.end(), back_inserter(m_goals));

  if(!_r)
    _r = this->GetRoadmap();
  m_path = unique_ptr<Path<MPTraits>>(
      new Path<MPTraits>(this->GetMPProblem(), _r));
}

/*------------------------------- Helpers ------------------------------------*/

template <typename MPTraits>
void
QueryMethod<MPTraits>::
SetSearchAlgViaString(string _alg, const string& _where) {
  transform(_alg.begin(), _alg.end(), _alg.begin(), ::tolower);
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
QueryMethod<MPTraits>::
SameCC(const VID _start, const VID _end) const {
  if(this->m_debug)
    cout << "\tChecking connectivity..." << endl;

  auto g = m_path->GetRoadmap()->GetGraph();
  auto stats = this->GetStatClass();

  stats->IncGOStat("CC Operations");

  stats->StartClock("QueryMethod::CCTesting");
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  bool connected = is_same_cc(*g, cmap, _start, _end);
  stats->StopClock("QueryMethod::CCTesting");

  if(this->m_debug)
    cout << "\t\tNodes " << _start << " and " << _end << " are "
         << (connected ? "" : "not ") << "connected." << endl;

  return connected;
}


template <typename MPTraits>
void
QueryMethod<MPTraits>::
GeneratePath(const VID _start, const VID _end) {
  auto g = m_path->GetRoadmap()->GetGraph();
  auto stats = this->GetStatClass();
  stats->IncGOStat("Graph Search");
  stats->StartClock("QueryMethod::GraphSearch");
  vector<VID> path;
  switch(m_searchAlg) {
    case DIJKSTRAS:
      find_path_dijkstra(*g, _start, _end, path, WeightType::MaxWeight());
      break;
    case ASTAR:
      Heuristic<MPTraits> heuristic(g->GetVertex(_end),
          this->GetEnvironment()->GetPositionRes(),
          this->GetEnvironment()->GetOrientationRes());
      astar(*g, _start, _end, path, heuristic);
      break;
  }
  *m_path += path;
  stats->StopClock("QueryMethod::GraphSearch");
}

/*----------------------------------------------------------------------------*/

#endif
