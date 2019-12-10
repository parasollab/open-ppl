#ifndef PMPL_CONNECTION_METHOD_H_
#define PMPL_CONNECTION_METHOD_H_

#include "ConfigurationSpace/Weight.h"

#include "MPLibrary/LocalPlanners/LocalPlannerMethod.h"
#include "MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethod.h"
#include "MPLibrary/MPBaseObject.h"
#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"

#include <boost/mpl/for_each.hpp>

#include <set>


namespace pmpl_detail {

  //////////////////////////////////////////////////////////////////////////////
  /// Facilitate function calls for general connection methods.
  /// @tparam CM Connector method derived class type
  /// @tparam RDMP Roadmap type
  /// @tparam I1 Input iterator 1 type
  /// @tparam I2 Input iterator 2 type
  /// @tparam O Output iterator tyep
  ///
  /// Facilitates simulation of pure virtualism of template functions in C++,
  /// which is explicitly disallowed for the language. Essentially iterates over
  /// a type list and attempts dynamic casts to that type to find appropriate
  /// derived class.
  /// @ingroup Connectors
  //////////////////////////////////////////////////////////////////////////////
  template <typename CM, typename RDMP, typename I1, typename I2, typename O>
  struct VirtualConnect final {

    ///@name Construction
    ///@{

    VirtualConnect(CM* _v, RDMP* _r, I1 _i1f, I1 _i1l, I2 _i2f, I2 _i2l,
        bool _b, O _o) :
        m_memory(_v), m_rdmp(_r), m_i1first(_i1f), m_i1last(_i1l),
        m_i2first(_i2f), m_i2last(_i2l), m_fromFullRoadmap(_b), m_output(_o) { }

    ///@}
    ///@name Interface
    ///@{

    template<typename T>
    void operator()(T& _t) {
      T* tptr = dynamic_cast<T*>(m_memory);
      if(tptr)
        tptr->Connect(m_rdmp, m_i1first, m_i1last,
            m_i2first, m_i2last, m_fromFullRoadmap, m_output);
    }

    ///@}

    private:

      ///@name Internal State
      ///@{

      CM* m_memory;
      RDMP* m_rdmp;
      I1 m_i1first, m_i1last;
      I2 m_i2first, m_i2last;
      bool m_fromFullRoadmap;
      O m_output;

      ///@}
  };

}


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref Connectors. Connectors attempt to
/// connect each of a first set of roadmap vertices to each of a second set of
/// roadmap vertices (successes generate new edges in the roadmap).
///
/// @todo Adjust connection caching so that it considers the originating roadmap
///       as well. Currently it will give incorrect results if we use the same
///       connector on more than one roadmap in a problem run.
///
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ConnectorMethod : public MPBaseObject<MPTraits>
{
  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID            VID;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::pair<VID, VID>          ConnectionAttempt;
    typedef std::set<ConnectionAttempt>  ConnectionAttemptsCache;

    ///@}
    ///@name Construction
    ///@{

    ConnectorMethod();

    ConnectorMethod(XMLNode& _node);

    virtual ~ConnectorMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}
    ///@name Connection Interface
    ///@{

    /// Generate edges between two sets of nodes. Uses the entire roadmap as the
    /// first and second set of nodes.
    /// @param _r The roadmap to connect.
    /// @param _collision Output for obstacle cfgs discovered during connection
    ///                   attempts.
    template <typename AbstractRoadmapType,
              typename OutputIterator = NullOutputIterator>
    void Connect(AbstractRoadmapType* _r,
        OutputIterator _collision = OutputIterator());

    /// Generate edges between two sets of nodes. Uses a single node as the
    /// first set of nodes and the entire roadmap as second set of nodes.
    /// @param _r The roadmap to connect.
    /// @param _vid The vertex to connect to the rest of the roadmap.
    /// @param _collision Output for obstacle cfgs discovered during connection
    ///                   attempts.
    template <typename AbstractRoadmapType,
              typename OutputIterator = NullOutputIterator>
    void Connect(AbstractRoadmapType* _r, VID _vid,
        OutputIterator _collision = OutputIterator());

    /// Generate edges between two sets of nodes. Uses entire roadmap as second
    /// set of nodes.
    /// @param _r The roadmap to connect.
    /// @param _itrFirst Begin iterator to the first set of nodes.
    /// @param _itrLast  End iterator to the first set of nodes.
    /// @param _collision Output for obstacle cfgs discovered during connection
    ///                   attempts.
    template <typename AbstractRoadmapType, typename InputIterator,
              typename OutputIterator = NullOutputIterator>
    void Connect(AbstractRoadmapType* _r, InputIterator _itrFirst,
        InputIterator _itrLast, OutputIterator _collision = OutputIterator());

    /// Generate edges between two sets of nodes. Uses a single VID as the first
    /// set of nodes.
    /// @param _r The roadmap to connect.
    /// @param _vid The vertex to connect to the second set of nodes.
    /// @param _itrFirst Begin iterator to the second set of nodes.
    /// @param _itrLast  End iterator to the second set of nodes.
    /// @param _fromFullRoadmap True if [_itrFirst, _itrLast) is the full
    ///                         roadmap. This implies using the saved internal
    ///                         NF model for finding neighbors with advanced
    ///                         NFMethods.
    /// @param _collision Output for obstacle cfgs discovered during connection
    ///                   attempts.
    template <typename AbstractRoadmapType, typename InputIterator,
              typename OutputIterator = NullOutputIterator>
    void Connect(AbstractRoadmapType* _r, VID _vid,
        InputIterator _itrFirst, InputIterator _itrLast,
        bool _fromFullRoadmap, OutputIterator _collision = OutputIterator());

    /// Generate edges between two sets of nodes.
    /// @param _r The roadmap to generate edges in and where the input nodes
    ///           are found
    /// @param _itr1First Begin iterator of first set of VIDs
    /// @param _itr1Last End iterator of first set of VIDs
    /// @param _itr2First Begin iterator of second set of VIDs
    /// @param _itr2Last End iterator of second set of VIDs
    /// @param _fromFullRoadmap True if [_itr2First, _itr2Last) is the full
    ///                         roadmap. This implies using the saved internal
    ///                         NF model for finding neighbors with advanced
    ///                         NF methods.
    /// @param _collision Output iterator to store collision witnesses
    template <typename AbstractRoadmapType, typename InputIterator1,
              typename InputIterator2,
              typename OutputIterator = NullOutputIterator>
    void Connect(AbstractRoadmapType* _r,
        InputIterator1 _itr1First, InputIterator1 _itr1Last,
        InputIterator2 _itr2First, InputIterator2 _itr2Last,
        bool _fromFullRoadmap,
        OutputIterator _collision = OutputIterator());

    /// Check whether this is a rewiring connector (which may delete edges).
    /// @return True if this is a rewiring connector.
    bool IsRewiring() const noexcept;

    ///@}

  protected:

    ///@name Connection Helpers
    ///@{

    /// Try to connect a given configuration to each cfg in an input set.
    /// @param _r The roadmap.
    /// @param _source The configuration.
    /// @param _first The begin iterator in the range of other cfgs.
    /// @param _last The end iterator in the range of other cfgs.
    /// @param _collision Output for invalid configurations.
    template <typename AbstractRoadmapType, typename InputIterator,
              typename OutputIterator>
    void ConnectNeighbors(
        AbstractRoadmapType* _r, VID _source,
        InputIterator _first, InputIterator _last,
        OutputIterator _collision);

    /// Attempt a connection between two individual configurations.
    /// @param _r The roadmap.
    /// @param _source The source configuration's VID.
    /// @param _target The target configuration's VID.
    /// @param _collision Output for invalid configurations.
    /// @return True if the connection succeeded.
    template <typename OutputIterator>
    bool ConnectNodes(RoadmapType* _r, const VID _source, const VID _target,
        OutputIterator _collision);

    /// Attempt a connection between two group configurations.
    /// @param _r The group roadmap.
    /// @param _source The source configuration's VID.
    /// @param _target The target configuration's VID.
    /// @param _collision Output for invalid configurations.
    /// @return True if the connection succeeded.
    template <typename OutputIterator>
    bool ConnectNodes(GroupRoadmapType* _r, const VID _source,
        const VID _target, OutputIterator _collision);

    /// Check whether a connection should be attempted.
    /// @param _r The roadmap.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @return True if the connection should not be attempted.
    template <typename AbstractRoadmapType>
    bool DoNotCheck(AbstractRoadmapType* _r, const VID _source,
        const VID _target) const;

    ///@}
    ///@name Connection Caching
    ///@{

    /// Add a failed connection attempt to the cache.
    /// @param _source The source VID.
    /// @param _target The target VID.
    void CacheFailedConnection(const VID _source, const VID _target) noexcept;

    /// Check if attempt is in the failure cache.
    /// @param _source The source VID.
    /// @param _target The target VID.
    /// @return True if the attempt to connect _source to _target has already
    ///         failed.
    bool IsCached(const VID _source, const VID _target) const noexcept;

    /// Clear the attempts cache.
    void ClearConnectionAttempts();

    ///@}
    ///@name Internal State
    ///@{

    ConnectionAttemptsCache m_attemptsCache; ///< All failed connection attempts.
    std::string m_lpLabel;                   ///< Local Planner

    bool m_skipIfSameCC{true};  ///< Skip connections within the same CC?
    size_t m_maxFailures{0};    ///< Maximum allowed failures per iteration.

    bool m_rewiring{false};     ///< Does this connector delete edges?

		bool m_selfEdges{false}; 		///< Indicates if roadmap vertices should have self-edges.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ConnectorMethod<MPTraits>::
ConnectorMethod()
{ }


template <typename MPTraits>
ConnectorMethod<MPTraits>::
ConnectorMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  m_lpLabel = _node.Read("lpLabel", true, "", "Local Planner");

  m_skipIfSameCC = _node.Read("checkIfSameCC", false, m_skipIfSameCC,
      "Skip connections between nodes in the same CC?");

  m_maxFailures = _node.Read("maxFailures", false, m_maxFailures,
      size_t(0), std::numeric_limits<size_t>::max(),
      "Terminate Connect operations after this many failures (0 to disable).");
	
	m_selfEdges = _node.Read("selfEdges", false, m_selfEdges,
			"Indicates if the connector should allow self edges.");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
ConnectorMethod<MPTraits>::
Print(std::ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\n\tlpLabel: " << m_lpLabel
      << "\n\tskip if same cc: " << m_skipIfSameCC
      << "\n\tmax failures: " << m_maxFailures
      << std::endl;
}


template <typename MPTraits>
void
ConnectorMethod<MPTraits>::
Initialize() {
  ClearConnectionAttempts();
}

/*---------------------------- Connection Interface --------------------------*/

template <typename MPTraits>
template <typename AbstractRoadmapType, typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(AbstractRoadmapType* _r, OutputIterator _collision) {
  Connect(_r, _r->begin(), _r->end(), _r->begin(), _r->end(), true, _collision);
}


template <typename MPTraits>
template <typename AbstractRoadmapType, typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(AbstractRoadmapType* _r, VID _vid, OutputIterator _collision) {
  Connect(_r, _vid, _r->begin(), _r->end(), true, _collision);
}


template <typename MPTraits>
template <typename AbstractRoadmapType, typename InputIterator,
          typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(AbstractRoadmapType* _r, InputIterator _itrFirst, InputIterator _itrLast,
    OutputIterator _collision) {
  Connect(_r, _itrFirst, _itrLast, _r->begin(), _r->end(), true, _collision);
}


template <typename MPTraits>
template <typename AbstractRoadmapType, typename InputIterator,
          typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(AbstractRoadmapType* _r, VID _vid, InputIterator _itrFirst,
    InputIterator _itrLast, bool _fromFullRoadmap, OutputIterator _collision) {
  std::vector<VID> vids(1, _vid);
  Connect(_r, vids.begin(), vids.end(), _itrFirst, _itrLast,
      _fromFullRoadmap, _collision);
}


template <typename MPTraits>
template <typename AbstractRoadmapType, typename InputIterator1,
          typename InputIterator2,
          typename OutputIterator>
void
ConnectorMethod<MPTraits>::
Connect(AbstractRoadmapType* _r,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    bool _fromFullRoadmap, OutputIterator _collision) {
  MethodTimer mt(this->GetStatClass(), this->GetName() + "::Connect");

  typedef typename MPTraits::ConnectorMethodList MethodList;
  boost::mpl::for_each<MethodList>(
      pmpl_detail::VirtualConnect<
      ConnectorMethod<MPTraits>, AbstractRoadmapType,
      InputIterator1, InputIterator2, OutputIterator>(
        this, _r, _itr1First, _itr1Last,
        _itr2First, _itr2Last, _fromFullRoadmap, _collision)
      );
}


template <typename MPTraits>
inline
bool
ConnectorMethod<MPTraits>::
IsRewiring() const noexcept {
  return m_rewiring;
}

/*--------------------------- Connection Helpers -----------------------------*/

template <typename MPTraits>
template <typename AbstractRoadmapType, typename InputIterator,
          typename OutputIterator>
void
ConnectorMethod<MPTraits>::
ConnectNeighbors(AbstractRoadmapType* _r, VID _source,
    InputIterator _first, InputIterator _last, OutputIterator _collision) {
  size_t failCount = 0;

  // Try to connect each node in _first, _last to _source.
  for(auto iter = _first; iter != _last; ++iter) {
    // Check if we have exceeded the failure limit.
    if(m_maxFailures and failCount >= m_maxFailures) {
      if(this->m_debug)
        std::cout << "\t\tFailures (" << failCount << ") exceed threshold "
                  << "of " << m_maxFailures << ", stopping."
                  << std::endl;
      return;
    }

    // Attempt the next connection.
    const VID target = iter->target;
    if(this->m_debug)
      std::cout << "\t\tAttempting connection to " << target
                << " at distance " << iter->distance
                << std::endl;

    // Check if this attempt should be skipped.
    if(DoNotCheck(_r, _source, target))
      continue;

    // Attempt connection with the local planner.
    const bool connected = this->ConnectNodes(_r, _source, target, _collision);
    failCount += !connected;

    if(this->m_debug)
      std::cout << "\t\tConnection " << (connected ? "succeeded" : "failed")
                << "." << std::endl;
  }

	//Connect vertices to themselves
	if(m_selfEdges) {
		this->ConnectNodes(_r, _source, _source, _collision);
	}
}


template <typename MPTraits>
template <typename OutputIterator>
bool
ConnectorMethod<MPTraits>::
ConnectNodes(RoadmapType* _r, const VID _source, const VID _target,
    OutputIterator _collision) {
  auto env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  auto lp = this->GetLocalPlanner(m_lpLabel);

  const CfgType& c1 = _r->GetVertex(_source),
               & c2 = _r->GetVertex(_target);

  CfgType collision(robot);
  LPOutput<MPTraits> lpOutput;
  const bool connected = lp->IsConnected(c1, c2, collision, &lpOutput,
        env->GetPositionRes(), env->GetOrientationRes(), true);

  if(connected)
    _r->AddEdge(_source, _target, lpOutput.m_edge);
  else {
    CacheFailedConnection(_source, _target);
    *_collision++ = collision;
  }

  return connected;
}


template <typename MPTraits>
template <typename OutputIterator>
bool
ConnectorMethod<MPTraits>::
ConnectNodes(GroupRoadmapType* _r, const VID _source, const VID _target,
    OutputIterator _collision) {
  auto env = this->GetEnvironment();
  auto lp = this->GetLocalPlanner(m_lpLabel);

  const GroupCfgType& c1 = _r->GetVertex(_source),
                    & c2 = _r->GetVertex(_target);

  GroupCfgType collision(_r);
  GroupLPOutput<MPTraits> lpOutput(_r);
  const bool connected = lp->IsConnected(c1, c2, collision, &lpOutput,
        env->GetPositionRes(), env->GetOrientationRes(), true);

  if(connected)
    _r->AddEdge(_source, _target, lpOutput.m_edge);
  else {
    CacheFailedConnection(_source, _target);
    *_collision++ = collision;
  }

  return connected;
}


template <typename MPTraits>
template <typename AbstractRoadmapType>
bool
ConnectorMethod<MPTraits>::
DoNotCheck(AbstractRoadmapType* _r, const VID _source, const VID _target) const {
  const std::string indent = "\t\t\t";

  std::string connection;
  if(this->m_debug)
    connection = "(" + std::to_string(_source) + ", "
               + std::to_string(_target) + ")";

  // Check that both VIDs are valid.
  if(_source == INVALID_VID or _target == INVALID_VID) {
    if(this->m_debug)
      std::cout << indent
                << "Skipping connection with invalid node " << connection << "."
                << std::endl;
    return true;
  }

  // Check for self-connection.
  if(_source == _target and !m_selfEdges) {
    if(this->m_debug)
      std::cout << indent
                << "Skipping self-connection " << connection << "."
                << std::endl;
    return true;
  }

  // Check for cached connection.
  if(this->IsCached(_source, _target)) {
    if(this->m_debug)
      std::cout << indent
                << "Skipping cached failed connection " << connection << "."
                << std::endl;
    return true;
  }

  // Check if the edge already exists.
  if(_r->IsEdge(_source, _target)) {
    if(this->m_debug)
      std::cout << indent
                << "Skipping existing connection " << connection << "."
                << std::endl;
    return true;
  }

  // Check if the nodes are in the same connected component.
  if(m_skipIfSameCC) {
    typename AbstractRoadmapType::ColorMap colorMap;
    if(stapl::sequential::is_same_cc(*_r, colorMap, _source, _target)) {
      if(this->m_debug)
        std::cout << indent
                  << "Skipping connection within the same connected component "
                  << connection << "."
                  << std::endl;
      return true;
    }
  }

  return false;
}


/*------------------------------ Connection Caching --------------------------*/

template <typename MPTraits>
void
ConnectorMethod<MPTraits>::
CacheFailedConnection(const VID _source, const VID _target) noexcept {
  m_attemptsCache.insert({_source, _target});
}


template <typename MPTraits>
bool
ConnectorMethod<MPTraits>::
IsCached(const VID _source, const VID _target) const noexcept {
  const bool cached = m_attemptsCache.count({_source, _target});
  this->GetStatClass()->GetAverage(this->GetName() + "::CacheHitRatio") += cached;
  return cached;
}


template <typename MPTraits>
void
ConnectorMethod<MPTraits>::
ClearConnectionAttempts() {
  m_attemptsCache.clear();
}

/*----------------------------------------------------------------------------*/

#endif
