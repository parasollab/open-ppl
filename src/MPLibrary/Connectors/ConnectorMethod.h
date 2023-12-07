#ifndef PMPL_CONNECTION_METHOD_H_
#define PMPL_CONNECTION_METHOD_H_

#include "ConfigurationSpace/Weight.h"

#include "MPLibrary/LocalPlanners/LocalPlannerMethod.h"
#include "MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethod.h"
#include "MPLibrary/MPBaseObject.h"
#include "Utilities/Hash.h"
#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"

#include <iterator>
#include <set>


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref Connectors. Connectors attempt to
/// connect a set of 'source' roadmap vertices to each of a second 'target' set
/// of vertices. Successes generate new edges in the roadmap.
///
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
class ConnectorMethod : public MPBaseObject
{
  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType      GroupCfgType;
    typedef typename MPBaseObject::RoadmapType       RoadmapType;
    typedef typename MPBaseObject::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::pair<VID, VID>                    ConnectionAttempt;
    typedef std::unordered_set<ConnectionAttempt>  ConnectionAttempts;
    typedef std::unordered_map<void*, ConnectionAttempts> ConnectionAttemptsCache;

    template <typename AbstractRoadmapType>
    using OutputIterator = std::back_insert_iterator<
                             std::vector<
                               typename AbstractRoadmapType::VP
                             >
                           >;

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

    /// Generate edges using all vertices in the roadmap as the source.
    /// @param _r The roadmap to connect.
    /// @param _targetSet The set of target vertices, or null for the full
    ///                   roadmap.
    /// @param _collision An optional output iterator for collisions.
    template <typename AbstractRoadmapType>
    void Connect(AbstractRoadmapType* const _r,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<AbstractRoadmapType>* const _collision = nullptr);

    /// Generate edges using a single vertex as the source.
    /// @param _r The roadmap to connect.
    /// @param _source The source vertex.
    /// @param _targetSet The set of target vertices, or null for the full
    ///                   roadmap.
    /// @param _collision An optional output iterator for collisions.
    template <typename AbstractRoadmapType>
    void Connect(AbstractRoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<AbstractRoadmapType>* const _collision = nullptr);

    /// Generate edges using a range of VIDs as the source.
    /// @tparam InputIterator Either a roadmap iterator or any iterator that
    ///                       dereferences to a VID.
    /// @param _r The roadmap to connect.
    /// @param _sourceBegin The beginning of the VID range.
    /// @param _sourceEnd The end of the VID range.
    /// @param _targetSet The set of target vertices, or null for the full
    ///                   roadmap.
    /// @param _collision An optional output iterator for collisions.
    template <typename AbstractRoadmapType, typename InputIterator>
    void Connect(AbstractRoadmapType* const _r,
        InputIterator _sourceBegin, InputIterator _sourceEnd,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<AbstractRoadmapType>* const _collision = nullptr);

    /// Check whether this is a rewiring connector (which may delete edges).
    /// @return True if this is a rewiring connector.
    bool IsRewiring() const noexcept;

    ///@}

  protected:

    ///@name Connection Helpers
    ///@{

    /// Connect a source node to some subset of a target set. Derived classes
    /// specify how this will happen.
    /// @param _r The owning roadmap.
    /// @param _source The source node to connect.
    /// @param _targetSet The candidate target nodes, or null for the whole
    ///                   roadmap.
    /// @param _collision An optional output iterator for collisions.
    virtual void ConnectImpl(RoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<RoadmapType>* const _collision = nullptr);

    /// @overload This version is for group roadmaps.
    virtual void ConnectImpl(GroupRoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<GroupRoadmapType>* const _collision = nullptr);

    ///@example Connectors_UseCase.cpp
    /// This is an example of how to use the Connector Methods.

    /// Try to connect a given configuration to a set of nearest neighbors.
    /// @param _r The roadmap.
    /// @param _source The configuration.
    /// @param _neighbors The set of neighbors to try.
    /// @param _collision Output for invalid configurations.
    /// @param _earlyQuit Quit after the first successful connection?
    template <typename AbstractRoadmapType>
    void ConnectNeighbors(
        AbstractRoadmapType* const _r, const VID _source,
        const std::vector<Neighbor>& _neighbors,
        OutputIterator<AbstractRoadmapType>* const _collision = nullptr,
        const bool _earlyQuit = false);

    /// Attempt a connection between two individual configurations.
    /// @param _r The roadmap.
    /// @param _source The source configuration's VID.
    /// @param _target The target configuration's VID.
    /// @param _collision Output for invalid configurations.
    /// @return True if the connection succeeded.
    bool ConnectNodes(RoadmapType* const _r, const VID _source,
        const VID _target,
        OutputIterator<RoadmapType>* const _collision = nullptr);

    /// Attempt a connection between two group configurations.
    /// @param _r The group roadmap.
    /// @param _source The source configuration's VID.
    /// @param _target The target configuration's VID.
    /// @param _collision Output for invalid configurations.
    /// @return True if the connection succeeded.
    bool ConnectNodes(GroupRoadmapType* const _r, const VID _source,
        const VID _target,
        OutputIterator<GroupRoadmapType>* const _collision = nullptr);

    /// Check whether a connection should be attempted.
    /// @param _r The roadmap.
    /// @param _source The source vertex.
    /// @param _target The target vertex.
    /// @return True if the connection should not be attempted.
    template <typename AbstractRoadmapType>
    bool DoNotCheck(AbstractRoadmapType* const _r, const VID _source,
        const VID _target) const;

    ///@}
    ///@name Connection Caching
    ///@{

    /// Add a failed connection attempt to the cache.
    /// @param _map The map pointer (either individual or group roadmap).
    /// @param _source The source VID.
    /// @param _target The target VID.
    void CacheFailedConnection(void* const _map, const VID _source,
        const VID _target) noexcept;

    /// Check if attempt is in the failure cache.
    /// @param _map The map pointer (either individual or group roadmap).
    /// @param _source The source VID.
    /// @param _target The target VID.
    /// @return True if the attempt to connect _source to _target has already
    ///         failed.
    bool IsCached(void* const _map, const VID _source, const VID _target) const
        noexcept;

    /// Clear the attempts cache.
    void ClearConnectionAttempts();

    ///@}
    ///@name Internal State
    ///@{

    ConnectionAttemptsCache m_attemptsCache; ///< All failed connection attempts.
    std::string m_lpLabel;                   ///< Local Planner

    bool m_skipIfSameCC{true};  ///< Skip connections within the same CC?
    size_t m_maxFailures{0};    ///< Maximum allowed failures per use.
    size_t m_failures{0};       ///< Failures in this use.
    bool m_oneWay{false};       ///< Create one-way edges or two-way?
    bool m_rewiring{false};     ///< Does this connector delete edges?

		bool m_selfEdges{false}; 		///< Indicates if roadmap vertices should have self-edges.

    /// This is a performance optimization which makes a big impact. The
    /// neighbor set can be as large as the roadmap, so it is important to avoid
    /// re-allocating it on every call to ConnectImpl.
    std::vector<Neighbor> m_neighborBuffer;

    ///@}

};


template <typename AbstractRoadmapType>
void
ConnectorMethod::
Connect(AbstractRoadmapType* const _r, const VertexSet* const _targetSet,
    OutputIterator<AbstractRoadmapType>* const _collision) {
  Connect(_r, _r->begin(), _r->end(), _targetSet, _collision);
}


template <typename AbstractRoadmapType>
void
ConnectorMethod::
Connect(AbstractRoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<AbstractRoadmapType>* const _collision)
{
  const std::array<VID, 1> source{_source};
  Connect(_r, source.begin(), source.end(), _targetSet, _collision);
}


template <typename AbstractRoadmapType, typename InputIterator>
void
ConnectorMethod::
Connect(AbstractRoadmapType* const _r,
    InputIterator _sourceBegin, InputIterator _sourceEnd,
    const VertexSet* const _targetSet,
    OutputIterator<AbstractRoadmapType>* const _collision)
{
  const std::string id = this->GetNameAndLabel() + "::Connect";
  MethodTimer mt(this->GetStatClass(), id);
  if(this->m_debug)
    std::cout << id << std::endl;

  m_failures = 0;
  for(auto iter = _sourceBegin; iter != _sourceEnd; ++iter) {
    const VID sourceVID = _r->GetVID(iter);
    if(this->m_debug)
      std::cout << "\tAttempting connections from node "
                << sourceVID << " at " << _r->GetVertex(sourceVID).PrettyPrint()
                << std::endl;
    ConnectImpl(_r, sourceVID, _targetSet, _collision);
  }
}


template <typename AbstractRoadmapType>
void
ConnectorMethod::
ConnectNeighbors(AbstractRoadmapType* const _r, const VID _source,
    const std::vector<Neighbor>& _neighbors,
    OutputIterator<AbstractRoadmapType>* const _collision,
    const bool _earlyQuit) {
  // Try to connect _source to each neighbor.
  for(const auto& neighbor : _neighbors) {
    // Check if we have exceeded the failure limit.
    if(m_maxFailures and m_failures >= m_maxFailures) {
      if(this->m_debug)
        std::cout << "\t\tFailures (" << m_failures << ") exceed threshold "
                  << "of " << m_maxFailures << ", stopping."
                  << std::endl;
      return;
    }

    // Attempt the next connection.
    const VID target = neighbor.target;
    if(this->m_debug)
      std::cout << "\t\tAttempting connection from " << _source << " to "
                << target << " at distance " << neighbor.distance << "."
                << std::endl;

    // Check if this attempt should be skipped.
    if(DoNotCheck(_r, _source, target))
      continue;

    // Attempt connection with the local planner.
    const bool connected = ConnectNodes(_r, _source, target, _collision);
    m_failures += !connected;

    if(this->m_debug)
      std::cout << "\t\t\tConnection " << (connected ? "succeeded" : "failed")
                << "." << std::endl;

    // Quit early on success if requested.
    if(_earlyQuit && connected)
      return;
  }

	//Connect vertices to themselves
	//if(m_selfEdges) {
		//this->ConnectNodes(_r, _source, _source, _collision);
	//}
}


template <typename AbstractRoadmapType>
bool
ConnectorMethod::
DoNotCheck(AbstractRoadmapType* const _r, const VID _source, const VID _target)
    const {
  // Check that both VIDs are valid.
  if(_source == INVALID_VID or _target == INVALID_VID) {
    if(this->m_debug)
      std::cout << "\t\t\tSkipping connection with invalid node "
                << "(" << _source << ", " << _target << ")."
                << std::endl;
    return true;
  }

  // Check for self-connection.
  if(_source == _target and !m_selfEdges) {
    if(this->m_debug)
      std::cout << "\t\t\tSkipping self-connection "
                << "(" << _source << ", " << _target << ")."
                << std::endl;
    return true;
  }

  // Check for cached connection.
  if(this->IsCached(_r, _source, _target)) {
    if(this->m_debug)
      std::cout << "\t\t\tSkipping cached failed connection "
                << "(" << _source << ", " << _target << ")."
                << std::endl;
    return true;
  }

  // Check if the edge already exists.
  if(_r->IsEdge(_source, _target)) {
    if(this->m_debug)
      std::cout << "\t\t\tSkipping existing connection "
                << "(" << _source << ", " << _target << ")."
                << std::endl;
    return true;
  }

  // Check if the nodes are in the same connected component.
  if(m_skipIfSameCC) {
    auto ccTracker = _r->GetCCTracker();
    if(!ccTracker)
      throw RunTimeException(WHERE) << "A CCTracker is required for checking "
                                    << "same CCs.";
    const bool sameCC = ccTracker->InSameCC(_source, _target);
    if(sameCC) {
      if(this->m_debug)
        std::cout << "\t\t\tSkipping connection "
                  << "(" << _source << ", " << _target << ")"
                  << " within the same connected component."
                  << std::endl;
      return true;
    }
  }

  return false;
}

#endif
