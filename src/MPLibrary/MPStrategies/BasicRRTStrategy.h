#ifndef PMPL_BASIC_RRT_STRATEGY_H_
#define PMPL_BASIC_RRT_STRATEGY_H_

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/NeighborhoodFinders/Neighbors.h"
#include "MPStrategyMethod.h"
#include "Utilities/XMLNode.h"

// #include <iomanip>
// #include <iterator>
// #include <string>
// #include <unordered_set>
// #include <vector>


////////////////////////////////////////////////////////////////////////////////
/// The RRT algorithm grows one or more trees from a set of root nodes to solve
/// a single-query planning problem.
///
/// References:
///   Original RRT:
///   LaValle, Steven M. "Rapidly-Exploring Random Trees: A New Tool for Path
///   Planning." TR 98-11, Computer Science Dept., Iowa State Univ., 1998.
///   RRT Connect (bi-directional):
///   James Kuffner and Steven LaValle. "RRT-Connect: An Efficient Approach to
///   Single-Query Path Planning". ICRA 2000.
///   Nonholonomic RRT:
///   Steven LaValle and James Kuffner. "Randomized Kinodynamic Planning." IJRR
///   2001.
///
/// This method supports both uni-directional and bi-directional variants.
/// Nonholonomic problems are supported with the appropriate extender and
/// uni-directional growth.
///
/// For uni-directional methods, we support an additional 'goal extension'
/// heuristic which attempts to connect configurations near the goal to the goal
/// region. This is necessary to get RRT to terminate - the alternative is to
/// rely on goal-biased sampling to eventually complete the problem, which is
/// not an efficient solution.
///
/// For bi-directional methods, the algorithm will attempt to connect trees
/// after each successful extension. The new node will be extended toward the
/// nearest neighbor in each other tree. If the extension reaches its
/// destination, the two trees will merge.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class BasicRRTStrategy : public MPStrategyMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef size_t VID;
    typedef typename std::unordered_set<VID> VertexSet;

    ///@}
    ///@name Construction
    ///@{

    BasicRRTStrategy();

    BasicRRTStrategy(XMLNode& _node);

    virtual ~BasicRRTStrategy();

    ///@}
    ///@name MPBaseObject overrides
    ///@{

    virtual void Print(std::ostream& _os) const;

    ///@}

  protected:

    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;

    ///@}
    ///@name Direction Helpers
    ///@{

    /// Get a random configuration to grow towards.
    virtual Cfg SelectTarget();

    /// Sample a target configuration to grow towards from an existing
    /// configuration. m_disperseTrials samples are attempted.
    /// @param _v The VID of the existing configuration.
    /// @return The sample who's growth direction yields the greatest separation
    ///         from the existing configuration's neighbors.
    /// @todo This functionality can probably be moved into a dispersed
    ///       extender, which we could call several times here.
    Cfg SelectDispersedTarget(const VID _v);

    ///@}
    ///@name Neighbor Helpers
    ///@{

    /// Find the nearest roadmap configuration to an arbitrary configuration.
    /// @param _cfg The query configuration.
    /// @param _candidates The candidate set to search, or null for whole
    ///                    roadmap.
    /// @return The VID of the roadmap configuration nearest to _cfg.
    virtual VID FindNearestNeighbor(const Cfg& _cfg,
        const VertexSet* const _candidates = nullptr);

    /// Select the best neighbor from the set of candidates returned by the NF.
    /// Default implementation selects the nearest.
    /// @param _cfg The query configuration.
    /// @param _neighbors The set of neighbors returned by the NF.
    /// @return The best of _neighbors to extend from for this method.
    virtual Neighbor SelectNeighbor(const Cfg& _cfg,
        const std::vector<Neighbor>& _neighbors);

    ///@}
    ///@name Growth Helpers
    ///@{

    /// Extend a new configuration from a nearby configuration towards a growth
    /// target.
    /// @param _nearVID The nearby configuration's VID.
    /// @param _target  The growth target.
    /// @param _lp An LPOutput for returning local planner info.
    /// @param _requireNew Require the extension to generate a new roadmap node
    ///                    (true unless connecting trees).
    /// @return The new node's VID.
    virtual VID Extend(const VID _nearVID, const Cfg& _target,
        LPOutput& _lp, const bool _requireNew = true);

    /// @overload
    VID Extend(const VID _nearVID, const Cfg& _target,
        const bool _requireNew = true);

    /// Add a new configuration to the roadmap and current tree.
    /// @param _newCfg The new configuration to add.
    /// @return A pair with the added VID and a bool indicating whether the new
    ///         node was already in the map.
    virtual std::pair<VID, bool> AddNode(const Cfg& _newCfg);

    /// Add a new edge to the roadmap.
    /// @param _source The source node.
    /// @param _target The target node.
    /// @param _lpOutput The extender output.
    virtual void AddEdge(const VID _source, const VID _target,
        const LPOutput& _lpOutput);

    /// Try to connect a configuration to its neighbors.
    /// @param _newVID The VID of the configuration to connect.
    void ConnectNeighbors(const VID _newVID);

    /// Try to extend a new configuration toward each goal region that is within
    /// the extender's range.
    /// @param _newVID The VID of a newly extended configuration.
    /// @note This only applies when not growing goals.
    void TryGoalExtension(const VID _newVID);

    /// Try to extend a new configuration toward a specific goal region. No-op
    /// if the goal is outside the extender's range.
    /// @param _newVID The VID of a newly extended configuration.
    /// @param _boundary The goal boundary.
    /// @note This only applies when not growing goals.
    void TryGoalExtension(const VID _newVID, const Boundary* const _boundary);

    ///@}
    ///@name Tree Helpers
    ///@{

    /// Attempt to expand the map by growing towards a target configuration from
    /// the nearest existing node.
    /// @param _target The target configuration.
    /// @return The VID of a newly created Cfg if successful, INVALID_VID
    ///         otherwise.
    VID ExpandTree(const Cfg& _target);

    /// Attempt to expand the map by growing towards a target configuration from
    /// an arbitrary existing node.
    /// @param _nearestVID The VID to grow from.
    /// @param _target The target configuration.
    /// @return The VID of a newly created Cfg if successful, INVALID_VID
    ///         otherwise.
    virtual VID ExpandTree(const VID _nearestVID, const Cfg& _target);

    /// If multiple trees exist, try to connect the current tree with the
    /// one that is nearest to a recently grown configuration.
    /// @param _recentlyGrown The VID of the recently grown configuration.
    void ConnectTrees(const VID _recentlyGrown);

    ///@}
    ///@name MP Object Labels
    ///@{

    std::string m_samplerLabel;  ///< The sampler label.
    std::string m_nfLabel;       ///< The neighborhood finder label.
    std::string m_ncLabel;       ///< The connector label (for RRG).
    std::string m_exLabel;       ///< The extender label.
    std::string m_goalDmLabel;   ///< Dm for checking goal extensions.

    std::string m_fallbackNfLabel; ///< NF for searching the active set, used if the main one fails.

    ///@}
    ///@name RRT Properties
    ///@{

    bool m_growGoals{false};      ///< Grow trees from goals.
    double m_growthFocus{0};      ///< The fraction of goal-biased expansions.
    double m_goalThreshold{0};    ///< Distance threshold for goal extension.
    size_t m_numDirections{1};    ///< Expansion directions per iteration.
    size_t m_disperseTrials{3};   ///< Sample attempts for disperse search.

    ///@}
    ///@name Internal State
    ///@{

    std::vector<VertexSet> m_trees;  ///< The current tree set.

    ///@}

};

#endif
