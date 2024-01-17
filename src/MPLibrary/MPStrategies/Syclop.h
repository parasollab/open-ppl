#ifndef PMPL_SYCLOP_H_
#define PMPL_SYCLOP_H_

#include "BasicRRTStrategy.h"
#include "MPLibrary/MPTools/TetGenDecomposition.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Workspace/WorkspaceRegion.h"

////////////////////////////////////////////////////////////////////////////////
/// This method is the 'Synergistic Combination of Layers of Planning' technique
/// that adds workspace guidance to RRT methods.
///
/// Reference:
///   Erion Plaku, Lydia E. Kavraki, Moshe Y. Vardi. "Synergistic Combination of
///   Layers of Planning". IEEE Transactions on Robotics. 2010.
///
/// @warning This method doesn't support bi-directional growth.
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class Syclop : public BasicRRTStrategy {
 public:
  ///@name Motion Planning Types
  ///@{

  typedef typename MPBaseObject::WeightType WeightType;
  typedef typename MPBaseObject::RoadmapType RoadmapType;
  typedef typename RoadmapType::VID VID;

  ///@}
  ///@name Local Types
  ///@{

  using typename BasicRRTStrategy::VertexSet;

  ///@}
  ///@name Construction
  ///@{

  Syclop();

  Syclop(XMLNode& _node);

  virtual ~Syclop() = default;

  ///@}

 protected:
  ///@name MPStrategy Overrides
  ///@{

  virtual void Initialize() override;

  ///@}
  ///@name BasicRRTStrategy Overrides
  ///@{

  /// As basic RRT, but picks nearest neighbor from only the available
  /// regions.
  virtual VID FindNearestNeighbor(
      const Cfg& _cfg,
      const VertexSet* const _candidates = nullptr) override;

  /// As basic RRT, but also logs extension attempts.
  virtual VID Extend(const VID _nearVID,
                     const Cfg& _qRand,
                     LPOutput& _lp,
                     const bool _requireNew = true) override;

  /// As basic RRT, but also updates coverage information.
  virtual std::pair<VID, bool> AddNode(const Cfg& _newCfg) override;

  /// As basic RRT, but also updates region edge connectivity information.
  virtual void AddEdge(const VID _source,
                       const VID _target,
                       const LPOutput& _lpOutput) override;

  ///@}
  ///@name Syclop Functions
  ///@{

  /// Compute a high-level plan (a sequence of regions).
  std::vector<VID> DiscreteLead();

  /// Compute a set of potential regions from the discrete lead.
  void FindAvailableRegions(std::vector<VID> _lead);

  /// Select a region from a set of available regions.
  const WorkspaceRegion* SelectRegion();

  ///@}
  ///@name Syclop Helpers
  ///@}

  /// Count the number of times that _r1 and _r2 have been selected as part of
  /// a discrete lead.
  size_t Sel(const WorkspaceRegion* const _r1,
             const WorkspaceRegion* const _r2);

  /// Estimate the progress made in connecting _r1 to _r2.
  size_t Conn(const WorkspaceRegion* const _r1,
              const WorkspaceRegion* const _r2);

  /// Compute the edge weight in the region graph from _r1 to _r2.
  double Cost(const WorkspaceRegion* const _r1,
              const WorkspaceRegion* const _r2);
  double Cost(const WorkspacePortal& _p);  ///< @overload

  ///@}
  ///@name Pre-processing Stuff
  ///@{

  void ComputeFreeVolumes();  ///< Estimate the free volume of each region.

  ///@}
  ///@name Auxiliary Classes
  ///@{

  ////////////////////////////////////////////////////////////////////////////
  /// Holds all external data related to a specific workspace region.
  ////////////////////////////////////////////////////////////////////////////
  struct RegionData {
    ///@name Internal State
    ///@{

    /// Relative probability of selecting this region from a lead.
    double weight;

    /// The edge-weight coefficient for this region.
    double alpha;

    /// The estimated free state-space volume of this region.
    double freeVolume;

    /// The number of times this region has been selected.
    size_t numTimesSelected{0};

    VertexSet vertices;  ///< The VID's of the configurations in this region.

    std::set<size_t> cells;  ///< The set of coverage cells in the target that
                             ///< were reached from the source.

    size_t Coverage() const { return cells.size(); }

    ///@}
    ///@name Update Functions
    ///@{

    /// Add a cell to this region's cell indexes and update alpha/weight.
    void AddCell(size_t _index) {
      cells.insert(_index);
      UpdateAlpha();
      UpdateWeight();
    }

    void UpdateAlpha() {
      alpha = 1. / ((1. + Coverage()) * std::pow(freeVolume, 4.));
    }

    void UpdateWeight() {
      weight = std::pow(freeVolume, 4.) /
               ((1. + Coverage()) * (1. + std::pow(numTimesSelected, 2.)));
    }

    ///@}
  };

  //////////////////////////////////////////////////////////////////////////////
  /// Tracks data related to edges between regions in the decomposition graph.
  //////////////////////////////////////////////////////////////////////////////
  struct RegionPairData {
    /// Add the index of a cell containing a vertex in the target that was
    /// reached by extending from the source.
    void AddCell(size_t _index) { cells.insert(_index); }

    /// Get the coverage for this edge.
    size_t Coverage() const { return cells.size(); }

    ///@name Internal State
    ///@{

    size_t numLeadUses{0};  ///< Times this edge was used in a lead.
    size_t numAttempts{0};  ///< Attempted extensions across this edge.

   private:
    /// The set of coverage cells in the target that were reached from the
    /// source.
    std::set<size_t> cells;

    ///@}
  };

  //////////////////////////////////////////////////////////////////////////////
  /// A functor for getting associating region graph edges with the cost
  /// function.
  //////////////////////////////////////////////////////////////////////////////
  struct WeightFunctor {
    ///@name Local Types
    ///@{

    typedef WorkspacePortal property_type;
    typedef double value_type;
    typedef Syclop strategy_type;

    ///@}
    ///@name Internal State
    ///@{

    strategy_type* m_syclop;

    ///@}
    ///@name Construction
    ///@{

    WeightFunctor(strategy_type* _s) : m_syclop(_s) {}

    ///@}
    ///@name Edge Map Functor Interface
    ///@{

    value_type get(property_type& _p) { return m_syclop->Cost(_p); }

    void put(property_type& _p, value_type& _v) {}

    template <typename functor>
    void apply(property_type& _p, functor _f) {
      _f(_p);
    }

    ///@}
  };

  //////////////////////////////////////////////////////////////////////////////
  /// A visitor to track the parent-child relationships discovered during DFS.
  //////////////////////////////////////////////////////////////////////////////
  struct DFSVisitor : public stapl::visitor_base<WorkspaceDecomposition> {
    ///@name Local Types
    ///@{

    typedef WorkspaceDecomposition graph_type;
    typedef stapl::visitor_return visitor_return;
    typedef std::map<VID, VID> map_type;

    ///@}
    ///@name Internal State
    ///@{

    map_type& m_parentMap;

    ///@}
    ///@name Construction
    ///@{

    DFSVisitor(map_type& _pm) : m_parentMap(_pm) {}

    ///@}
    ///@name Visitor Interface
    ///@{

    virtual visitor_return tree_edge(
        graph_type::vertex_iterator _vit,
        graph_type::adj_edge_iterator _eit) override {
      m_parentMap[_eit->target()] = _eit->source();
      return visitor_return::CONTINUE;
    }

    ///@}
  };

  ///@}
  ///@name Syclop State
  ///@{

  /// Holds extra data associated with the regions.
  std::map<const WorkspaceRegion*, RegionData> m_regionData;

  /// Holds extra data associated with region pairs.
  std::map<std::pair<const WorkspaceRegion*, const WorkspaceRegion*>,
           RegionPairData>
      m_regionPairData;

  std::string m_tmLabel;  ///< The topological map label.

  /// The currently available regions.
  std::set<const WorkspaceRegion*> m_availableRegions;

  ///@}
  ///@name Switch Tracking
  ///@{
  /// Data for knowing when to change regions/leads.

  size_t m_currentLeadUses{0};
  size_t m_maxLeadUses{6};

  size_t m_currentRegionUses{0};
  size_t m_maxRegionUses{6};

  std::string m_freeVolumeSampler{"UniformRandom"};
  std::string m_freeVolumeVcLabel{"pqp_solid"};

  const WorkspaceRegion* m_currentRegion{nullptr};

  bool m_improvement{false};  ///< Have we improved the map w/ current region?

  ///@}
};

#endif
