#ifndef PMPL_TOPOLOGICAL_MAP_H_
#define PMPL_TOPOLOGICAL_MAP_H_

#include "MPLibrary/MPBaseObject.h"
#include "Simulator/Conversions.h"
#include "Utilities/Hash.h"
#include "Utilities/SSSP.h"
#include "Utilities/XMLNode.h"
#include "Workspace/GridOverlay.h"
#include "Workspace/WorkspaceDecomposition.h"

#include "glutils/obj_file.h"
#include "glutils/triangulated_model.h"

////////////////////////////////////////////////////////////////////////////////
/// A topological map from roadmap vertices to decomposition cells. The map is
/// updated whenever a vertex is added or removed.
///
/// The documentation will frequently refer to VIDs or configurations that are
/// 'contained' within a workspace region or grid cell. In this context, a
/// configuration is contained by a region or cell if it's reference point (used
/// for distance measurement) lies within.
///
/// Reference:
///   Read Sandstrom, Andrew Bregger, Ben Smith, Shawna Thomas, and Nancy M.
///   Amato. "Topological Nearest-Neighbor Filtering for Sampling-based
///   Planners". ICRA 2018.
///   - and -
///   Read Sandstrom, Jory Denny, and Nancy M. Amato. "Asymptotically-Optimal
//    Topological Nearest-Neighbor Filtering". Under review for RA-L @ IROS 20.
//    - and -
//    Read Sandstrom. "Approximating Configuration Space Topology with Workspace
//    Models". PhD Thesis, May 2020.
///
/// @WARNING The current implementation assumes that the workspace decomposition
///          will not change once created. If this occurs, the maps must be
///          reset.
////////////////////////////////////////////////////////////////////////////////
class TopologicalMap final : public MPBaseObject {
 public:
  ///@}
  ///@name Local Types
  ///@{

  typedef typename MPBaseObject::WeightType WeightType;
  typedef typename MPBaseObject::RoadmapType RoadmapType;
  typedef typename RoadmapType::VID VID;
  typedef typename RoadmapType::VI VI;
  typedef typename RoadmapType::VertexSet VertexSet;

  typedef std::vector<const WorkspaceRegion*> NeighborhoodKey;

  /// A map describing the distance to a region from some starting point.
  typedef std::unordered_map<const WorkspaceRegion*, double> DistanceMap;

  /// A map of the VIDs occupying a given region.
  typedef std::map<const WorkspaceRegion*, VertexSet> OccupancyMap;

  ///@}
  ///@name Construction
  ///@{

  TopologicalMap();
  TopologicalMap(double gridSize, string decomp_label) {
    this->m_gridSize = gridSize;
    this->m_decompositionLabel = decomp_label;
  }
  TopologicalMap(XMLNode& _node);

  virtual ~TopologicalMap();

  ///@}
  ///@name MPBaseObject Overrides
  ///@{

  virtual void Initialize() override;

  ///@}
  ///@name Map Queries
  ///@{

  /// Get the set of VIDs that are bucketed within a given region.
  /// @param _region The region of interest.
  /// @param _bodyIndex The body to use.
  /// @return The set of VIDs that have body _bodyIndex mapped to _region.
  const VertexSet* GetMappedVIDs(RoadmapType* const _r,
                                 const WorkspaceRegion* const _region,
                                 const size_t _bodyIndex = 0) const;

  /// Get the workspace region to which a given VID is mapped.
  /// @param _vid The VID of interest.
  /// @param _bodyIndex The body to use.
  /// @return The workspace region in which body _bodyIndex resides when
  ///         configured at _vid.
  const WorkspaceRegion* GetMappedRegion(RoadmapType* const _r,
                                         const VID _vid,
                                         const size_t _bodyIndex = 0) const;

  /// Check if a region is populated.
  /// @param _region The region to check.
  /// @param _bodyIndex The body to use.
  /// @return True if the region contains any configurations of _bodyIndex.
  bool IsPopulated(RoadmapType* const _r,
                   const WorkspaceRegion* const _region,
                   const size_t _bodyIndex = 0) const;

  ///@}
  ///@name Region and Cell Location
  ///@{

  /// Let it be a light for you in dark places, when all other lights go out.
  const WorkspaceRegion* GetRandomRegion() const;

  /// Find the workspace region that contains the reference point for a given
  /// configuration and body.
  /// @param _vid The configuration's VID.
  /// @param _bodyIndex The body to use.
  /// @return The region containing _bodyIndex at _vid, or null if in obstacle
  ///         space.
  const WorkspaceRegion* LocateRegion(RoadmapType* const _r,
                                      const VID _vid,
                                      const size_t _bodyIndex = 0) const;

  /// Find the workspace region that contains the reference point for a given
  /// configuration and body.
  /// @param _c The configuration.
  /// @param _bodyIndex The body to use.
  /// @return The region containing _bodyIndex at _c, or null if in obstacle
  ///         space.
  const WorkspaceRegion* LocateRegion(const Cfg& _c,
                                      const size_t _bodyIndex = 0) const;

  /// Find the workspace region that contains a given point.
  /// @param _p The workspace point.
  /// @return The region containing _p, or null if _p is in obstacle space.
  const WorkspaceRegion* LocateRegion(const Point3d& _p) const;

  /// Try to find the nearest region for a configuration in obstacle space.
  /// @param _c The configuration in obstacle space.
  /// @param _bodyIndex The body to use.
  /// @return The nearest region to _p, or null if it cannot be found.
  const WorkspaceRegion* LocateNearestRegion(const Cfg& _c,
                                             const size_t _bodyIndex = 0) const;

  /// Try to find the nearest region for a point in obstacle space.
  /// @param _p The point in obstacle space.
  /// @return The nearest region to _p, or null if it cannot be found.
  const WorkspaceRegion* LocateNearestRegion(const Point3d& _p) const;

  /// Find the grid cell index that holds a given configuration.
  /// @param _v The configuration VID.
  /// @param _bodyIndex The body to use.
  /// @return The index of the grid cell which contains _bodyIndex when
  ///         configured at _v.
  size_t LocateCell(RoadmapType* const _r,
                    const VID _v,
                    const size_t _bodyIndex = 0) const;

  /// Find the grid cell index that holds a given configuration.
  /// @param _c The configuration.
  /// @param _bodyIndex The body to use.
  /// @return The index of the grid cell which contains _bodyIndex when
  ///         configured at _c.
  size_t LocateCell(const Cfg& _c, const size_t _bodyIndex = 0) const;

  /// Find the grid cell index that holds a workspace point.
  /// @param _p The workspace point.
  /// @return The index of the grid cell which contains _p.
  size_t LocateCell(const Point3d& _p) const;

  ///@}
  ///@name Neighborhood Keys
  ///@{
  /// A neighborhood key describes the workspace regions that are occupied by
  /// each body of the robot. This is the result of LocateRegion for each
  /// body.

  /// Locate the neighborhood that is occupied by a robot in a particular
  /// configuration.
  NeighborhoodKey LocateNeighborhood(RoadmapType* const _r, const VID _v) const;
  NeighborhoodKey LocateNeighborhood(const Cfg& _c) const;  ///< @overload

  ///@}
  ///@name Decomposition Access
  ///@{

  /// Get the decomposition used by this map.
  const WorkspaceDecomposition* GetDecomposition() const;

  ///@}
  ///@name Inter-Region Distance
  ///@{

  /// Approximate the minimum inner distance between two regions. This
  /// estimates the shortest-path distance through free space.
  /// @param _source The source region.
  /// @param _target The target region.
  /// @return The approximate minimum distance from _source to _target
  ///         measured through free workspace. If it hasn't already been
  ///         computed, infinity will be returned. Passing null as one of the
  ///         regions will return the max computed frontier distance (0 for
  ///         none).
  double ApproximateMinimumInnerDistance(const WorkspaceRegion* const _source,
                                         const WorkspaceRegion* const _target);

  /// Compute approximate minimum inner cell distances from a source region
  /// out to a given radius.
  /// @param _source The source cell.
  /// @param _radius Compute inner distances for cells within this radius.
  const DistanceMap& ComputeApproximateMinimumInnerDistances(
      const WorkspaceRegion* const _source,
      const double _radius);

  ///@}

 private:
  ///@name Cfg Mapping
  ///@{

  /// Create a new mapping for a robot.
  void EnsureMap(RoadmapType* const _r);

  /// Map a new configuration to the appropriate decomposition region and vis
  /// versa.
  /// @param _vertex A roadmap graph iterator to the newly added vertex.
  void MapCfg(RoadmapType* const _r, const VI _vertex);

  /// Unmap a to-be-deleted configuration.
  /// @param _vertex A roadmap graph iterator to the to-be-deleted vertex.
  void UnmapCfg(RoadmapType* const _r, const VI _vertex);

  /// Clear the cfg maps.
  void ClearCfgMaps();

  /// Get the forward map from region -> VIDs for a given body.
  /// @param _bodyIndex The body index.
  /// @return The map for this body.
  const OccupancyMap& GetForwardMap(RoadmapType* const _r,
                                    const size_t _bodyIndex) const;

  /// Get the inverse map from VID -> regions for each body.
  /// @param _vid The VID to unmap.
  /// @return The neighborhood key for this configuration.
  const NeighborhoodKey& GetInverseMap(RoadmapType* const _r,
                                       const VID _vid) const;

  ///@}
  ///@name Internal State
  ///@{

  std::string m_decompositionLabel;  ///< The workspace decomposition to use.

  std::string m_pqpLabel;  ///< PQP CD for finding nearest regions.

  /// A grid matrix overlaid on the workspace.
  std::unique_ptr<const GridOverlay> m_grid;

  double m_gridSize{.1};  ///< Length of each grid cell.

  /// A mapping from grid cells to workspace regions.
  GridOverlay::DecompositionMap m_cellToRegions;

  /// A mapping from workspace regions to contained VIDs.
  std::unordered_map<RoadmapType*, std::vector<OccupancyMap>> m_regionToVIDs;

  /// An inverse mapping from VID to the containing neighborhood.
  std::unordered_map<RoadmapType*, std::unordered_map<VID, NeighborhoodKey>>
      m_vidToNeighborhood;

  /// A set of grid cells which lie on the boundary of obstacle space.
  std::unordered_set<size_t> m_boundaryCells;

  /// A cache for approximate inner-distance between regions.
  std::unordered_map<const WorkspaceRegion*, DistanceMap> m_innerDistanceMap;

  ///@}
};

#endif
