#ifndef PMPL_TOPOLOGICAL_MAP_H_
#define PMPL_TOPOLOGICAL_MAP_H_

#include <map>
#include <vector>

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/SSSP.h"
#include "Utilities/XMLNode.h"
#include "Workspace/GridOverlay.h"
#include "Workspace/WorkspaceDecomposition.h"


////////////////////////////////////////////////////////////////////////////////
/// A topological map from roadmap vertices to decomposition cells. The map is
/// updated whenever a vertex is added or removed.
///
/// The documentation will frequently refer to VIDs or configurations that are
/// 'contained' within a workspace region or grid cell. In this context, a
/// configuration is contained by a region or cell if it's reference point (used
/// for distance measurement) lies within.
///
/// @WARNING The current implementation assumes that the workspace decomposition
///          will not change once created. If this occurs, the maps must be reset.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TopologicalMap final : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::WeightType             WeightType;
    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::GraphType           GraphType;
    typedef typename GraphType::VI                    VI;
    typedef WorkspaceDecomposition::vertex_descriptor VD;
    typedef WorkspaceDecomposition::adj_edge_iterator EI;

    ///@}
    ///@name Local Types
    ///@{

    typedef SSSPAdjacencyMap<WorkspaceDecomposition> AdjacencyMap;
    typedef SSSPOutput<WorkspaceDecomposition>       SSSPData;

    typedef std::vector<const WorkspaceRegion*>      NeighborhoodKey;

    ///@}
    ///@name Construction
    ///@{

    TopologicalMap();

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
    std::vector<VID> GetMappedVIDs(const WorkspaceRegion* const _region,
        const size_t _bodyIndex = 0) const;

    /// Get the set of VIDs that are bucketed within a set of regions.
    /// @param _regions The regions of interest.
    /// @param _bodyIndex The body to use.
    /// @return The set of VIDs that have body _bodyIndex mapped to _region.
    std::vector<VID> GetMappedVIDs(
        const std::vector<const WorkspaceRegion*>& _regions,
        const size_t _bodyIndex = 0) const;

    /// Get the workspace region to which a given VID is mapped.
    /// @param _vid The VID of interest.
    /// @param _bodyIndex The body to use.
    /// @return The workspace region in which body _bodyIndex resides when
    ///         configured at _vid.
    const WorkspaceRegion* GetMappedRegion(const VID _vid,
        const size_t _bodyIndex = 0) const;

    /// Check if a region is populated.
    /// @param _region The region to check.
    /// @param _bodyIndex The body to use.
    /// @return True if the region contains any configurations of _bodyIndex.
    bool IsPopulated(const WorkspaceRegion* const _region,
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
    const WorkspaceRegion* LocateRegion(const VID _vid,
        const size_t _bodyIndex = 0) const;

    /// Find the workspace region that contains the reference point for a given
    /// configuration and body.
    /// @param _c The configuration.
    /// @param _bodyIndex The body to use.
    /// @return The region containing _bodyIndex at _c, or null if in obstacle
    ///         space.
    const WorkspaceRegion* LocateRegion(const CfgType& _c,
        const size_t _bodyIndex = 0) const;

    /// Find the workspace region that contains a given point.
    /// @param _p The workspace point.
    /// @return The region containing _p, or null if _p is in obstacle space.
    const WorkspaceRegion* LocateRegion(const Point3d& _p) const;

    /// Try to find the nearest region for a configuration in obstacle space.
    /// @param _c The configuration in obstacle space.
    /// @param _bodyIndex The body to use.
    /// @return The nearest region to _p, or null if it cannot be found.
    const WorkspaceRegion* LocateNearestRegion(const CfgType& _c,
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
    size_t LocateCell(const VID _v, const size_t _bodyIndex = 0) const;

    /// Find the grid cell index that holds a given configuration.
    /// @param _c The configuration.
    /// @param _bodyIndex The body to use.
    /// @return The index of the grid cell which contains _bodyIndex when
    ///         configured at _c.
    size_t LocateCell(const CfgType& _c, const size_t _bodyIndex = 0) const;

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
    NeighborhoodKey LocateNeighborhood(const VID _v) const;
    NeighborhoodKey LocateNeighborhood(const CfgType& _c) const; ///< @overload

    ///@}
    ///@name Decomposition Access
    ///@{

    /// Get the decomposition used by this map.
    const WorkspaceDecomposition* GetDecomposition() const;

    ///@}
    ///@name Frontier
    ///@{

    /// Compute the frontier of occupied cells for a given body, starting from
    /// a designated cell.
    /// @param _region The starting region.
    /// @param _bodyIndex The body to use.
    /// @param _earlyStopDistance Stop after the last cell added is this far
    ///                           from the first populated cell. Use -1 to
    ///                           disable.
    /// @param _adjacency An optional adjacency map to use instead of the
    ///                   decomposition's.
    /// @return The SSSP results that describe the discovered frontier
    ///         (including populated and unpopulated cells).
    SSSPData
    ComputeFrontier(const WorkspaceRegion* const _region,
        const size_t _bodyIndex = 0,
        const double _earlyStopDistance = -1,
        const AdjacencyMap& _adjacency = {});

    ///@}

  private:

    ///@name Cfg Mapping
    ///@{

    /// Map a new configuration to the appropriate decomposition region and vis
    /// versa.
    /// @param _vertex A roadmap graph iterator to the newly added vertex.
    void MapCfg(const VI _vertex);

    /// Unmap a to-be-deleted configuration.
    /// @param _vertex A roadmap graph iterator to the to-be-deleted vertex.
    void UnmapCfg(const VI _vertex);

    /// Clear the maps.
    void ClearMaps();

    /// Get the forward map from region -> VIDs for a given body.
    /// @param _bodyIndex The body index.
    /// @return The map for this body.
    const std::map<const WorkspaceRegion*, std::vector<VID>>& GetForwardMap(
        const size_t _bodyIndex) const;

    /// Get the inverse map from VID -> regions for each body.
    /// @param _vid The VID to unmap.
    /// @return The neighborhood key for this configuration.
    const NeighborhoodKey& GetInverseMap(const VID _vid) const;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_decompositionLabel; ///< The workspace decomposition to use.

    /// A grid matrix overlaid on the workspace.
    std::unique_ptr<const GridOverlay> m_grid;

    double m_gridSize{.1}; ///< Length of each grid cell.

    /// A mapping from grid cells to workspace regions.
    GridOverlay::DecompositionMap m_cellToRegions;

    /// A mapping from workspace regions to contained VIDs.
    std::vector<std::map<const WorkspaceRegion*, std::vector<VID>>>
        m_regionToVIDs;

    /// An inverse mapping from VID to the containing neighborhood.
    std::unordered_map<VID, NeighborhoodKey> m_vidToNeighborhood;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
TopologicalMap<MPTraits>::
TopologicalMap() : MPBaseObject<MPTraits>("TopologicalMap") {
}


template <typename MPTraits>
TopologicalMap<MPTraits>::
TopologicalMap(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("TopologicalMap");

  m_gridSize = _node.Read("gridSize", true, 0.,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The grid cell length. Very small values will cause memory overflow "
      "as we try to map huge quantities of grid cell to workspace region "
      "relations. Over-large values will cause slow mappings as we will need "
      "to check many candidate regions.");

  m_decompositionLabel = _node.Read("decompositionLabel", true, "",
      "The workspace decomposition to use.");
}


template <typename MPTraits>
TopologicalMap<MPTraits>::
~TopologicalMap() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
TopologicalMap<MPTraits>::
Initialize() {
  // Initialize the maps.
  ClearMaps();

  auto envBound = this->GetEnvironment()->GetBoundary();
  auto decomposition = this->GetMPTools()->GetDecomposition(m_decompositionLabel);

  // Initialize the grid and decomposition map.
  m_grid = std::unique_ptr<const GridOverlay>(new GridOverlay(envBound, m_gridSize));
  {
    MethodTimer mt(this->GetStatClass(), "GridOverlay::ComputeDecompositionMap");
    m_cellToRegions = m_grid->ComputeDecompositionMap(decomposition);
  }

  // Set the size of the mapping.
  auto mb = this->GetTask()->GetRobot()->GetMultiBody();
  m_regionToVIDs.resize(mb->GetNumBodies());

  // Install roadmap hooks.
  auto g = this->GetRoadmap()->GetGraph();

  // Map cfgs to workspace regions whenever they are added.
  g->InstallHook(GraphType::HookType::AddVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->MapCfg(_vi);});
  // Undo the mapping when cfgs are deleted.
  g->InstallHook(GraphType::HookType::DeleteVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->UnmapCfg(_vi);});

  // If the graph has existing vertices, map them now.
  std::cout << "Graph has " << g->Size() << " vertices." << std::endl;
  for(auto iter = g->begin(); iter != g->end(); ++iter)
    MapCfg(iter);
}

/*---------------------------------- Queries ---------------------------------*/

template <typename MPTraits>
std::vector<typename TopologicalMap<MPTraits>::VID>
TopologicalMap<MPTraits>::
GetMappedVIDs(const WorkspaceRegion* const _region, const size_t _bodyIndex)
    const {
  /// @bug This version only returns a sorted output because the graph uses
  ///      ascending VIDs. Things will get confused if we add VIDs out of order.
  ///      Ideally we need to check for this in MapCfg, and maybe also make a
  ///      loaded roadmap compress its VIDs if there are any missing.
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::GetMappedVIDs");

  const auto& forwardMap = GetForwardMap(_bodyIndex);

  auto iter = forwardMap.find(_region);
  return iter == forwardMap.end() ? std::vector<VID>() : iter->second;
}


template <typename MPTraits>
std::vector<typename TopologicalMap<MPTraits>::VID>
TopologicalMap<MPTraits>::
GetMappedVIDs(const std::vector<const WorkspaceRegion*>& _regions,
    const size_t _bodyIndex) const {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::GetMappedVIDs");

  const auto& forwardMap = GetForwardMap(_bodyIndex);

  // Find all VIDs that live within the region set.
  /// @todo Profile effect of changing all to a std::set and removing the later
  ///       sort/unique calls.
  std::vector<VID> all;
  for(const auto& region : _regions) {
    // Skip empty regions.
    auto iter = forwardMap.find(region);
    if(iter == forwardMap.end())
      continue;

    // Copy these VIDs.
    const auto& vids = iter->second;
    std::copy(vids.begin(), vids.end(), std::back_inserter(all));
  }

  // Sort and make sure list is unique.
  std::sort(all.begin(), all.end());
  auto newEnd = std::unique(all.begin(), all.end());

  // Check that we didn't double-add any vertices.
  if(all.end() != newEnd)
    throw RunTimeException(WHERE) << "Unique removed vertices. Each vertex should "
                                  << "be mapped to only one region.";

  return all;
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
GetMappedRegion(const VID _vid, const size_t _bodyIndex) const {
  const auto& inverseMap = GetInverseMap(_vid);
  try {
    return inverseMap.at(_bodyIndex);
  }
  catch(const std::runtime_error& _e) {
    throw RunTimeException(WHERE) << "Inverse map for VID " << _vid
                                  << " has no entry for body index " << _bodyIndex
                                  << ".";
  }
}


template <typename MPTraits>
bool
TopologicalMap<MPTraits>::
IsPopulated(const WorkspaceRegion* const _region, const size_t _bodyIndex) const {
  const auto& forwardMap = GetForwardMap(_bodyIndex);
  auto iter = forwardMap.find(_region);
  return iter != forwardMap.end() and !iter->second.empty();
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
GetRandomRegion() const {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::GetRandomRegion");

  auto d = GetDecomposition();
  return &d->GetRegion(LRand() % d->GetNumRegions());
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
LocateRegion(const VID _vid, const size_t _bodyIndex) const {
  return LocateRegion(this->GetRoadmap()->GetGraph()->GetVertex(_vid),
      _bodyIndex);
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
LocateRegion(const CfgType& _cfg, const size_t _bodyIndex) const {
  _cfg.ConfigureRobot();
  auto body = _cfg.GetMultiBody()->GetBody(_bodyIndex);

  return LocateRegion(body->GetWorldTransformation().translation());
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
LocateRegion(const Point3d& _point) const {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::LocateRegion");

  // Find the grid cell that contains the new configuration's reference point.
  const size_t cell = m_grid->LocateCell(_point);

  // Find the correct region out of the candidates.
  const auto& candidateRegions = m_cellToRegions[cell];

  for(const WorkspaceRegion* region : candidateRegions)
    if(region->GetBoundary()->InBoundary(_point))
      return region;

  // If we get here, we didn't find a region.
  return nullptr;
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
LocateNearestRegion(const CfgType& _c, const size_t _bodyIndex) const {
  _c.ConfigureRobot();
  auto body = _c.GetMultiBody()->GetBody(_bodyIndex);

  return LocateNearestRegion(body->GetWorldTransformation().translation());
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
LocateNearestRegion(const Point3d& _p) const {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::LocateNearestRegion");

  // Put the point robot here.
  auto pointRobot = this->GetMPProblem()->GetRobot("point");
  CfgType temp(_p, pointRobot);

  // Use PQPSolid to get the CD info. If the check is valid, this was already in
  // free space.
  /// @TODO This relies on magic strings in the XML. Make it more robust.
  CDInfo cdInfo(true);
  auto vc = this->GetValidityChecker("pqp_solid");
  if(vc->IsValid(temp, cdInfo, "TopologicalMap::FindCandidateRegions"))
    return LocateRegion(_p);

  // Compute the vector from the sampled point to the outside of the obstacle.
  const Vector3d delta = cdInfo.m_objectPoint - cdInfo.m_robotPoint;

  // The nearest possible free point is just outside of the obstacle along
  // delta.
  auto env = this->GetEnvironment();
  return LocateRegion(cdInfo.m_objectPoint + delta.scale(env->GetPositionRes()));
}


template <typename MPTraits>
size_t
TopologicalMap<MPTraits>::
LocateCell(const VID _v, const size_t _bodyIndex) const {
  return LocateCell(this->GetRoadmap()->GetGraph()->GetVertex(_v), _bodyIndex);
}


template <typename MPTraits>
size_t
TopologicalMap<MPTraits>::
LocateCell(const CfgType& _cfg, const size_t _bodyIndex) const {
  _cfg.ConfigureRobot();
  auto body = _cfg.GetMultiBody()->GetBody(_bodyIndex);

  return LocateCell(body->GetWorldTransformation().translation());
}


template <typename MPTraits>
size_t
TopologicalMap<MPTraits>::
LocateCell(const Point3d& _p) const {
  return m_grid->LocateCell(_p);
}

/*---------------------------- Neighborhood Keys -----------------------------*/

template <typename MPTraits>
typename TopologicalMap<MPTraits>::NeighborhoodKey
TopologicalMap<MPTraits>::
LocateNeighborhood(const VID _v) const {
  return LocateNeighborhood(this->GetRoadmap()->GetGraph()->GetVertex(_v));
}


template <typename MPTraits>
typename TopologicalMap<MPTraits>::NeighborhoodKey
TopologicalMap<MPTraits>::
LocateNeighborhood(const CfgType& _c) const {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::LocateNeighborhood");

  // Create a key with storage for each body.
  auto mb = _c.GetMultiBody();
  NeighborhoodKey key(mb->GetNumBodies(), nullptr);

  // Locate the region occupied by each body's centroid.
  for(size_t i = 0; i < key.size(); ++i) {
    auto body = mb->GetBody(i);
    key[i] = LocateRegion(body->GetWorldTransformation().translation());
  }

  return key;
}

/*--------------------------- Decomposition Access ---------------------------*/

template <typename MPTraits>
const WorkspaceDecomposition*
TopologicalMap<MPTraits>::
GetDecomposition() const {
  return this->GetMPTools()->GetDecomposition(m_decompositionLabel);
}

/*-------------------------------- Cfg Mapping -------------------------------*/

template <typename MPTraits>
void
TopologicalMap<MPTraits>::
MapCfg(const VI _vertex) {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::MapCfg");

  const auto& cfg = _vertex->property();
  const VID vid = _vertex->descriptor();

  const auto neighborhood = LocateNeighborhood(cfg);

  // Forward-map the regions to the VID.
  for(size_t i = 0; i < neighborhood.size(); ++i) {
    auto region = neighborhood[i];
    m_regionToVIDs[i][region].push_back(vid);
  }

  // Inverse-map the VID to the regions.
  m_vidToNeighborhood[vid] = neighborhood;
}


template<typename MPTraits>
void
TopologicalMap<MPTraits>::
UnmapCfg(const VI _vertex) {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::UnmapCfg");

  const VID vid = _vertex->descriptor();

  // Assert that this vertex was mapped to a neighborhood.
  auto neighborhoodIter = m_vidToNeighborhood.find(vid);

  if(neighborhoodIter == m_vidToNeighborhood.end())
    throw RunTimeException(WHERE) << "Tried to unmap vertex " << vid
                                  << ", but didn't find a neighborhood for it.";

  // Remove the mapping from each neighborhood region to this VID.
  const auto& neighborhood = neighborhoodIter->second;
  for(size_t i = 0; i < neighborhood.size(); ++i) {
    const auto region = neighborhood[i];

    // Assert that the region was mapped to the vertex.
    auto& regionMap = m_regionToVIDs[i][region];
    auto vidIter = std::find(regionMap.begin(), regionMap.end(), vid);

    if(vidIter == regionMap.end())
      throw RunTimeException(WHERE) << "Tried to unmap vertex " << vid
                                    << ", but it was missing from it's region map.";

    // Remove this vertex from the map.
    regionMap.erase(vidIter);
  }

  m_vidToNeighborhood.erase(neighborhoodIter);
}


template <typename MPTraits>
void
TopologicalMap<MPTraits>::
ClearMaps() {
  m_regionToVIDs.clear();
  m_vidToNeighborhood.clear();
}


template <typename MPTraits>
const std::map<const WorkspaceRegion*,
               std::vector<typename MPTraits::RoadmapType::VID>>&
TopologicalMap<MPTraits>::
GetForwardMap(const size_t _bodyIndex) const {
  try {
    return m_regionToVIDs.at(_bodyIndex);
  }
  catch(const std::runtime_error& _e) {
    throw RunTimeException(WHERE) << "No forward map for body index "
                                  << _bodyIndex << ".";
  }
}


template <typename MPTraits>
const typename TopologicalMap<MPTraits>::NeighborhoodKey&
TopologicalMap<MPTraits>::
GetInverseMap(const VID _vid) const {
  try {
    return m_vidToNeighborhood.at(_vid);
  }
  catch(const std::runtime_error& _e) {
    throw RunTimeException(WHERE) << "No inverse map for VID " << _vid << ".";
  }
}

/*----------------------------------- SSSP -----------------------------------*/

template <typename MPTraits>
typename TopologicalMap<MPTraits>::SSSPData
TopologicalMap<MPTraits>::
ComputeFrontier(const WorkspaceRegion* const _region, const size_t _bodyIndex,
    const double _earlyStopDistance, const AdjacencyMap& _adjacency) {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::ComputeFrontier");

  // Const cast is required because STAPL has an API error preventing useful
  // iteration over non-const graphs.
  auto wd = const_cast<WorkspaceDecomposition*>(
      this->GetMPTools()->GetDecomposition(m_decompositionLabel));

  if(this->m_debug)
    std::cout << "TopologicalMap::ComputeFrontier"
              << "\n\tSearching from region " << wd->GetDescriptor(*_region)
              << "\n\tBody index: " << _bodyIndex
              << std::endl;

  // Create an early stop criterion if required.
  SSSPTerminationCriterion<WorkspaceDecomposition> stop;

  // Create storage for the maximum distance and whether we have found a
  // populated region.
  double maxDistance = std::numeric_limits<double>::max();
  bool foundFirstPopulated = false;

  const bool earlyStop = _earlyStopDistance != -1;
  if(earlyStop)
    stop = [this, wd, _bodyIndex, _earlyStopDistance, &maxDistance,
            &foundFirstPopulated](
        typename WorkspaceDecomposition::vertex_iterator& _vi,
        const SSSPOutput<WorkspaceDecomposition>& _sssp) {
      const WorkspaceDecomposition::vertex_descriptor vd = _vi->descriptor();
      const double distance = _sssp.distance.at(vd);

      // If we haven't found the first populated region yet, check for it now.
      if(!foundFirstPopulated) {
        auto& region = wd->GetRegion(vd);
        if(this->IsPopulated(&region, _bodyIndex)) {
          foundFirstPopulated = true;
          maxDistance = distance + _earlyStopDistance;

          if(this->m_debug)
            std::cout << "\t\tFirst populated node found, max distance is "
                      << std::setprecision(4) << maxDistance << "."
                      << std::endl;
        }
      }

      // Check for exceeding the max distance (not an else-condition incase the
      // max distance is 0).
      if(foundFirstPopulated and distance >= maxDistance) {
        if(this->m_debug)
          std::cout << "\t\tLast populated node found at distance "
                    << std::setprecision(4) << distance
                    << std::endl;
        return SSSPTermination::EndSearch;
      }

      return SSSPTermination::Continue;
    };
  else
    stop = SSSPDefaultTermination<WorkspaceDecomposition>();

  // Get the descriptor of the root node.
  const VD root = wd->GetDescriptor(*_region);

  return DijkstraSSSP(wd, {root}, stop, _adjacency);
}

/*----------------------------------------------------------------------------*/

#endif
