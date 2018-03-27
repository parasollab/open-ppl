#ifndef TOPOLOGICAL_MAP_H_
#define TOPOLOGICAL_MAP_H_

#include <map>
#include <vector>

#include "MPLibrary/MPBaseObject.h"
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

    /// A descriptor-based adjacency map for the successors from an SSSP run.
    typedef std::unordered_map<VD, std::vector<VD>> AdjacencyMap;

    ////////////////////////////////////////////////////////////////////////////
    /// The output of an SSSP run.
    ////////////////////////////////////////////////////////////////////////////
    struct SSSPOutput {

      std::unordered_map<VD, double> distances; ///< Distance to each cell from start.
      std::vector<VD> ordering;                 ///< Cell discovery ordering.
      AdjacencyMap successors;                  ///< Maps predecessor -> successors.

    };

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
    /// @return The set of VIDs that are mapped to _region.
    std::vector<VID> GetMappedVIDs(const WorkspaceRegion* const _region) const;

    /// Get the set of VIDs that are bucketed within a set of regions.
    /// @param _regions The regions of interest.
    /// @return The set of VIDs that are mapped to _regions.
    std::vector<VID> GetMappedVIDs(
        const std::vector<const WorkspaceRegion*>& _regions) const;

    /// Get the workspace region to which a given VID is mapped.
    /// @param _vid The VID of interest.
    /// @return The workspace region in which _vid resides.
    const WorkspaceRegion* GetMappedRegion(const VID _vid) const;

    /// Check if a region is populated.
    /// @param _region The region to check.
    /// @return True if the region is populated.
    bool IsPopulated(const WorkspaceRegion* const _region) const;

    ///@}
    ///@name Region and Cell Location
    ///@{

    /// Let it be a light for you in dark places, when all other lights go out.
    const WorkspaceRegion* GetRandomRegion() const;

    /// Find the workspace region that contains a configuration's reference
    /// point. Will return null for configurations in obstacle space.
    const WorkspaceRegion* LocateRegion(const VID _vid) const;
    const WorkspaceRegion* LocateRegion(const CfgType& _c) const; ///< @overload
    const WorkspaceRegion* LocateRegion(const Point3d& _p) const; ///< @overload

    /// Try to find the nearest region for a point in obstacle space. Will
    /// return null if it cannot be found.
    const WorkspaceRegion* LocateNearestRegion(const CfgType& _c) const;
    const WorkspaceRegion* LocateNearestRegion(const Point3d& _p) const;

    /// Find the grid cell index that holds a given configuration.
    size_t LocateCell(const VID _v) const;
    size_t LocateCell(const CfgType& _c) const; ///< @overload
    size_t LocateCell(const Point3d& _p) const; ///< @overload

    ///@}
    ///@name Decomposition Access
    ///@{

    /// Get the decomposition used by this map.
    const WorkspaceDecomposition* GetDecomposition() const;

    ///@}
    ///@name SSSP
    ///@{

    /// Compute the SSSP through the workspace decomposition graph starting from
    /// a designated cell.
    /// @param _region The starting region.
    /// @param _earlyStopDistance Stop after the last cell added is this far
    ///                           from the first populated cell. Use -1 to
    ///                           disable.
    /// @param _adjacency An optional adjacency map to use instead of the
    ///                   decomposition's.
    /// @return The cell distances and successor tree.
    SSSPOutput
    ComputeSSSP(const WorkspaceRegion* const _region,
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
    void ClearMap();

    ///@}
    ///@name Internal State
    ///@{

    std::string m_decompositionLabel; ///< The workspace decomposition to use.

    /// A grid matrix overlaid on the workspace.
    const GridOverlay* m_grid{nullptr};

    double m_gridSize{.1}; ///< Length of each grid cell.

    /// A mapping from grid cells to workspace regions.
    GridOverlay::DecompositionMap m_cellToRegions;

    /// A mapping from workspace regions to contained VIDs.
    std::map<const WorkspaceRegion*, std::vector<VID>> m_regionToVIDs;

    /// An inverse mapping from VID to the containing region.
    std::unordered_map<VID, const WorkspaceRegion*> m_vidToRegion;

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
~TopologicalMap() {
  delete m_grid;
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
TopologicalMap<MPTraits>::
Initialize() {
  ClearMap();

  auto env = this->GetEnvironment();
  auto stats = this->GetStatClass();
  auto decomposition = this->GetMPTools()->GetDecomposition(m_decompositionLabel);

  // Initialize the grid and decomposition map.
  delete m_grid;
  m_grid = new GridOverlay(env->GetBoundary(), m_gridSize);
  {
    MethodTimer mt(stats, "GridOverlay::ComputeDecompositionMap");
    m_cellToRegions = m_grid->ComputeDecompositionMap(decomposition);
  }

  // Install roadmap hooks if needed.
  auto g = this->GetRoadmap()->GetGraph();

  // Map cfgs to workspace regions whenever they are added.
  g->InstallHook(GraphType::HookType::AddVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->MapCfg(_vi);});
  // Undo the mapping when cfgs are deleted.
  g->InstallHook(GraphType::HookType::DeleteVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->UnmapCfg(_vi);});
}

/*---------------------------------- Queries ---------------------------------*/

template <typename MPTraits>
std::vector<typename TopologicalMap<MPTraits>::VID>
TopologicalMap<MPTraits>::
GetMappedVIDs(const WorkspaceRegion* const _region) const {
  auto iter = m_regionToVIDs.find(_region);
  return iter == m_regionToVIDs.end() ? std::vector<VID>() : iter->second;
}


template <typename MPTraits>
std::vector<typename TopologicalMap<MPTraits>::VID>
TopologicalMap<MPTraits>::
GetMappedVIDs(const std::vector<const WorkspaceRegion*>& _regions) const {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::GetMappedVIDs");

  // Find all VIDs that live within the region set.
  std::vector<VID> all;
  for(const auto& region : _regions) {
    // Skip empty regions.
    auto iter = m_regionToVIDs.find(region);
    if(iter == m_regionToVIDs.end())
      continue;

    // Copy these VIDs.
    const auto& vids = iter->second;
    std::copy(vids.begin(), vids.end(), std::back_inserter(all));
  }

  const size_t preUniqueSize = all.size();

  // Sort and make sure list is unique.
  std::sort(all.begin(), all.end());
  std::unique(all.begin(), all.end());

  // Check that we didn't double-add any vertices.
  if(preUniqueSize != all.size())
    throw RunTimeException(WHERE, "Unique removed vertices. Each vertex should "
        "be mapped to only one region.");

  return all;
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
GetMappedRegion(const VID _vid) const {
  return m_vidToRegion.at(_vid);
}


template <typename MPTraits>
bool
TopologicalMap<MPTraits>::
IsPopulated(const WorkspaceRegion* const _region) const {
  auto iter = m_regionToVIDs.find(_region);
  return iter != m_regionToVIDs.end() and !iter->second.empty();
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
GetRandomRegion() const {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::GetRandomRegion");
  auto decomposition = this->GetMPTools()->GetDecomposition(m_decompositionLabel);
  return &decomposition->GetRegion(LRand() % decomposition->GetNumRegions());
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
LocateRegion(const VID _vid) const {
  return LocateRegion(this->GetRoadmap()->GetGraph()->GetVertex(_vid));
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
LocateRegion(const CfgType& _cfg) const {
  return LocateRegion(_cfg.GetPoint());
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

  for(const WorkspaceRegion* candidate : candidateRegions)
    if(candidate->GetBoundary()->InBoundary(_point))
      return candidate;

  // If we get here, we didn't find a region.
  return nullptr;
}


template <typename MPTraits>
const WorkspaceRegion*
TopologicalMap<MPTraits>::
LocateNearestRegion(const CfgType& _c) const {
  return LocateNearestRegion(_c.GetPoint());
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
LocateCell(const VID _v) const {
  return LocateCell(this->GetRoadmap()->GetGraph()->GetVertex(_v));
}


template <typename MPTraits>
size_t
TopologicalMap<MPTraits>::
LocateCell(const CfgType& _cfg) const {
  return LocateCell(_cfg.GetPoint());
}


template <typename MPTraits>
size_t
TopologicalMap<MPTraits>::
LocateCell(const Point3d& _p) const {
  return m_grid->LocateCell(_p);
}


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

  auto region = LocateRegion(_vertex->property());
  const VID vid = _vertex->descriptor();

  // Map the VID to region and vis versa.
  m_regionToVIDs[region].push_back(vid);
  m_vidToRegion[vid] = region;
}


template<typename MPTraits>
void
TopologicalMap<MPTraits>::
UnmapCfg(const VI _vertex) {
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::UnmapCfg");

  const VID vid = _vertex->descriptor();

  // Assert that this vertex was mapped to a region.
  auto regionIter = m_vidToRegion.find(vid);

  if(regionIter == m_vidToRegion.end())
    throw RunTimeException(WHERE, "Tried to unmap vertex "
        + std::to_string(vid) + ", but didn't find a region for it.");

  // Assert that the region was mapped to the vertex.
  auto& regionMap = m_regionToVIDs[regionIter->second];
  auto vidIter = std::find(regionMap.begin(), regionMap.end(), vid);

  if(vidIter == regionMap.end())
    throw RunTimeException(WHERE, "Tried to unmap vertex "
        + std::to_string(vid) + ", but it was missing from it's region map.");


  // Remove this vertex from the mappings.
  regionMap.erase(vidIter);
  m_vidToRegion.erase(regionIter);
}


template <typename MPTraits>
void
TopologicalMap<MPTraits>::
ClearMap() {
  m_regionToVIDs.clear();
  m_vidToRegion.clear();
}

/*----------------------------------- SSSP -----------------------------------*/

template <typename MPTraits>
typename TopologicalMap<MPTraits>::SSSPOutput
TopologicalMap<MPTraits>::
ComputeSSSP(const WorkspaceRegion* const _region, const double _earlyStopDistance,
    const AdjacencyMap& _adjacency) {
  /// @TODO Homogenize our custom SSSP implementations. We have them now at
  ///       least in WorkspaceDecomposition, TopologicalMap, and QueryMethod.
  MethodTimer mt(this->GetStatClass(), "TopologicalMap::ComputeSSSP");
  const bool customAdjacency = !_adjacency.empty();
  const bool earlyStop = _earlyStopDistance != -1;

  if(this->m_debug)
    std::cout << "TopologicalMap::ComputeSSSP\n";

  SSSPOutput output;
  // Const cast is required because STAPL has an API error preventing useful
  // iteration over non-const graphs.
  auto decomposition = const_cast<WorkspaceDecomposition*>(
      this->GetMPTools()->GetDecomposition(m_decompositionLabel));


  /// An element in the PQ for dijkstra's, representing one instance of
  /// discovering a cell. Cells may be discovered multiple times from different
  /// parents with different distances.
  struct element {

    VD parent;         ///< The parent cell descriptor.
    VD vd;             ///< This cell descriptor.
    double distance;   ///< Computed distance at the time of insertion.

    /// Total ordering by decreasing distance.
    bool operator>(const element& _e) const noexcept {
      return distance > _e.distance;
    }

  };

  // Define a min priority queue for dijkstras. We will not update elements when
  // better distances are found - instead we will track the most up-to-date
  // distance and ignore elements with different values. This is effectively a
  // lazy delete of stale elements.
  std::priority_queue<element,
                      std::vector<element>,
                      std::greater<element>> pq;

  std::unordered_map<VD, bool> visited;
  std::unordered_map<VD, double> distance;

  // Define a relax edge function.
  auto relax = [&distance, this, &pq](EI& _ei) {
    const VD source = _ei->source(),
             target = _ei->target();
    const double sourceDistance = distance.count(source)
                                ? distance[source]
                                : std::numeric_limits<double>::infinity(),
                 targetDistance = distance.count(target)
                                ? distance[target]
                                : std::numeric_limits<double>::infinity(),
                 edgeWeight     = _ei->property().GetWeight(),
                 newDistance    = sourceDistance + edgeWeight;

    // If the new distance isn't better, quit.
    if(newDistance >= targetDistance)
      return;

    // Otherwise, update target distance and add the target to the queue.
    distance[target] = newDistance;
    pq.push(element{source, target, newDistance});
  };

  // Initialize the first node.
  const VD root = decomposition->GetDescriptor(*_region);
  distance[root] = 0;
  pq.push(element{root, root, 0});

  double maxDistance = std::numeric_limits<double>::max();
  bool foundFirstPopulated = false;

  // Dijkstras.
  while(!pq.empty()) {
    // Get the next element.
    element current = pq.top();
    pq.pop();

    // If we are done with this node, the element is stale. Discard.
    if(visited.count(current.vd))
      continue;
    visited[current.vd] = true;

    // Save this score and successor relationship.
    output.ordering.push_back(current.vd);
    output.distances[current.vd] = distance[current.vd];
    output.successors[current.parent].push_back(current.vd);

    if(this->m_debug)
      std::cout << "\tVertex " << current.vd
                << " has parent " << current.parent
                << ", score " << std::setprecision(4) << distance[current.vd]
                << ", and center at "
                << decomposition->GetRegion(current.vd).FindCenter()
                << std::endl;

    // Manage early stop conditions.
    if(earlyStop) {
      // If we haven't found the first populated region yet, check for it now.
      if(!foundFirstPopulated) {
        auto& region = decomposition->GetRegion(current.vd);
        if(IsPopulated(&region)) {
          foundFirstPopulated = true;
          maxDistance = distance[current.vd] + _earlyStopDistance;

          if(this->m_debug)
            std::cout << "\t\tFirst populated node found, max distance is "
                      << std::setprecision(4) << maxDistance << "."
                      << std::endl;
        }
      }
      // Otherwise, check for exceeding the max distance.
      else if(distance[current.vd] > maxDistance) {
        if(this->m_debug)
          std::cout << "\t\tLast populated node found."
                    << std::endl;
        break;
      }
    }


    // Relax each outgoing edge.
    auto vi = decomposition->find_vertex(current.vd);
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      // If we are not using custom adjacency, simply relax the edge.
      if(!customAdjacency)
        relax(ei);
      // Otherwise, only relax if this edge appears in _adjacency.
      else if(_adjacency.count(current.vd)) {
        const auto& neighbors = _adjacency.at(current.vd);
        auto iter = std::find(neighbors.begin(), neighbors.end(), ei->target());
        if(iter != neighbors.end())
          relax(ei);
      }
    }
  }

  // The root was added to its own successor map - fix that now.
  auto& rootMap = output.successors[root];
  rootMap.erase(std::find(rootMap.begin(), rootMap.end(), root));

  // The root was added to the front of the distance set, which we want.

  return output;
}

/*----------------------------------------------------------------------------*/

#endif
