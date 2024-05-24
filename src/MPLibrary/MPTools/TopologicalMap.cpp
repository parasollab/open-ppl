#include "TopologicalMap.h"

#include "MPLibrary/MPLibrary.h"
#include "Utilities/Conversions.h"

#include <algorithm>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/*------------------------------- Construction -------------------------------*/

TopologicalMap::TopologicalMap() : MPBaseObject("TopologicalMap") {
  this->SetName("TopologicalMap");
}

TopologicalMap::TopologicalMap(XMLNode& _node) : MPBaseObject(_node) {
  this->SetName("TopologicalMap");

  m_gridSize = _node.Read(
      "gridSize", true, 0., std::numeric_limits<double>::min(),
      std::numeric_limits<double>::max(),
      "The grid cell length. Very small values will cause memory overflow "
      "as we try to map huge quantities of grid cell to workspace region "
      "relations. Over-large values will cause slow mappings as we will need "
      "to check many candidate regions.");

  m_decompositionLabel = _node.Read("decompositionLabel", true, "",
                                    "The workspace decomposition to use.");

  m_pqpLabel = _node.Read(
      "cdLabel", false, "pqp_solid",
      "Optional collision detection method which can be used to locate nearest "
      "regions for points in obstacle space. This must be a method which "
      "provides proximity information (like PQP solid).");
}

TopologicalMap::~TopologicalMap() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

void TopologicalMap::Initialize() {
  // This object only works for single-robot problems right now.
  if (this->GetGroupTask())
    throw NotImplementedException(WHERE) << "Topological map does not yet "
                                         << "support robot groups.";

  // Initialize the maps.
  ClearCfgMaps();

  auto env = this->GetEnvironment();
  auto decomposition = this->GetDecomposition();

  // If we are debugging, write the decomposition file to OBJ for inspection.
  /// @todo Move this to the decomposition class, which currently cannot do
  ///       this job because it does not know its own label.
  if (this->m_debug)
    decomposition->WriteObj(this->GetLabel() + ".obj");

  // Initialize the grid and decomposition map.
  const std::string gridLabel = this->GetNameAndLabel() + "::GridOverlay";
  if (!m_grid.get()) {
    m_grid = std::unique_ptr<const GridOverlay>(
        new GridOverlay(env->GetBoundary(), m_gridSize));
    this->GetStatClass()->SetStat(gridLabel + "::Size", m_grid->Size());

    // If we're debugging, run the grid test to make sure it's working right.
    if (this->m_debug) {
      std::cout << "Testing grid (debug mode)..." << std::endl;
      m_grid->Test();
    }

    // Compute the mapping from grid cell to region.
    if (this->m_debug)
      std::cout << "Computing grid to region map." << std::endl;
    {
      /// @todo We could probably do this lazily if we use something like a
      ///       segment tree to find the regions which intersect a cell.
      MethodTimer mt(this->GetStatClass(),
                     gridLabel + "::ComputeDecompositionMap");
      m_cellToRegions = m_grid->ComputeDecompositionMap(decomposition, true);
    }

    // Compute the set of boundary cells.
    if (this->m_debug)
      std::cout << "Computing boundary grid cells." << std::endl;

    /// @TODO This exception should probably be a check on whether the robot's
    ///       largest minimum body radius is larger than the grid resolution, I
    ///       think probably at least 3x?
    if (env->UsingBoundaryObstacle())
      throw RunTimeException(WHERE) << "I'm pretty sure that using a boundary "
                                    << "obstacle with this won't work right, "
                                    << "unless your cell resolution is "
                                    << "significantly finer than the minimum "
                                    << "robot radius. Uncomment this exception "
                                    << "at your own peril!";

    MethodTimer mt(this->GetStatClass(), gridLabel + "::ComputeBoundary");

    m_boundaryCells.clear();
    const size_t numObstacles = env->NumObstacles();
    for (size_t i = 0; i < numObstacles; ++i) {
      const MultiBody* const obst = env->GetObstacle(i);
      for (const Body& b : obst->GetBodies()) {
        const auto& poly = b.GetWorldPolyhedron();
        const std::unordered_set<size_t> cells =
            m_grid->LocateCells(poly, {}, GridOverlay::CellSet::Interior);

        if (this->m_debug)
          std::cout << "\tFound " << cells.size() << " cells for obstacle " << i
                    << ", body " << b.GetIndex() << "." << std::endl;

        std::copy(cells.begin(), cells.end(),
                  std::inserter(m_boundaryCells, m_boundaryCells.end()));
      }
    }

    if (this->m_debug)
      std::cout << "Found " << m_boundaryCells.size() << " boundary cells."
                << std::endl;
  }
}

/*---------------------------------- Queries ---------------------------------*/

const typename TopologicalMap::VertexSet* TopologicalMap::GetMappedVIDs(
    RoadmapType* const _r,
    const WorkspaceRegion* const _region,
    const size_t _bodyIndex) const {
  MethodTimer mt(this->GetStatClass(),
                 this->GetNameAndLabel() + "::GetMappedVIDs");

  const auto& forwardMap = GetForwardMap(_r, _bodyIndex);

  auto iter = forwardMap.find(_region);
  return iter == forwardMap.end() ? nullptr : &iter->second;
}

const WorkspaceRegion* TopologicalMap::GetMappedRegion(
    RoadmapType* const _r,
    const VID _vid,
    const size_t _bodyIndex) const {
  const auto& inverseMap = GetInverseMap(_r, _vid);
  try {
    return inverseMap.at(_bodyIndex);
  } catch (const std::runtime_error& _e) {
    throw RunTimeException(WHERE)
        << "Inverse map for VID " << _vid << " has no entry for body index "
        << _bodyIndex << ".";
  }
}

bool TopologicalMap::IsPopulated(RoadmapType* const _r,
                                 const WorkspaceRegion* const _region,
                                 const size_t _bodyIndex) const {
  const auto& forwardMap = GetForwardMap(_r, _bodyIndex);
  auto iter = forwardMap.find(_region);
  return iter != forwardMap.end() and !iter->second.empty();
}

const WorkspaceRegion* TopologicalMap::GetRandomRegion() const {
  MethodTimer mt(this->GetStatClass(),
                 this->GetNameAndLabel() + "::GetRandomRegion");

  auto d = GetDecomposition();
  return &d->GetRegion(LRand() % d->GetNumRegions());
}

const WorkspaceRegion* TopologicalMap::LocateRegion(
    RoadmapType* const _r,
    const VID _vid,
    const size_t _bodyIndex) const {
  return LocateRegion(_r->GetVertex(_vid), _bodyIndex);
}

const WorkspaceRegion* TopologicalMap::LocateRegion(
    const Cfg& _cfg,
    const size_t _bodyIndex) const {
  _cfg.ConfigureRobot();
  auto body = _cfg.GetMultiBody()->GetBody(_bodyIndex);

  return LocateRegion(body->GetWorldTransformation().translation());
}

const WorkspaceRegion* TopologicalMap::LocateRegion(
    const Point3d& _point) const {
  MethodTimer mt(this->GetStatClass(),
                 this->GetNameAndLabel() + "::LocateRegion");

  // Find the grid cell that contains the new configuration's reference point.
  const size_t cell = m_grid->LocateCell(_point);

  // Find the correct region out of the candidates.
  const auto& candidateRegions = m_cellToRegions[cell];

  const WorkspaceRegion* r = nullptr;
  for (const WorkspaceRegion* region : candidateRegions) {
    if (region->GetBoundary()->InBoundary(_point)) {
      r = region;
      break;
    }
  }

  if (this->m_debug) {
    // Use pqp to check if _point lies within an obstacle.
    using CDType = CollisionDetectionValidity;
    auto vc = static_cast<CDType*>(
        this->GetMPLibrary()->GetValidityChecker(m_pqpLabel));
    const bool inObstacle = vc->IsInsideObstacle(_point);

    // Check for boundary containment.
    auto env = this->GetEnvironment()->GetBoundary();
    const bool inBoundary = env->InBoundary(_point);

    std::cout << "TopologicalMap::LocateRegion"
              << "\n\tPoint " << _point << "is " << (inObstacle ? "" : "not ")
              << "inside an obstacle."
              << "\n\tPoint " << _point << "is " << (inBoundary ? "" : "not ")
              << "inside the env boundary."
              << "\n\tCell: " << cell << " with center "
              << m_grid->CellCenter(cell) << ", width " << m_grid->CellLength()
              << "\n\tCandidate regions: " << candidateRegions.size()
              << "\n\tContaining region: " << r << std::endl;

    // If the point isn't in an obstacle and we have no region, then something
    // went wrong with the cell to region map.
    // Build an obj mesh of the candidate regions, and a small box for the
    // point. Merge these into a single obj and print to file for inspection.
    if (!inObstacle and !r) {
      glutils::triangulated_model t;

      // Add the env boundary so that we know roughly where in the environment
      // the problem occured.
      auto b = glutils::triangulated_model::make_box(env->GetRange(0).Length(),
                                                     env->GetRange(1).Length(),
                                                     env->GetRange(2).Length());
      {
        const std::vector<double>& center = env->GetCenter();
        b.translate(ToGLUtils(Point3d(center[0], center[1], center[2])));
        t.add_model(b);
      }

      // Add the candidate regions, and find their max bounding sphere size.
      double maxRadius = 0.;
      for (const WorkspaceRegion* region : candidateRegions) {
        // Add the facets.
        for (const WorkspaceRegion::Facet facet : region->GetFacets()) {
          t.add_facet(t.add_point(ToGLUtils(facet.GetPoint(0))),
                      t.add_point(ToGLUtils(facet.GetPoint(1))),
                      t.add_point(ToGLUtils(facet.GetPoint(2))));
        }

        // Find the center.
        const Point3d c = region->FindCenter();
        // Find max distance from center.
        for (const Point3d& p : region->GetPoints())
          maxRadius = std::max(maxRadius, (p - c).norm());
      }

      // Add the cell.
      auto c = glutils::triangulated_model::make_box(
          m_grid->CellLength(), m_grid->CellLength(), m_grid->CellLength());
      c.translate(ToGLUtils(m_grid->CellCenter(cell)));
      t.add_model(c);

      // Add the point as a small sphere of 1/20 the radius.
      auto p = glutils::triangulated_model::make_sphere(maxRadius / 20., 4);
      p.translate(ToGLUtils(_point));
      t.add_model(p);

      // Output the model to an obj file.
      {
        glutils::obj_file file("tm-error.obj");
        file << t;
      }

      throw RunTimeException(WHERE)
          << "Freespace point " << _point << " is not in any region.";
    }
  }

  return r;
}

const WorkspaceRegion* TopologicalMap::LocateNearestRegion(
    const Cfg& _c,
    const size_t _bodyIndex) const {
  _c.ConfigureRobot();
  auto body = _c.GetMultiBody()->GetBody(_bodyIndex);

  return LocateNearestRegion(body->GetWorldTransformation().translation());
}

const WorkspaceRegion* TopologicalMap::LocateNearestRegion(
    const Point3d& _p) const {
  MethodTimer mt(this->GetStatClass(),
                 this->GetNameAndLabel() + "::LocateNearestRegion");

  // Put the point robot here.
  auto pointRobot = this->GetMPProblem()->GetRobot("point");
  Cfg temp(_p, pointRobot);

  // Use PQPSolid to get the CD info. If the check is valid, this was already in
  // free space.
  CDInfo cdInfo(true);
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_pqpLabel);
  if (vc->IsValid(temp, cdInfo,
                  this->GetNameAndLabel() + "::LocateNearestRegion"))
    return LocateRegion(_p);

  // Compute the vector from the sampled point to the outside of the obstacle.
  const Vector3d delta = cdInfo.m_objectPoint - cdInfo.m_robotPoint;
  auto env = this->GetEnvironment();

  // The nearest possible free point is just outside of the obstacle along
  // delta.
  return LocateRegion(cdInfo.m_objectPoint +
                      delta.scale(env->GetPositionRes()));
}

size_t TopologicalMap::LocateCell(RoadmapType* const _r,
                                  const VID _v,
                                  const size_t _bodyIndex) const {
  return LocateCell(_r->GetVertex(_v), _bodyIndex);
}

size_t TopologicalMap::LocateCell(const Cfg& _cfg,
                                  const size_t _bodyIndex) const {
  _cfg.ConfigureRobot();
  auto body = _cfg.GetMultiBody()->GetBody(_bodyIndex);

  return LocateCell(body->GetWorldTransformation().translation());
}

size_t TopologicalMap::LocateCell(const Point3d& _p) const {
  return m_grid->LocateCell(_p);
}

/*---------------------------- Neighborhood Keys -----------------------------*/

typename TopologicalMap::NeighborhoodKey TopologicalMap::LocateNeighborhood(
    RoadmapType* const _r,
    const VID _v) const {
  return LocateNeighborhood(_r->GetVertex(_v));
}

typename TopologicalMap::NeighborhoodKey TopologicalMap::LocateNeighborhood(
    const Cfg& _c) const {
  MethodTimer mt(this->GetStatClass(),
                 this->GetNameAndLabel() + "::LocateNeighborhood");

  // Create a key with storage for each body.
  auto mb = _c.GetMultiBody();
  NeighborhoodKey key(mb->GetNumBodies(), nullptr);

  // Locate the region occupied by each body's centroid.
  _c.ConfigureRobot();
  for (size_t i = 0; i < key.size(); ++i) {
    auto body = mb->GetBody(i);
    key[i] = LocateRegion(body->GetWorldTransformation().translation());
  }

  return key;
}

/*--------------------------- Decomposition Access ---------------------------*/

inline const WorkspaceDecomposition* TopologicalMap::GetDecomposition() const {
  return this->GetMPLibrary()->GetMPTools()->GetDecomposition(
      m_decompositionLabel);
}

/*-------------------------------- Cfg Mapping -------------------------------*/

void TopologicalMap::EnsureMap(RoadmapType* const _r) {
  // If we already have maps for _r, do nothing.
  if (m_regionToVIDs.count(_r))
    return;

  // Create forward and inverse maps.
  auto& forward = m_regionToVIDs[_r];
  auto& inverse = m_vidToNeighborhood[_r];

  // Set the size of the mappings.
  auto mb = _r->GetRobot()->GetMultiBody();
  forward.resize(mb->GetNumBodies());
  inverse.reserve(_r->Size());

  // Install roadmap hooks.
  _r->InstallHook(RoadmapType::HookType::AddVertex, this->GetNameAndLabel(),
                  [this, _r](const VI _vi) { this->MapCfg(_r, _vi); });
  _r->InstallHook(RoadmapType::HookType::DeleteVertex, this->GetNameAndLabel(),
                  [this, _r](const VI _vi) { this->UnmapCfg(_r, _vi); });

  // If the graph has existing vertices, map them now.
  for (auto iter = _r->begin(); iter != _r->end(); ++iter)
    MapCfg(_r, iter);
}

void TopologicalMap::MapCfg(RoadmapType* const _r, const VI _vertex) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::MapCfg");

  const auto& cfg = _vertex->property();
  const VID vid = _vertex->descriptor();

  // Inverse-map the VID to the regions.
  const auto neighborhood = LocateNeighborhood(cfg);
  m_vidToNeighborhood[_r][vid] = neighborhood;

  // Forward-map the regions to the VID.
  auto& forward = m_regionToVIDs[_r];
  for (size_t i = 0; i < neighborhood.size(); ++i) {
    auto region = neighborhood[i];
    forward[i][region].insert(vid);
  }
}

void TopologicalMap::UnmapCfg(RoadmapType* const _r, const VI _vertex) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::UnmapCfg");

  const VID vid = _vertex->descriptor();

  // Assert that this vertex was mapped to a neighborhood.
  auto& inverse = m_vidToNeighborhood[_r];
  auto neighborhoodIter = inverse.find(vid);
  if (neighborhoodIter == inverse.end())
    throw RunTimeException(WHERE) << "Tried to unmap vertex " << vid
                                  << ", but didn't find a neighborhood for it.";

  // Remove the mapping from each neighborhood region to this VID.
  const auto& neighborhood = neighborhoodIter->second;
  auto& forward = m_regionToVIDs[_r];
  for (size_t i = 0; i < neighborhood.size(); ++i) {
    const auto region = neighborhood[i];

    // Assert that the region was mapped to the vertex.
    auto& regionMap = forward[i][region];
    auto vidIter = std::find(regionMap.begin(), regionMap.end(), vid);

    if (vidIter == regionMap.end())
      throw RunTimeException(WHERE) << "Tried to unmap vertex " << vid
                                    << ", but it was missing from it's region "
                                    << "map.";

    // Remove this vertex from the map.
    regionMap.erase(vidIter);
  }

  inverse.erase(neighborhoodIter);
}

void TopologicalMap::ClearCfgMaps() {
  m_regionToVIDs.clear();
  m_vidToNeighborhood.clear();
}

inline const typename TopologicalMap::OccupancyMap&
TopologicalMap::GetForwardMap(RoadmapType* const _r,
                              const size_t _bodyIndex) const {
  const_cast<TopologicalMap*>(this)->EnsureMap(_r);
  try {
    return m_regionToVIDs.at(_r).at(_bodyIndex);
  } catch (const std::runtime_error& _e) {
    throw RunTimeException(WHERE) << "No forward map for "
                                  << "roadmap " << _r << ", "
                                  << "body index " << _bodyIndex << ".";
  }
}

inline const typename TopologicalMap::NeighborhoodKey&
TopologicalMap::GetInverseMap(RoadmapType* const _r, const VID _vid) const {
  const_cast<TopologicalMap*>(this)->EnsureMap(_r);
  try {
    return m_vidToNeighborhood.at(_r).at(_vid);
  } catch (const std::runtime_error& _e) {
    throw RunTimeException(WHERE) << "No inverse map for "
                                  << "roadmap " << _r << ", "
                                  << "VID " << _vid << ".";
  }
}

/*--------------------------- Inter-Region Distance --------------------------*/

double TopologicalMap::ApproximateMinimumInnerDistance(
    const WorkspaceRegion* const _source,
    const WorkspaceRegion* const _target) {
  MethodTimer mt(this->GetStatClass(),
                 this->GetNameAndLabel() + "::ApproximateMinimumInnerDistance");

  // First check for the trivial case.
  if (_source == _target)
    return 0;

  // Next check if we've alreay cached inner distances from this source.
  auto sourceIter = m_innerDistanceMap.find(_source);
  const bool sourceCached = sourceIter != m_innerDistanceMap.end();
  if (!sourceCached)
    return std::numeric_limits<double>::infinity();

  // Next look for the target entry in the source map.
  const auto& sourceMap = sourceIter->second;
  auto targetIter = sourceMap.find(_target);
  const bool targetCached = targetIter != sourceMap.end();
  return targetCached ? targetIter->second
                      : std::numeric_limits<double>::infinity();
}

const typename TopologicalMap::DistanceMap&
TopologicalMap::ComputeApproximateMinimumInnerDistances(
    const WorkspaceRegion* const _source,
    const double _radius) {
  /// @todo Refactor this and SSSP code to unify implementations. I think it
  ///       should be feasible to make an adaptor class to give our grid (which
  ///       represents an implicit graph) an API that is compatible with the
  ///       STAPL (explicit) graph.

  DistanceMap* distanceMap{nullptr};

  // Check if we've already computed for this source.
  {
    auto iter = m_innerDistanceMap.find(_source);
    const bool alreadyComputed = iter != m_innerDistanceMap.end();

    // If we already computed this cell, don't do it again unless the radius is
    // bigger.
    if (alreadyComputed) {
      distanceMap = &iter->second;
      const double lastRadius = distanceMap->at(nullptr);
      if (lastRadius >= _radius)
        return *distanceMap;
    }

    // We haven't computed this distance map to the necessary radius.
    // (Re)compute with this radius.
    distanceMap = &m_innerDistanceMap[_source];
    (*distanceMap)[nullptr] = _radius;
  }

  // Define a function for updating the minimum inner distance to the regions
  // touching a given cell.
  auto updateInnerDistance = [_source, this, distanceMap](
                                 const size_t _cell, const double _distance) {
    // Find all regions touching the cell.
    const auto& regions = this->m_cellToRegions[_cell];

    // Set the distance if it isn't already set.
    for (const WorkspaceRegion* const region : regions) {
      if (distanceMap->count(region))
        continue;
      distanceMap->emplace(region, _distance);
    }
  };

  /// A search element in the priority queue.
  struct element {
    size_t cell;      ///< The visited cell.
    double distance;  ///< The distance to the nearest search root from cell.

    element(const size_t _target, const double _distance)
        : cell(_target), distance(_distance) {}

    /// Total ordering by increasing distance.
    bool operator>(const element& _other) const noexcept {
      return distance > _other.distance;
    }
  };

  // Set up a best-first search through the grid overlay to estimate the minimum
  // inner distance.
  std::unordered_set<size_t> visited;
  std::unordered_map<size_t, double> distance;
  std::unordered_map<size_t, size_t> parent;
  std::priority_queue<element, std::vector<element>, std::greater<element>> pq;

  // Find the source and target cells.
  const Boundary* const sourceBoundary = _source->GetBoundary();
  const std::unordered_set<size_t> sourceCells =
      m_grid->LocateCells(sourceBoundary, GridOverlay::CellSet::Closure);

  // Initialize visited and distance maps.
  {
    // Mark all source cells as visited and distance 0.
    for (const size_t cell : sourceCells) {
      visited.insert(cell);
      distance[cell] = 0;
      updateInnerDistance(cell, 0);
    }

    // Find all neighbors of the source cells.
    std::unordered_set<size_t> allNeighbors;
    for (const size_t cell : sourceCells) {
      std::unordered_set<size_t> neighbors = m_grid->LocateAllNeighbors(cell);
      allNeighbors.insert(neighbors.begin(), neighbors.end());
    }

    // Put the neighbors in the queue as visited @ distance 0.
    for (const size_t neighbor : allNeighbors) {
      // Skip visited cells.
      if (visited.count(neighbor))
        continue;
      // Skip boundary cells.
      if (m_boundaryCells.count(neighbor))
        continue;

      // updateInnerDistance(neighbor, 0);
      // visited.insert(neighbor);
      distance[neighbor] = 0;
      parent[neighbor] = neighbor;
      pq.emplace(neighbor, 0);
    }
  }

  // Define a relax edge function.
  const double cellLength = this->m_grid->CellLength();
  auto relax = [&distance, &pq, &cellLength, &parent, this](
                   const size_t _source, const size_t _target) {
    // If the target cell is a boundary cell, quit.
    if (m_boundaryCells.count(_target))
      return;

    // Compute the new distance.
    const double sourceDistance = distance[_source],
                 targetDistance = distance.count(_target)
                                      ? distance[_target]
                                      : std::numeric_limits<double>::infinity(),
                 newDistance = sourceDistance + cellLength;

    // If the new distance isn't better, quit.
    if (newDistance >= targetDistance)
      return;

    // Otherwise, update target distance and add the target to the queue.
    distance[_target] = newDistance;
    parent[_target] = _source;
    pq.emplace(_target, newDistance);
  };

  // Run Dijkstra's algorithm until discovering a cell with min distance >
  // _radius.
  while (!pq.empty()) {
    // Get the next element.
    const element current = pq.top();
    pq.pop();

    // If we are done with this node, the element is stale. Discard.
    if (visited.count(current.cell))
      continue;
    visited.insert(current.cell);

    // Check for early termination.
    const bool stop = current.distance > _radius;
    if (stop)
      break;

    // Update the inner distance map for all regions touched by this cell.
    updateInnerDistance(current.cell, current.distance);

    // Relax each outgoing edge.
    const std::unordered_set<size_t> neighbors =
        m_grid->LocateAllNeighbors(current.cell);
    for (const size_t n : neighbors)
      if (!visited.count(n))
        relax(current.cell, n);
  }

  return *distanceMap;
}

/*----------------------------------------------------------------------------*/