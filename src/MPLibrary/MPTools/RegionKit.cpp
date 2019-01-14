#include "RegionKit.h"

#include <random>

#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"
#include "Utilities/MPUtils.h"
#include "Utilities/XMLNode.h"


/*------------------------------- Construction -------------------------------*/

RegionKit::
RegionKit(XMLNode& _node) {
  m_debug = _node.Read("debug", false, m_debug, "Show debug messages");

  m_explore = _node.Read("explore", true, m_explore, 0., 1.,
      "Weight of explore vs. exploit in region selection probabilities");

  m_regionFactor = _node.Read("regionFactor", true,
      m_regionFactor, 1., std::numeric_limits<double>::max(),
      "Regions are this * robot's bounding sphere radius");

  m_penetrationFactor = _node.Read("penetration", true,
      m_penetrationFactor, std::numeric_limits<double>::min(), 1.,
      "Fraction of bounding sphere penetration that is considered touching");
}


RegionKit::
~RegionKit() {
  Clear();
}


void
RegionKit::
Clear() {
  // Release any remaining dynamic regions.
  for(auto& keyValue : m_regionData)
    delete keyValue.first;
  m_regionData.clear();
  m_visited.clear();
}

/*--------------------------------- Sampling ---------------------------------*/

const Boundary*
RegionKit::
SelectRegion() {
  // Update all region probabilities.
  vector<double> probabilities = ComputeProbabilities();

  // Construct with random number generator with the region probabilities.
  static std::default_random_engine generator(0);
  std::discrete_distribution<size_t> distribution(probabilities.begin(),
      probabilities.end());

  const size_t index = distribution(generator);
  const bool envSelected = index == m_regionData.size();

  if(m_debug) {
    std::cout << "RegionKit:: updated region selection probabilities ("
              << "last is whole env):\n\t";

    for(auto p : distribution.probabilities())
      std::cout << std::setprecision(4) << p << " ";

    std::cout << "\n\tSelected index " << index
              << (envSelected ? " (whole env)." : " ")
              << std::endl;
  }

  if(envSelected)
    return nullptr;

  // Get the selected region.
  auto it = m_regionData.begin();
  advance(it, index);

  if(m_debug) {
    auto c = it->first->GetCenter();
    std::cout << "\tRegion is " << it->first << " with center at "
              << Vector3d(c[0], c[1], c[2]) << ", success rate so far "
              << it->second.successes << " / " << it->second.attempts
              << "." << std::endl;
  }

  // total samples increment
  ++it->second.attempts;

  return it->first;
}


void
RegionKit::
IncrementFailure(const Boundary* const _region, const size_t _count) {
  // _region should be a const handle to a non-const boundary owned by this
  // object, so we should be safe to cast away the const.
  Boundary* region = const_cast<Boundary*>(_region);

  // Ensure that this region exists.
  auto iter = m_regionData.find(region);
  if(iter == m_regionData.end()) {
    ostringstream oss;
    oss << "Cannot increment success for non-existing region '" << region << "'";
    throw RunTimeException(WHERE, oss.str());
  }

  iter->second.attempts += _count;
}


void
RegionKit::
IncrementSuccess(const Boundary* const _region, const size_t _count) {
  // _region should be a const handle to a non-const boundary owned by this
  // object, so we should be safe to cast away the const.
  Boundary* region = const_cast<Boundary*>(_region);

  // Ensure that this region exists.
  auto iter = m_regionData.find(region);
  if(iter == m_regionData.end()) {
    ostringstream oss;
    oss << "Cannot increment attempts for non-existing region '" << region << "'";
    throw RunTimeException(WHERE, oss.str());
  }

  iter->second.successes += _count;
  iter->second.attempts  += _count;
}


const Vector3d
RegionKit::
GetVelocityBias(const Boundary* const _region) const {
  // _region should be a const handle to a non-const boundary owned by this
  // object, so we should be safe to cast away the const.
  Boundary* region = const_cast<Boundary*>(_region);

  // Get the region data.
  const auto& regionData = m_regionData.at(region);
  const size_t index = regionData.edgeIndex;

  // Find the skeleton edge path the region is traversing.
  auto edge = m_skeleton->FindEdge(regionData.edgeDescriptor);
  const auto& path = edge->property();

  // Helper to make the biasing direction and print debug info.
  auto makeBias = [&](const Vector3d& _start, const Vector3d& _end) {
    if(m_debug)
      std::cout << "Computed velocity bias: " << (_end - _start).normalize()
                << "\n\tStart: " << _start
                << "\n\tEnd:   " << _end
                << std::endl;
    return (_end - _start).normalize();
  };

  // If there is at least one valid path point after the current path index,
  // then return the direction to the next point.
  if(index < path.size() - 1) {
    if(m_debug)
      std::cout << "Biasing velocity along next path step"
                << "\n\tPath index: " << index
                << "\n\tPath size:  " << path.size()
                << std::endl;
    return makeBias(path[index], path[index + 1]);
  }

  // Otherwise, the region has reached a skeleton vertex.
  auto vertex = m_skeleton->GetGraph().find_vertex(edge->target());

  // If the vertex has no outgoing edges, this is the end of the skeleton. In
  // that case, use the previous biasing direction. All paths have at least two
  // points so this is safe.
  if(vertex->size() == 0) {
    if(m_debug)
      std::cout << "Biasing velocity along previous path step"
                << "\n\tPath index: " << index
                << "\n\tPath size:  " << path.size()
                << std::endl;
    return makeBias(path[index - 1], path[index]);
  }

  // Otherwise, randomly select an outgoing and use it's next point.
  auto eit = vertex->begin();
  const size_t nextEdgeIndex = LRand() % vertex->size();
  std::advance(eit, nextEdgeIndex);
  if(m_debug)
    std::cout << "Biasing velocity along next edge (index " << nextEdgeIndex
              << ")\n\tPath index: " << index
              << "\n\tPath size:  " << path.size()
              << "\n\tNext edge path size: " << eit->property().size()
              << std::endl;
  return makeBias(path[index], eit->property()[1]);
}

/*------------------------------------ I/O -----------------------------------*/

void
RegionKit::
Print(std::ostream& _os) const {
  _os << "RegionKit"
      << "\n\tRegion factor: " << m_regionFactor
      << "\n\tRegion radius: " << m_regionRadius
      << "\n\tPreference for explore in [0:1]:" << m_explore
      << "\n\tBounding sphere penetration fraction in [0:1]:"
      << m_penetrationFactor
      << std::endl;
}

/*---------------------------- Skeleton Following ----------------------------*/

void
RegionKit::
InitRegions(const Point3d& _start) {
  // Mark all nodes unvisited.
  m_visited.clear();
  WorkspaceSkeleton::GraphType& g = m_skeleton->GetGraph();
  for(auto vit = g.begin(); vit != g.end(); ++vit)
    m_visited[vit->descriptor()] = false;

  // Find the vertex nearest to start.
  auto sit = m_skeleton->FindNearestVertex(_start);

  // Spark a region for each outgoing edge from _start.
  for(auto eit = sit->begin(); eit != sit->end(); ++eit) {
    auto r = new WorkspaceBoundingSphere(sit->property(), m_regionRadius);
    m_regionData.emplace(r, RegionData(eit->descriptor()));
  }
  m_visited[sit->descriptor()] = true;

  if(m_debug)
    std::cout << "RegionKit:: initialized new region with radius "
              << std::setprecision(4) << m_regionRadius
              << " at vertex " << sit->descriptor()
              << " nearest to point " << _start << "."
              << std::endl;
}


void
RegionKit::
CreateRegions(const Point3d& _p) {
  auto& g = m_skeleton->GetGraph();

  // Check each skeleton node to see if a new region should be created.
  for(auto vit = g.begin(); vit != g.end(); ++vit) {
    // Skip skeleton nodes that are already visited.
    if(m_visited[vit->descriptor()])
      continue;

    // Skip skeleton nodes that are too far away.
    const double dist = (vit->property() - _p).norm();
    if(dist >= m_regionRadius)
      continue;

    // If we're still here, the region is in range and unvisited.
    // Create a new region for each outgoing edge of this skeleton node.
    for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
      auto r = new WorkspaceBoundingSphere(vit->property(), m_regionRadius);
      m_regionData.emplace(r, RegionData{eit->descriptor()});

      if(m_debug)
        std::cout << "RegionKit:: created new region with radius "
                  << std::setprecision(4) << m_regionRadius
                  << " on edge (" << eit->source() << ", "
                  << eit->target() << ", " << eit->id() << ") "
                  << "nearest to point " << _p << "."
                  << std::endl;
    }
    m_visited[vit->descriptor()] = true;
  }
}


void
RegionKit::
AdvanceRegions(const Cfg& _cfg) {
  // Iterate all regions and see if any regions contain the new cfg
  // If a region contains the new cfg, advance it along the flow edge and
  // until the new region we get from this is at the end of the flow edge or
  // it no longer contains the cfg

  if(m_debug)
    std::cout << "RegionKit:: checking " << m_regionData.size()
              << " regions for contact with new configuration."
              << std::endl;

  // Iterate through all regions to see which should be advanced.
  for(auto iter = m_regionData.begin(); iter != m_regionData.end(); ) {
    Boundary* region = iter->first;
    bool increment = true;

    // Advance this region until the robot at _cfg is no longer touching it.
    while(IsTouching(_cfg, region)) {
      // Get region data and the edge path it is traversing.
      auto& data = m_regionData[region];
      const vector<Point3d>& path = m_skeleton->FindEdge(data.edgeDescriptor)->
          property();
      size_t& i = data.edgeIndex;

      // If there are more points left on this edge, advance the region and
      // index.
      if(i < path.size() - 1) {
        const Point3d& current = path[i];
        const Point3d& next = path[++i];
        region->SetCenter({next[0], next[1], next[2]});

        if(m_debug)
          std::cout << "\tAdvancing region " << region << " from index "
                    << i - 1 << " to " << i << " / " << path.size()
                    << "(" << (next - current).norm() << " units)."
                    << std::endl;
      }
      // If not, the region has finished traversing its edge and must be
      // deleted.
      else {
        if(m_debug)
          std::cout << "\tRegion " << region << " has reached the end of its "
                    << "path, erasing it now. " << m_regionData.size() - 1
                    << " regions remain."
                    << std::endl;

        iter = m_regionData.erase(iter);

        delete region;
        region = nullptr;
        increment = false;

        break;
      }
    }
    if(increment && iter != m_regionData.end())
      ++iter;
  }
}


bool
RegionKit::
IsTouching(const Cfg& _cfg, const Boundary* const _region) const {
  // Compute the penetration distance required. We want the robot's bounding
  // sphere to penetrate the region by the fraction m_penetrationThreshold.
  const double robotRadius  = _cfg.GetMultiBody()->GetBoundingSphereRadius(),
               threshold    = 2 * robotRadius * m_penetrationFactor;

  // Compute the penetration distance.
  const Point3d robotCenter = _cfg.GetPoint();
  const double penetration = _region->GetClearance(robotCenter) + robotRadius;

  // The configuration is touching if the penetration exceeds the threshold.
  const bool touching = penetration >= threshold;

  if(m_debug)
    std::cout << "\tNew Cfg's center penetrates "
              << std::setprecision(4) << _region->GetClearance(robotCenter)
              << " units into the region center."
              << "\n\t  Bounding sphere radius: " << robotRadius
              << "\n\t  Region radius: " << m_regionRadius
              << "\n\t  Bounding sphere penetrates by "
              << std::setprecision(4) << penetration << " units."
              << "\n\t  Robot is" << (touching ? "" : " not")
              << " within 'touching' threshold of "
              << std::setprecision(4) << threshold
              << " units."
              << std::endl;

  return touching;
}


std::vector<double>
RegionKit::
ComputeProbabilities() {
  /// @TODO The current implementation is O(|regions|) time, but we should be
  ///       able to do it in O(1) by keeping a running total weight and adjusting
  ///       it and the region weight after each sampling attempt. Main blocker
  ///       at this time is the probability distribution object, which we could
  ///       easily manually re-implement. Region deletion will still be
  ///       O(|regions|) though because we need to update all the probabilites.
  ///       Even an incremental solution here will be O(|regions|) on average.

  // Sum all weights of all current regions.
  double totalWeight = 0.;
  for(auto& keyValue : m_regionData) {
    auto& regionInfo = keyValue.second;

    // Compute weight of this region.
    regionInfo.weight = regionInfo.successes /
        static_cast<double>(regionInfo.attempts);

    // Add to total.
    totalWeight += regionInfo.weight;
  }

  // Compute the probabilities for the current regions.
  std::vector<double> probabilities;
  probabilities.reserve(m_regionData.size() + 1);

  const double explore = m_explore / (m_regionData.size() + 1);

  for(const auto& keyValue : m_regionData) {
    const auto& weight = keyValue.second.weight;
    const double exploit = (1 - m_explore) * weight / totalWeight;

    probabilities.emplace_back(exploit + explore);
  }

  // Get the probability for the whole environment.
  probabilities.emplace_back(explore);

  return probabilities;
}

/*----------------------------------------------------------------------------*/
