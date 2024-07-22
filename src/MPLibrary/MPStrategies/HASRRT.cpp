#include "HASRRT.h"

#include "MPLibrary/MPLibrary.h"
#include "MPProblem/Constraints/Constraint.h"

#include "MPLibrary/MPTools/MeanCurvatureSkeleton3D.h"
#include "MPLibrary/MPTools/ReebGraphConstruction.h"
#include "Utilities/MedialAxis2D.h"
#include "Utilities/XMLNode.h"
#include "Utilities/MPUtils.h"


/*------------------------------ Construction --------------------------------*/

HASRRT::
HASRRT() : BasicRRTStrategy() {
    this->SetName("HASRRT");
}

HASRRT::
HASRRT(XMLNode& _node) : BasicRRTStrategy(_node) {
  this->SetName("HASRRT");

  m_skeletonType = _node.Read("skeletonType", true, "",
      "the type of skeleton to use, Available options are reeb and mcs "
      "for 3d, ma for 2d");

  m_inputSkeleton = _node.Read("inputSkeleton", false, "", "the input skeleton file "
      "if already constructed");

  m_outputSkeleton = _node.Read("outputSkeleton", false, "", "the output skeleton file");

  // If using a reeb skeleton, we need a decomposition to build it.
  m_decompositionLabel = _node.Read("decompositionLabel",
      m_skeletonType == "reeb", "",
      "The workspace decomposition to use.");

  m_scuLabel = _node.Read("scuLabel", false, "", "The skeleton clearance utility "
      "to use. If not specified, we use the hack-fix from wafr16.");

  m_directSkeleton = _node.Read("directSkeleton", false, m_directSkeleton,
      "Direct the skeleton from the start cfg");

  m_refineEdges = _node.Read("refineEdges", false, m_refineEdges,
      "Refine the skeleton edges to shorten long edges");

  m_velocityBiasing = _node.Read("velocityBiasing", false, m_velocityBiasing,
      "Bias nonholonomic samples along the skeleton?");

  m_velocityAlignment = _node.Read("velocityAlignment", false,
      m_velocityAlignment, -1., .99,
      "Minimum dot product for sampled velocity and biasing direction.");

  m_explore = _node.Read("explore", true, m_explore, 0., 1.,
      "Weight of explore vs. exploit in region selection probabilities");

  m_regionFactor = _node.Read("regionFactor", true,
      m_regionFactor, 1., std::numeric_limits<double>::max(),
      "Regions are this * robot's bounding sphere radius");

  m_penetrationFactor = _node.Read("penetration", true,
      m_penetrationFactor, std::numeric_limits<double>::min(), 1.,
      "Fraction of bounding sphere penetration that is considered touching");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void
HASRRT::
Print(std::ostream& _os) const {
  BasicRRTStrategy::Print(_os);

  _os << "\tSkeleton Type:" << m_skeletonType << std::endl;

  if(!m_decompositionLabel.empty())
    _os << "\tWorkspace Decomposition Label:" << m_decompositionLabel << std::endl;

  if(!m_scuLabel.empty())
    _os << "\tSkeleton Clearance Utility:" << m_scuLabel << std::endl;

  _os << "\tVelocity Biasing: " << m_velocityBiasing << std::endl;
  _os << "\tVelocity Alignment: " << m_velocityAlignment << std::endl;

  _os << "\tRegion Factor: " << m_regionFactor << std::endl;
  _os << "\tRegion Radius: " << m_regionRadius << std::endl;
  _os << "\tPenetration Factor: " << m_penetrationFactor << std::endl;
  _os << "\tExploration Factor: " << m_explore << std::endl;
}

/*---------------------------- MPStrategy Overrides --------------------------*/

void
HASRRT::
Initialize() {

  BasicRRTStrategy::Initialize();

  // Check that only one direction is being extended.
  if (this->m_numDirections > 1)
    throw RunTimeException(WHERE) << "Extending more than one direction "
      << "is not supported.";

  // Disable velocity biasing if the robot is holonomic.
  m_velocityBiasing &= this->GetTask()->GetRobot()->IsNonholonomic();

  const double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->
    GetBoundingSphereRadius();
  m_regionRadius = m_regionFactor * robotRadius;

  m_parentRegions.clear();
  m_regions.clear();

  // Initialize the skeleton, local components, and regions.
  BuildSkeleton();

  if(this->m_debug)
    std::cout << "The directed skeleton has "
      << m_skeleton.get_num_vertices()
      << " vertices"
      << std::endl;
  if(this->m_debug) {
    for(auto it = m_skeleton.begin(); it != m_skeleton.end(); it++)
      std::cout << it->descriptor() << " ";
    std::cout << std::endl;
  }


  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::InitializeRoadmap");
  if(this->m_debug)
    std::cout << "Initializing region at start skeleton vertex."
      << std::endl;

  // Mark all nodes unvisited.
  m_visited.clear();
  for(auto vit = m_skeleton.begin(); vit != m_skeleton.end(); ++vit)
    m_visited[vit->descriptor()] = false;

  // Find the vertex nearest to start and create regions for each outgoing
  // edge.
  MakeQuery();
  auto iter = m_skeleton.FindNearestVertex(m_queryPair.first);
  CreateRegions(iter);
}


void
HASRRT::
Iterate() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::Iterate");

  //this->Finalize();
  // Find growth target.
  const Cfg target = this->SelectTarget();

  auto stats = this->GetStatClass();
  const std::string id = this->GetNameAndLabel() + "::SelectTargetCount";
  stats->IncStat(id);

  // Expand the tree from nearest neigbor to target.
  if(!m_selectedWholeEnv and m_regions.size() > 0) {
    const auto neighborCandidates = m_regions[m_selectedRegionIndex].samples;

    vector<VID> pnc;
    if(m_regions[m_selectedRegionIndex].previousRegionIndex != -1) {
      pnc = m_parentRegions[m_regions[m_selectedRegionIndex].previousRegionIndex].samples;
    }

    const auto previousNeighborCandidates = pnc;

    VID nearest;
    if(neighborCandidates.empty())
      if(previousNeighborCandidates.empty())
        nearest = this->FindNearestNeighbor(target);
      else
        previousNeighborCandidates.back();
        //nearest = this->FindNearestNeighbor(target, &previousNeighborCandidates);
    else
      nearest = neighborCandidates.back();
      //nearest = this->FindNearestNeighbor(target, &neighborCandidates);

    const VID nearestVID = nearest;

    if(nearestVID == INVALID_VID)
      return;
  
    const VID newVID = this->ExpandTree(nearestVID, target);


    if(newVID != INVALID_VID)  {

    // If, in the expansion process, we have finished exploring the skeleton and have no more regions left, don't bother with regoin bookkeeping.
      if(m_regions.size() != 0) {
        m_regions[m_selectedRegionIndex].samples.push_back(newVID);
      }
      

      // If growing goals, try to connect other trees to the new node. Otherwise
      // check for a goal extension.
      if(this->m_growGoals)
        this->ConnectTrees(newVID);
      else
        this->TryGoalExtension(newVID);
    }
    else {
      auto nearestCfg = this->GetRoadmap()->GetVertex(nearestVID);
      if(this->m_debug)
        std::cout << "Advance region based on nearestCfg"
                  << "\n Edge it now on " << m_regions[m_selectedRegionIndex].edgeIndex
                  << std::endl;

      m_regions[m_selectedRegionIndex].edgeIndex = GetBinaryIntermediate(nearestCfg, m_regions[m_selectedRegionIndex], true);
    }
  }

  else {
    auto envstats = this->GetStatClass();
    const std::string id2 = this->GetNameAndLabel() + "::WholeEnvCount";
    envstats->IncStat(id2);
    const VID nearestVID = this->FindNearestNeighbor(target);
    if(nearestVID != INVALID_VID) {
      const VID newVID = this->ExpandTree(nearestVID, target);
      if(newVID != INVALID_VID)  {

        // If growing goals, try to connect other trees to the new node. Otherwise
        // check for a goal extension.
        if(this->m_growGoals)
          this->ConnectTrees(newVID);
        else
          this->TryGoalExtension(newVID);
      }
    }
  }

}

/*------------------------ BasicRRTStrategy Overrides ------------------------*/

Cfg
HASRRT::
SelectTarget() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SelectTarget");

  // Get the sampler.
  const std::string* samplerLabel = &this->m_samplerLabel;

  // Select goal growth with probability m_growthFocus.
  auto goalTracker = this->GetMPLibrary()->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();

  if(unreachedGoals.size() and DRand() < this->m_growthFocus) {
    // Randomly select a goal constraint boundary.
    const auto& goalConstraints = this->GetTask()->GetGoalConstraints();
    const size_t index = unreachedGoals[LRand() % unreachedGoals.size()];
    const Boundary* const b = goalConstraints[index]->GetBoundary();

    // We may eventually support constraints that cannot be described in terms
    // of a boundary, but that is outside the scope of the present
    // implementation.
    if (!b)
      throw NotImplementedException(WHERE) << "Non-boundary constraints are not "
                                           << "yet supported.";

    // If there is a query sampler, use that for goal sampling.
    if (!this->m_querySampler.empty())
      samplerLabel = &this->m_querySampler;

    if (this->m_debug)
      std::cout << "Sampling growth target from goal " << index
                << " (sampler '" << *samplerLabel << "'):"
                << std::endl;

    m_selectedWholeEnv = true;
    return Sample(b, samplerLabel);
  }

  // Otherwise, use the designated sampler with the region (or environment)
  // boundary.
  // Select a region for sample generation.
  const size_t regionIdx = SelectSamplingRegion();

  // If we received a null boundary, use the full environment.
  if (regionIdx < m_regions.size()) {
    m_selectedRegionIndex = regionIdx;
    m_selectedWholeEnv = false;
    return Sample(&m_regions[regionIdx]);
  }
  else {
    m_selectedWholeEnv = true;
    return Sample(this->GetEnvironment()->GetBoundary(), samplerLabel);
  }
}

//delete same as drrt
std::pair<typename HASRRT::VID, bool>
HASRRT::
AddNode(const Cfg& _newCfg) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::AddNode");

  auto g = this->GetRoadmap();

  const VID lastVID = g->GetLastVID();
  const VID newVID  = g->AddVertex(_newCfg);

  const bool nodeIsNew = lastVID != g->GetLastVID();
  if(nodeIsNew) {
    if(this->m_debug)
      std::cout << "\tAdding VID " << newVID << "."
                << std::endl;
    

    // On each new sample, check if we need to advance our regions and generate
    // new ones. Add a roadmap hook to achieve this.
    auto vi = g->find_vertex(newVID);
    CheckRegionProximity(vi->property().GetPoint());
    AdvanceRegions(vi->property());
  }

  return {newVID, nodeIsNew};
}
/*---------------------------------- Helpers ---------------------------------*/

//delete same as drrt
Cfg
HASRRT::
Sample(SamplingRegion* _region) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SampleRegion");
  if (this->m_debug)
    std::cout << "\tSampling from region at "
              << "edge target vid " << _region->edgeIterator->target() << ", "
              << "edge index " << _region->edgeIndex << " and with center "
              << _region->GetCenter() << ", success rate so far "
              << _region->successes << " / " << _region->attempts
              << ", using sampler '" << this->m_samplerLabel << "'." << std::endl;

  // Get the boundary of the region.
  const auto center = _region->GetCenter();
  auto samplingBoundary = MakeBoundary(center);

  // Get the sampler.
  auto s = this->GetMPLibrary()->GetSampler(this->m_samplerLabel);

  std::vector<Cfg> samples, collision;
  while(samples.empty()) {
    s->Sample(1, 5, &samplingBoundary, std::back_inserter(samples),
      std::back_inserter(collision));

    // Increment successes as we sample.
    _region->TrackSuccess(samples.size(), samples.size() + collision.size());
  }
  auto target = samples.front();

  if(m_velocityBiasing)
    BiasVelocity(target, _region);

  if(this->m_debug)
    std::cout << "\t" << target.PrettyPrint() << std::endl;

  return target;
}

//delete same as drrt
Cfg
HASRRT::
Sample(const Boundary* const _boundary, const std::string* _samplerLabel) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SampleWholeEnv");
  // Get the sampler.
  auto s = this->GetMPLibrary()->GetSampler(*_samplerLabel);

  std::vector<Cfg> samples, collision;
  while(samples.empty())
    s->Sample(1, 5, _boundary, std::back_inserter(samples),
      std::back_inserter(collision));

  auto target = samples.front();
  if(this->m_debug)
    std::cout << "\t" << target.PrettyPrint() << std::endl;

  return target;
}

//delete same as drrt
const Vector3d
HASRRT::
GetVelocityBias(SamplingRegion* _region) {
  // Get the region data.
  // const auto& regionData = m_regionData.at(region);
  const size_t index = _region->edgeIndex;

  // Find the skeleton edge path the region is traversing.
  auto reit = _region->edgeIterator;
  const auto& path = reit->property();

  // Helper to make the biasing direction and print debug info.
  auto makeBias = [&](const Vector3d& _start, const Vector3d& _end) {
    if(this->m_debug)
      std::cout << "Computed velocity bias: " << (_end - _start).normalize()
                << "\n\tStart: " << _start
                << "\n\tEnd:   " << _end
                << std::endl;
    return (_end - _start).normalize();
  };

  // If there is at least one valid path point after the current path index,
  // then return the direction to the next point.
  if(index < path.size() - 1) {
    if(this->m_debug)
      std::cout << "Biasing velocity along next path step"
                << "\n\tPath index: " << index
                << "\n\tPath size:  " << path.size()
                << std::endl;
    return makeBias(path[index], path[index + 1]);
  }

  // Otherwise, the region has reached a skeleton vertex.
  WorkspaceSkeleton::VD targetVD = reit->target();
  auto vertex = m_skeleton.FindVertex(targetVD);

  // If the vertex has no outgoing edges, this is the end of the skeleton. In
  // that case, use the previous biasing direction. All paths have at least two
  // points so this is safe.
  if(vertex->size() == 0) {
    if(this->m_debug)
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
  if(this->m_debug)
    std::cout << "Biasing velocity along next edge (index " << nextEdgeIndex
              << ")\n\tPath index: " << index
              << "\n\tPath size:  " << path.size()
              << "\n\tNext edge path size: " << eit->property().size()
              << std::endl;
  return makeBias(path[index], eit->property()[1]);
}

//delete same as drrt
bool
HASRRT::
IsTouching(const Cfg& _cfg, SamplingRegion& _region) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::IsTouching");
  // Compute the penetration distance required. We want the robot's bounding
  // sphere to penetrate the region by the fraction m_penetrationThreshold.
  const double robotRadius  = _cfg.GetMultiBody()->GetBoundingSphereRadius(),
               threshold    = 2 * robotRadius * m_penetrationFactor;

  // Get the region boundary.
  const auto center = _region.GetCenter();
  auto boundary = MakeBoundary(center);

  // Compute the penetration distance (maximally enclosed bounding diameter).
  const Point3d robotCenter = _cfg.GetPoint();
  const double clearance = boundary.GetClearance(robotCenter);
  const double penetration = clearance + robotRadius;

  // The configuration is touching if the penetration exceeds the threshold.
  const bool touching = (penetration >= threshold);

  if(this->m_debug)
    std::cout << "\t Touch test: " << (touching ? "passed" : "failed")
              << "\n\t  Bounding sphere: " << robotCenter << " ; " << robotRadius
              << "\n\t  Region:          " << _region.GetCenter() << " ; "
              << m_regionRadius
              << "\n\t  Bounding sphere penetrates by "
              << std::setprecision(4)
              << penetration << (touching ? " >= " : " < ") << threshold
              << " units."
              << std::endl;

  return touching;
}


CSpaceBoundingSphere
HASRRT::
MakeBoundary(const Vector3d& _v) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::MakeBoundary");

  const bool threeD = this->GetTask()->GetRobot()->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;

  // I'm not sure what the boundary code might do with a negative radius. Bound
  // it below at zero just in case.
  const double radius = std::max(0., m_regionRadius);

  if (threeD)
    return CSpaceBoundingSphere({_v[0], _v[1], _v[2]}, radius);
  else
    return CSpaceBoundingSphere({_v[0], _v[1]}, radius);
}

/*--------------------------- Skeleton and Workspace -------------------------*/

void
HASRRT::
BuildSkeleton() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::BuildSkeleton");

  if(m_inputSkeleton != "")
    m_originalSkeleton.Read(m_inputSkeleton);

  else {

    // Determine if we need a 2d or 3d skeleton.
    auto env = this->GetEnvironment();
    auto robot = this->GetTask()->GetRobot();
    const bool threeD = robot->GetMultiBody()->GetBaseType()
      == Body::Type::Volumetric;

    if(threeD) {
      if(m_skeletonType == "mcs") {
        if(this->m_debug)
          std::cout << "Building a Mean Curvature skeleton." << std::endl;
        MeanCurvatureSkeleton3D mcs;
        mcs.SetEnvironment(this->GetEnvironment());
        mcs.BuildSkeleton();

        // Create the workspace skeleton.
        auto sk = mcs.GetSkeleton();
        m_originalSkeleton = sk.first;
        m_originalSkeleton.DoubleEdges();
      }
      else if(m_skeletonType == "reeb") {
        // Create a workspace skeleton using a reeb graph.
        if(this->m_debug)
          std::cout << "Building a Reeb Graph skeleton." << std::endl;
        auto decomposition = this->GetMPLibrary()->GetMPTools()->GetDecomposition(
            m_decompositionLabel);
        ReebGraphConstruction reeb;
        reeb.Construct(decomposition);

        // Create the workspace skeleton.
        m_originalSkeleton = reeb.GetSkeleton();
        m_originalSkeleton.DoubleEdges();
      }
      else
        throw ParseException(WHERE) << "Unrecognized skeleton type '"
          << m_skeletonType << "', options for 3d "
          << "problems are {mcs, reeb}.";
    }
    else {
      // Collect the obstacles we want to consider (all in this case).
      std::vector<GMSPolyhedron> polyhedra;
      for(size_t i = 0; i < env->NumObstacles(); ++i) {
        MultiBody* const obstacle = env->GetObstacle(i);
        for(size_t j = 0; j < obstacle->GetNumBodies(); ++j)
          polyhedra.emplace_back(obstacle->GetBody(j)->GetWorldPolyhedron());
      }

      // Build a skeleton from a 2D medial axis.
      if(this->m_debug)
        std::cout << "Build a skeleton from a 2D medial axis." << endl;
      MedialAxis2D ma(polyhedra, env->GetBoundary());
      ma.BuildMedialAxis();
      m_originalSkeleton = get<0>(ma.GetSkeleton(1)); // 1 for free space.
    }
  }

  if(m_directSkeleton) {
    if(this->m_debug)
      std::cout << "Direct input skeleton" << endl;
    m_originalSkeleton.DoubleEdges();
    DirectSkeleton();
    m_skeleton.Write("~/DIRECTEDSKELETON.graph");
  }
  else
    m_skeleton = m_originalSkeleton;

  if(m_refineEdges) {
    if(this->m_debug)
      std::cout << "Refine input skeleton" << endl;
    RefineEdges();
    m_skeleton.Write("~/RefinedSKELETON.graph");
  }

  if(m_outputSkeleton != "")
    m_skeleton.Write(m_outputSkeleton);
}


void
HASRRT::
MakeQuery() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::MakeQuery");
  // Only support single-goal tasks; this is inherent to the method. The problem"
  // is solvable but hasn't been solved yet.
  const auto& goalConstraints = this->GetTask()->GetGoalConstraints();
  if(goalConstraints.size() != 1)
    throw RunTimeException(WHERE) << "Only supports single-goal tasks. "
                                  << "Multi-step tasks will need new skeletons "
                                  << "for each sub-component.";

  // Find the workspace points which are nearest to the start and goal."
  auto g = this->GetRoadmap();
  auto goalTracker = this->GetMPLibrary()->GetGoalTracker();
  const auto& startVIDs = goalTracker->GetStartVIDs();
  const auto& goalVIDs  = goalTracker->GetGoalVIDs(0);
  Point3d start, goal;
  if(startVIDs.size() == 1) {
    const VID startVID = *startVIDs.begin();
    start = g->GetVertex(startVID).GetPoint();
  }
  else {
    // Probably we can just take the center of the start constraint boundary if
    // applicable, although we have no cases requiring that right now.
    throw RunTimeException(WHERE) << "Exactly one start VID is required, but "
                                  << startVIDs.size() << " were found.";
  }

  if(goalVIDs.size() == 1) {
    const VID goalVID = *goalVIDs.begin();
    goal = g->GetVertex(goalVID).GetPoint();
  }
  else {
    // Check for a goal boundary. We already checked that there is one goal
    // constraint, so it is safe to assume it exists here.
    const Boundary* const boundary = goalConstraints[0]->GetBoundary();
    if(!boundary)
      throw RunTimeException(WHERE) << "Exactly one goal VID is required, but "
                                    << goalVIDs.size() << " were found and no "
                                    << "constraint boundary was available.";

    // Try to sample a configuration in the boundary.
    auto sampler = this->GetMPLibrary()->GetSampler(this->m_samplerLabel);
    const size_t count    = 1,
                 attempts = 100;
    std::vector<Cfg> samples;
    sampler->Sample(count, attempts, boundary, std::back_inserter(samples));

    // If we couldn't generate a configuration here, the goal boundary isn't
    // realistic.
    if(samples.empty())
      throw RunTimeException(WHERE) << "Could not generate a sample within the "
                                    << "goal boundary " << *boundary
                                    << " after " << attempts << " attempts.";

    // We got a sample, take its point as the center point.
    goal = samples.front().GetPoint();
  }

  // If there is a new start and goal pair, redirect the skeleton
  std::pair<Point3d, Point3d> currentQuery{start, goal};

  if(this->m_debug)
    std::cout << "Start at " << start << " and goal at " << goal << std::endl;
  if(currentQuery == m_queryPair)
    return;
  m_queryPair = std::make_pair(start, goal);
}


void
HASRRT::
DirectSkeleton() {

  MakeQuery();

  // Direct the workspace skeleton outward from the starting point.
  if(this->m_debug)
    std::cout << "The skeleton has " << m_originalSkeleton.get_num_vertices() << " vertices" << std::endl;
  m_skeleton = m_originalSkeleton;
  m_skeleton = m_skeleton.Direct(m_queryPair.first);

  // Prune the workspace skeleton relative to the goal.
  m_skeleton.Prune(m_queryPair.second);

  if(this->m_debug)
    std::cout << "The pruned skeleton has " << m_skeleton.get_num_vertices() << " vertices" << std::endl;
}


const size_t
HASRRT::
SelectSamplingRegion() {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::SelectSamplingRegion");

  // Update all region probabilities.
  const std::vector<double> probabilities = ComputeProbabilities();


  // Select a region to sample from. The last region is the whole environment.
  double rand = DRand();
  double lowerBound = 0.0;
  int index;

  for(index = 0; index < (int)probabilities.size(); index++) {
    if((lowerBound < rand) and (rand < lowerBound + probabilities[index]))
      break;

    lowerBound += probabilities[index];
  }

  if(this->m_debug) {
    std::cout << "Computed region selection probabilities ("
              << "last is whole env):\n\t";

    for(auto p : probabilities)
      std::cout << std::setprecision(4) << p << " ";

    std::cout << "\n\tSelected index " << index
              << (index != (int)m_regions.size() ? "." : " (whole env).")
              << std::endl;
  }

  return index;
}


std::vector<double>
HASRRT::
ComputeProbabilities() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ComputeProbabilities");

  // Sum all weights of all current regions.
  double totalWeight = 0.;
  for(auto r : m_regions) {
    totalWeight += r.GetWeight();
  }

  // Compute the probabilities for the current regions.
  std::vector<double> probabilities;
  probabilities.reserve(m_regions.size() + 1);

  const double explore = m_explore / (m_regions.size() + 1);

  for(auto r : m_regions) {
    const double exploit = (1 - m_explore) * r.GetWeight() / totalWeight;

    probabilities.emplace_back(exploit + explore);
  }

  // Get the probability for the whole environment.
  probabilities.emplace_back(explore);

  return probabilities;
}


void
HASRRT::
BiasVelocity(Cfg& _cfg, SamplingRegion* _region) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::BiasVelocity");

  // Get the bias from the region kit.
  const Vector3d bias = GetVelocityBias(_region);
  if(bias.norm() == 0)
    throw RunTimeException(WHERE, "Bias cannot be zero.");

  // Resample the Cfg until its linear velocity aims relatively along the
  // biasing direction.
  Vector3d velocity;
  do {
    _cfg.GetRandomVelocity();
    velocity = _cfg.GetLinearVelocity().normalize();
    if(this->m_debug)
      std::cout << "\tSampled velocity direction: " << velocity
                << "\n\t\tDot product with bias: " << velocity * bias
                << (velocity * bias < m_velocityAlignment ? " < " : " >= ")
                << m_velocityAlignment
                << std::endl;
  } while(velocity * bias < m_velocityAlignment);
}


void
HASRRT::
CheckRegionProximity(const Point3d& _p) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::CheckRegionProximity");

  // Check each skeleton node to see if a new region should be created.
  for(auto iter = m_skeleton.begin(); iter != m_skeleton.end(); ++iter) {
    // Skip skeleton nodes that are too far away.
    const double dist = (iter->property() - _p).norm();
    if(dist >= m_regionRadius)
      continue;

    CreateRegions(iter);
  }
}


std::vector<typename HASRRT::SamplingRegion*>
HASRRT::
CreateRegions(const WorkspaceSkeleton::vertex_iterator _iter) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::CreateRegions");
  // Skip skeleton nodes that are already visited.
  if(m_visited[_iter->descriptor()])
    return {};

  m_visited[_iter->descriptor()] = true;

  // Save the set of created regions to return.
  std::vector<SamplingRegion*> newRegions;

  // Create a new region for each outgoing edge of this skeleton node.
  size_t count = 0;
  for(auto eit = _iter->begin(); eit != _iter->end(); ++eit) {
    m_regions.push_back(SamplingRegion(eit));
    count++;

    if(this->m_debug)
      std::cout << "Created new region with radius "
                << std::setprecision(4) << m_regionRadius
                << " on edge (" << eit->source() << ", "
                << eit->target() << ", " << eit->id() << ") "
                << "at skeleton vertex " << _iter->descriptor()
                << "(" << _iter->property() << ")."
                << "It has center " << m_regions.back().GetCenter()
                << std::endl;
  }


  for (size_t i = 1; i <= count; i++)
    newRegions.push_back(&m_regions[m_regions.size() - i]);

  return newRegions;
}


void
HASRRT::
AdvanceRegions(const Cfg& _cfg) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::AdvanceRegions");
  if(this->m_debug)
    std::cout << "Checking " << m_regions.size()
              << " regions for contact with new configuration "
              << _cfg.PrettyPrint() << "."
              << std::endl;

  // Keep track of any newly reached vertices to spawn regions on their outbound
  // edges.
  std::queue<WorkspaceSkeleton::VD> newlyReachedVertices;

  // Iterate through all existing regions to see which should be advanced.
  for(auto iter = m_regions.begin(); iter != m_regions.end(); ) {

    // Advance this region until the robot at _cfg is no longer touching it.
    if(!AdvanceRegionToCompletion(_cfg, *iter)) {
      ++iter;
      continue;
    }

    // We have reached the end of this region's edge. Delete it and save the
    // target vertex. to spawn new regions.
    auto eit = iter->edgeIterator;
    const auto target = eit->target();
    if(!m_visited[target]) {
      newlyReachedVertices.push(target);
      m_parentRegions.push_back(*iter);
    }
    iter = m_regions.erase(iter);
    if (m_selectedRegionIndex >= m_regions.size()){
      m_selectedRegionIndex = m_regions.size() - 1;
    }
  }

  // Create new regions for each newly reached vertex.
  while(!newlyReachedVertices.empty()) {
    // Pop the next vertex off the queue.
    WorkspaceSkeleton::VD targetVD = newlyReachedVertices.front();
    WorkspaceSkeleton::vertex_iterator target = m_skeleton.FindVertex(targetVD);
    newlyReachedVertices.pop();

    // Create regions at this vertex.
    std::vector<SamplingRegion*> newRegions = CreateRegions(target);

    // Advance each new region.
    /*for(auto region : newRegions) {
      auto iter = std::find(m_regions.begin(), m_regions.end(), *region);
      for(auto it2 = m_parentRegions.begin(); it2 != m_parentRegions.end(); it2++) {
        auto eit = it2->edgeIterator;
        if(eit->target() == targetVD) {
          iter->previousRegionIndex = distance(m_parentRegions.begin(), it2);
        }
      }
      // Advance this region until the robot at _cfg is no longer touching it.
      if(!AdvanceRegionToCompletion(_cfg, *region))
        continue;

      // We have reached the end of this region's edge. Delete it and save the
      // target vertex. to spawn new regions.
      //
      auto eit = region->edgeIterator;
      newlyReachedVertices.push(eit->target());
      m_parentRegions.push_back(*iter);
      m_regions.erase(iter);
    }*/
  }
}


bool
HASRRT::
AdvanceRegionToCompletion(const Cfg& _cfg, SamplingRegion& _region) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::AdvanceRegionToCompletion");
  // Find the edge path this region is traversing.
  auto eit = _region.edgeIterator;
  const std::vector<Point3d>& path = eit->property();
  size_t& i = _region.edgeIndex;

  if(this->m_debug)
    std::cout << "\tChecking region at "
              << _region.GetCenter() << "."
              << "\n\t Region is at index " << i << " / " << path.size() - 1
              << std::endl;

  if(IsTouching(_cfg, _region)) {

    if(this->m_debug)
      std::cout << "cfg is touching region" << std::endl;

    // If there are no more points left on this edge, this region is completed.
    if(_region.LastPoint()) {
      if(this->m_debug)
        std::cout << "\t Region has reached the end of its "
                  << "path, erasing it now. " << m_regions.size() - 1
                  << " regions remain."
                  << std::endl;

      return true;
    }

    // Otherwise there are still points left; advance the region and index.
    //_region.Advance();

    if(_region.edgeIndex < 1) {
      _region.Advance();
    }
    else
      _region.PushToEnd();
  }

  else {
    if(_region == m_regions[m_selectedRegionIndex]) {
      for(auto index = _region.edgeIndex; index > _region.frontLineIndex; index--) {
        auto tempRegion = m_regions[m_selectedRegionIndex];
        tempRegion.edgeIndex = index;
        if(IsTouching(_cfg, tempRegion)) {
          m_regions[m_selectedRegionIndex].frontLineIndex = tempRegion.edgeIndex;
          break;
        }
      }
      //auto newEdgeIdx = GetClosestIntermediate(_cfg, _region);
      auto newEdgeIdx = _region.edgeIndex < 1 ? (_region.edgeIndex + 1) : GetBinaryIntermediate(_cfg, _region);
      if(this->m_debug)
        std::cout << "\t Advancing region from index "
          << _region.edgeIndex << " to " << newEdgeIdx << " / " << path.size() - 1 << "."
          << std::endl;

      _region.edgeIndex = newEdgeIdx;
    }
  }

  if(this->m_debug)
    std::cout << "\t Region is still traversing this edge." << std::endl;

  return false;
}


size_t
HASRRT::
GetBinaryIntermediate(const Cfg& _cfg, SamplingRegion& _region, bool _isQnear) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::GetBinaryIntermediate");

  if(_isQnear) {
    size_t index =  (_region.edgeIndex > _region.frontLineIndex) ? (_region.edgeIndex - (_region.edgeIndex - _region.frontLineIndex) / 2)
                                                      : (_region.edgeIndex + (_region.frontLineIndex - _region.edgeIndex) / 2);
    if(this->m_debug) {
      std::cout << "current intermediate is at " << _region.edgeIndex << std::endl;
      std::cout << "frontline index: " << _region.frontLineIndex << std::endl;
      std::cout << "new index: " << index << std::endl;
    }
    return index;
  }

  auto closestEdgeIt = GetClosestIntermediate(_cfg, _region);

  size_t newIndex = _region.edgeIndex;

  if(_region.frontLineIndex < closestEdgeIt)
    newIndex =(_region.edgeIndex > closestEdgeIt) ? (_region.edgeIndex - (_region.edgeIndex - closestEdgeIt) / 2)
                                                      : (_region.edgeIndex + ( closestEdgeIt - _region.edgeIndex) / 2);
  else
    newIndex = (_region.edgeIndex > _region.frontLineIndex) ? (_region.edgeIndex - (_region.edgeIndex - _region.frontLineIndex) / 2)
                                                      : (_region.edgeIndex + (_region.frontLineIndex - _region.edgeIndex) / 2);

  if(this->m_debug) {
    std::cout << "current intermediate is at " << _region.edgeIndex << std::endl;
    std::cout << "closest intermediate is at " << closestEdgeIt << std::endl;
    std::cout << "frontline index: " << _region.frontLineIndex << std::endl;
    std::cout << "new index: " << newIndex << std::endl;
  }
  return newIndex;
}


size_t
HASRRT::
GetClosestIntermediate(const Cfg& _cfg, SamplingRegion& _region) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::GetClosestIntermediate");
  auto position = _cfg.GetPoint();
  auto eit = _region.edgeIterator;
  const std::vector<Point3d>& path = eit->property();
  size_t closestI = 0;
  double minDist = 1000.0;

  //size_t inter = (_region.edgeIndex < path.size() - 1) ? _region.edgeIndex : 1;
  size_t inter =  0;

  for(; inter <  path.size(); inter++) {
    auto dist = (path[inter] - position).norm();
    if(dist < minDist) {
      closestI = inter;
      minDist = dist;
    }
  }
  if(this->m_debug)
    std::cout << "closest intermediate is at " << closestI
              << " at distance " << minDist << std::endl;
  return closestI;
}


void
HASRRT::
RefineEdges() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::RefineEdges");

  if(this->m_debug)
    cout << "Skeleton size before refinement: " << m_skeleton.get_num_vertices() << endl;
  std::vector<std::vector<Point3d>> refinedVertices;
  vector<pair<size_t, size_t> > toDelete;
  vector<vector<Point3d> > toAdd;

  //auto boundary = this->GetEnvironment()->GetBoundary();
  auto vc = this->GetMPLibrary()->GetValidityChecker("pqp_solid");

  auto pointRobot = this->GetMPProblem()->GetRobot("point");

  auto boundary = this->GetEnvironment()->GetBoundary();

  // Function to compute clearance for input point _p.
  auto getClearance = [&](const Point3d& _p) -> double {
    // Check against obstacles using a point robot.
    Cfg cfg(_p, pointRobot);
    CDInfo cdInfo(true);
    vc->IsValid(cfg, cdInfo, "Skeleton ray Clearance");

    const double boundaryClearance = boundary->GetClearance(_p);
    if(boundaryClearance < cdInfo.m_minDist) {
      cdInfo.m_objectPoint = boundary->GetClearancePoint(_p);
      cdInfo.m_minDist = boundaryClearance;
    }
    // Return the minimum clearance.
    return cdInfo.m_minDist;
  };

  bool tobedeleted = false;

  for(auto ei = m_skeleton.edges_begin(); ei != m_skeleton.edges_end(); ei++) {

    std::vector<Point3d> newIntermediates;

    auto& intermediates = ei->property();

    std::cout<<"(" << ei->source() <<", "
      << ei->target() << "):"
      << intermediates.size() << std::flush << std::endl;

    if(intermediates.size() < 20)
      continue;

    Point3d direction = intermediates[1] - intermediates[0];

    size_t startIndex = 0;

    while(true) {
      auto endPoint = intermediates[startIndex] + direction;
      double clearance = getClearance(endPoint);
      if(clearance <= 0) {
        double minDist = 100;
        size_t closestPointId = 0;
        for(size_t i = startIndex; i < intermediates.size(); i++) {
          double distance = (endPoint - intermediates[i]).norm();
          if(distance < minDist) {
            minDist = distance;
            closestPointId = i;
          }
        }
        for(size_t i = startIndex; i <= closestPointId; i++) {
          newIntermediates.push_back(intermediates[i]);
        }

        if(newIntermediates.size() < ei->property().size()) {
          toAdd.push_back(newIntermediates);
          newIntermediates.clear();
          tobedeleted = true;
        }

        if(closestPointId == (intermediates.size() - 1))
          break;

        startIndex = closestPointId;
        direction = intermediates[startIndex + 1] - intermediates[startIndex];
      }

      direction += (direction * 0.5);
    }
    if(tobedeleted)
      toDelete.push_back(make_pair(ei->source(), ei->target()));

    tobedeleted = false;
  }

  size_t vd1 = m_skeleton.AddVertex(toAdd.front().front());
  for(auto edge : toAdd) {
    if(edge.size()) {
      if((m_skeleton.GetVertex(vd1) - edge.front()).norm() >= 0.1)
        vd1 = m_skeleton.AddVertex(edge.front());
      auto vd2 = m_skeleton.AddVertex(edge.back());
      m_skeleton.AddEdge(vd1, vd2, edge);
      vd1 = vd2;
    }
  }

  for(auto edge : toDelete) {
    (&m_skeleton)->DeleteEdge(edge.first, edge.second);
  }

  if(this->m_debug)
    cout << "Skeleton size after removing edges: " << m_skeleton.get_num_vertices() << endl;
}
