#include "BasicRRTStrategy.h"

#include "MPLibrary/MPLibrary.h"
#include "MPProblem/Constraints/Constraint.h"

#include <iomanip>
#include <iterator>
#include <string>
#include <unordered_set>
#include <vector>

/*----------------------------- Construction ---------------------------------*/

BasicRRTStrategy::
BasicRRTStrategy() {
  this->SetName("BasicRRTStrategy");
}


BasicRRTStrategy::
BasicRRTStrategy(XMLNode& _node) : MPStrategyMethod(_node) {
  this->SetName("BasicRRTStrategy");

  // Parse RRT parameters
  m_growGoals = _node.Read("growGoals", false, m_growGoals,
      "Grow a tree each goal in addition to the start?");

  m_growthFocus = _node.Read("growthFocus", false,
      m_growthFocus, 0.0, 1.0,
      "Fraction of goal-biased iterations");
  m_numDirections = _node.Read("m", false,
      m_numDirections, size_t(1), size_t(1000),
      "Number of directions to extend");
  m_disperseTrials = _node.Read("trial", m_numDirections > 1,
      m_disperseTrials, size_t(1), size_t(1000),
      "Number of trials to get a dispersed direction");

  // Parse MP object labels
  m_samplerLabel = _node.Read("samplerLabel", true, "", "Sampler Label");
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_exLabel = _node.Read("extenderLabel", true, "", "Extender label");
  m_ncLabel = _node.Read("connectorLabel", false, "",
      "Connection Method for RRG-like behavior.");
  m_fallbackNfLabel = _node.Read("fallbackNfLabel", false, "",
      "Fall back NF in case the main one fails.");

  m_goalDmLabel = _node.Read("goalDmLabel", false, "",
      "Distance metric for checking goal extensions in uni-directional RRT.");
  m_goalThreshold = _node.Read("goalThreshold", false,
      m_goalThreshold, 0., std::numeric_limits<double>::max(),
      "For each extension that ends within this distance of the goal (according "
      "to the goal DM), attempt to extend towards the goal. If the value is 0 or"
      " not set, the extender's max range will be used instead.");


  // Some options only apply when growing goals
  if(m_growGoals) {
    const std::string when(" with bi-directional growth.");

    if(!m_goalDmLabel.empty())
      throw ParseException(_node.Where()) << "Cannot use goal DM" << when;
    if(m_goalThreshold)
      throw ParseException(_node.Where()) << "Cannot use goal threshold" << when;
    if(m_growthFocus)
      throw ParseException(_node.Where()) << "Cannot use growth focus" << when;
  }
}

BasicRRTStrategy::
~BasicRRTStrategy() { }

/*------------------------- MPBaseObject Overrides ---------------------------*/

void
BasicRRTStrategy::
Print(std::ostream& _os) const {
  MPStrategyMethod::Print(_os);
  _os << "\tSampler: " << m_samplerLabel
      << "\n\tNeighborhood Finder: " << m_nfLabel
      << "\n\tExtender: " << m_exLabel
      << "\n\tConnection Method: " << m_ncLabel
      << "\n\tGoal check DM: " << m_goalDmLabel
      << "\n\tGrow Goals: " << m_growGoals
      << "\n\tGrowth Focus: " << m_growthFocus
      << "\n\tExpansion directions / trials: " << m_numDirections
      << " / " << m_disperseTrials
      << std::endl;
}

/*-------------------------- MPStrategy overrides ----------------------------*/

void
BasicRRTStrategy::
Initialize() {
  // Sanity checks on grow goals option.
  if(m_growGoals) {
    // Assert that we are not using a nonholonomic robot.
    if(this->GetTask()->GetRobot()->IsNonholonomic())
      throw RunTimeException(WHERE) << "Bi-directional growth with nonholonomic "
                                    << "robots is not supported (requires a "
                                    << "steering function).";

    // Assert that we are not using a rewiring connector.
    const bool rewiring = !m_ncLabel.empty()
                      and this->GetMPLibrary()->GetConnector(m_ncLabel)->IsRewiring();
    if(rewiring)
      throw RunTimeException(WHERE) << "Bi-directional growth is not supported "
                                    << "with rewiring connectors (rewiring "
                                    << "connectors need to follow the parent "
                                    << "trail which doesn't make sense for a "
                                    << "non-tree roadmap.";
  }

  // Try to generate a start configuration if we have start constraints.
  const VID start = this->GenerateStart(m_samplerLabel);
  const bool noStart = start == INVALID_VID;

  // If we are growing goals, try to generate goal configurations.
  if(m_growGoals)
    this->GenerateGoals(m_samplerLabel);

  // If we have neither start nor goals, generate a random root. Try up to 100
  // times.
  if(noStart and !m_growGoals)
  {
    // Determine which sampler to use.
    const auto& samplerLabel = this->m_querySampler.empty()
                             ? m_samplerLabel
                             : this->m_querySampler;

    auto s = this->GetMPLibrary()->GetSampler(m_samplerLabel);

    std::vector<Cfg> samples;
    s->Sample(1, 100, this->GetEnvironment()->GetBoundary(),
        std::back_inserter(samples));

    // If we made no samples, throw an error.
    if(samples.empty())
      throw RunTimeException(WHERE) << "Failed to generate a random root with "
                                    << "sampler '" << samplerLabel << "'.";

    // Add the configuration to the graph.
    auto g = this->GetRoadmap();
    const auto& cfg = samples[0];
    g->AddVertex(cfg);
  }
}


void
BasicRRTStrategy::
Iterate() {
  // Find growth target.
  const Cfg target = this->SelectTarget();

  // Expand the tree from nearest neigbor to target.
  const VID newVID = this->ExpandTree(target);
  if(newVID == INVALID_VID)
    return;

  // If growing goals, try to connect other trees to the new node. Otherwise
  // check for a goal extension.
  if(m_growGoals)
    ConnectTrees(newVID);
  else
    TryGoalExtension(newVID);
}

/*--------------------------- Direction Helpers ------------------------------*/

Cfg
BasicRRTStrategy::
SelectTarget() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SelectTarget");

  Cfg target(this->GetTask()->GetRobot());

  // Get the sampler and boundary.
  const Boundary* samplingBoundary = this->GetEnvironment()->GetBoundary();
  const std::string* samplerLabel = &m_samplerLabel;

  // Select goal growth with probability m_growthFocus.
  auto goalTracker = this->GetMPLibrary()->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();

  if(unreachedGoals.size() and DRand() < m_growthFocus) {
    // Randomly select a goal constraint boundary.
    const auto& goalConstraints = this->GetTask()->GetGoalConstraints();
    const size_t index = unreachedGoals[LRand() % unreachedGoals.size()];
    const Boundary* const b = goalConstraints[index]->GetBoundary();

    // We may eventually support constraints that cannot be described in terms
    // of a boundary, but that is outside the scope of the present
    // implementation.
    if(!b)
      throw NotImplementedException(WHERE) << "Non-boundary constraints are not "
                                           << "yet supported.";

    // If there is a query sampler, use that for goal sampling.
    if(!this->m_querySampler.empty())
      samplerLabel = &this->m_querySampler;

    if(this->m_debug)
      std::cout << "Sampling growth target from goal " << index
                << " (sampler '" << *samplerLabel << "'):"
                << std::endl;
  }
  // Otherwise, use the designated sampler with the environment boundary.
  else if(this->m_debug)
    std::cout << "Random growth target selected (sampler '" << *samplerLabel
              << "'):" << std::endl;

  std::vector<Cfg> samples;
  auto s = this->GetMPLibrary()->GetSampler(*samplerLabel);
  while(samples.empty())
    s->Sample(1, 100, samplingBoundary, std::back_inserter(samples));
  target = samples.front();

  if(this->m_debug)
    std::cout << "\t" << target.PrettyPrint() << std::endl;

  return target;
}


Cfg
BasicRRTStrategy::
SelectDispersedTarget(const VID _v) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SelectDispersedTarget");

  // Get original cfg with vid _v and its neighbors
  auto g = this->GetRoadmap();
  const std::vector<VID> neighbors = g->GetChildren(_v);
  const Cfg& originalCfg = g->GetVertex(_v);

  // Look for the best extension direction, which is the direction with the
  // largest angular separation from any neighbor.
  Cfg bestCfg(this->GetTask()->GetRobot());
  double bestAngle = -MAX_DBL;
  for(size_t i = 0; i < m_disperseTrials; ++i) {
    // Get a random configuration
    Cfg randCfg(this->GetTask()->GetRobot());
    randCfg.GetRandomCfg(this->GetEnvironment());

    // Get the unit direction toward randCfg
    Cfg randDir = randCfg - originalCfg;
    randDir /= randDir.Magnitude();

    // Calculate the minimum angular separation between randDir and the
    // unit directions to originalCfg's neighbors
    double minAngle = MAX_DBL;
    for(auto& vid : neighbors) {
      const Cfg& neighbor = g->GetVertex(vid);

      // Get the unit direction toward neighbor
      Cfg neighborDir = neighbor - originalCfg;
      neighborDir /= neighborDir.Magnitude();

      // Compute the angle between randDir and neighborDir
      double sum{0};
      for(size_t j = 0; j < originalCfg.DOF(); ++j)
        sum += randDir[j] * neighborDir[j];
      const double angle = std::acos(sum);

      // Update minimum angle
      minAngle = std::min(minAngle, angle);
    }

    // Now minAngle is the smallest angle between randDir and any neighborDir.
    // Keep the randDir that produces the largest minAngle.
    if(bestAngle < minAngle) {
      bestAngle = minAngle;
      bestCfg = randCfg;
    }
  }

  return bestCfg;
}

/*---------------------------- Neighbor Helpers ------------------------------*/

typename BasicRRTStrategy::VID
BasicRRTStrategy::
FindNearestNeighbor(const Cfg& _cfg, const VertexSet* const _candidates) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::FindNearestNeighbor");

  if(this->m_debug)
    std::cout << "Searching for nearest neighbors to " << _cfg.PrettyPrint()
              << " with '" << m_nfLabel << "' from "
              << (_candidates
                  ? "a set of size " + std::to_string(_candidates->size())
                  : "the full roadmap")
              << "."
              << std::endl;

  // Search for the nearest neighbors according to the NF.
  std::vector<Neighbor> neighbors;
  auto g = this->GetRoadmap();
  auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(m_nfLabel);
  if(_candidates)
    nf->FindNeighbors(g, _cfg, *_candidates, std::back_inserter(neighbors));
  else
    nf->FindNeighbors(g, _cfg, std::back_inserter(neighbors));

  // If we found no neighbors, try the fallback NF if we have one.
  if(neighbors.empty() and !m_fallbackNfLabel.empty()) {
    auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(m_fallbackNfLabel);
    if(_candidates)
      nf->FindNeighbors(g, _cfg, *_candidates, std::back_inserter(neighbors));
    else
      nf->FindNeighbors(g, _cfg, std::back_inserter(neighbors));
  }

  // Check for no neighbors. We really don't want this to happen - if you see
  // high numbers for this, you likely have problems with parameter or algorithm
  // selection.
  if(neighbors.empty()) {
    stats->IncStat(this->GetNameAndLabel() + "::FailedNF");
    if(this->m_debug)
      std::cout << "\tFailed to find a nearest neighbor."
                << std::endl;
    return INVALID_VID;
  }

  const Neighbor best = SelectNeighbor(_cfg, neighbors);

  if(this->m_debug) {
    const Neighbor& nearest = neighbors[0];
    std::cout << "\tFound " << neighbors.size() << " candidate neighbors."
              << std::endl
              << "\tNearest is VID " << nearest.target << " at distance "
              << std::setprecision(4) << nearest.distance << "."
              << std::endl
              << "\tBest is VID " << best.target << " at distance "
              << std::setprecision(4) << best.distance << "."
              << std::endl;
  }

  return best.target;
}


Neighbor
BasicRRTStrategy::
SelectNeighbor(const Cfg& _cfg, const std::vector<Neighbor>& _neighbors) {
  // Return the nearest (first).
  return _neighbors[0];
}

/*----------------------------- Growth Helpers -------------------------------*/

typename BasicRRTStrategy::VID
BasicRRTStrategy::
Extend(const VID _nearVID, const Cfg& _target, LPOutput& _lp,
    const bool _requireNew) {
  auto stats = this->GetStatClass();
  const std::string id = this->GetNameAndLabel() + "::Extend";
  MethodTimer mt(stats, id);
  stats->IncStat(id);

  auto e = this->GetMPLibrary()->GetExtender(m_exLabel);
  const Cfg& qNear = this->GetRoadmap()->GetVertex(_nearVID);
  Cfg qNew(this->GetTask()->GetRobot());

  const bool success = e->Extend(qNear, _target, qNew, _lp);
  if(this->m_debug)
    std::cout << "Extending from VID " << _nearVID
              << "\n\tqNear: " << qNear.PrettyPrint()
              << "\n\tExtended "
              << std::setprecision(4) << _lp.m_edge.first.GetWeight()
              << " units."
              << std::endl;

  if(!success) {
    // The extension failed to exceed the minimum distance.
    if(this->m_debug)
      std::cout << "\tNode too close, not adding." << std::endl;
    return INVALID_VID;
  }

  // The extension succeeded. Try to add the node.
  const auto extension = AddNode(qNew);

  const VID& newVID = extension.first;
  const bool nodeIsNew = extension.second;
  if(!nodeIsNew) {
    // The extension reproduced an existing node.
    if(_requireNew) {
      if(this->m_debug)
        std::cout << "\tNode already exists (" << newVID
                  << "), not adding." << std::endl;
      return INVALID_VID;
    }
    else if(this->m_debug)
      std::cout << "\tConnected to existing node " << newVID << "."
                << std::endl;
  }

  // The node was ok. Add the edge.
  AddEdge(_nearVID, newVID, _lp);

  return newVID;
}


typename BasicRRTStrategy::VID
BasicRRTStrategy::
Extend(const VID _nearVID, const Cfg& _target, const bool _requireNew) {
  LPOutput dummyLP;
  return this->Extend(_nearVID, _target, dummyLP, _requireNew);
}


std::pair<typename BasicRRTStrategy::VID, bool>
BasicRRTStrategy::
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
  }

  return {newVID, nodeIsNew};
}


void
BasicRRTStrategy::
AddEdge(const VID _source, const VID _target,
    const LPOutput& _lpOutput) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::AddEdge");

  if(this->m_debug)
    std::cout << "\tAdding Edge (" << _source << ", " << _target << ")."
              << std::endl;

  // Add the edge.
  auto g = this->GetRoadmap();

  // If we are growing goals, we need to add bi-directional edges for the query
  // to work (otherwise the trees join at fruitless junctions with no strong
  // connectivity between them). This also applies if we are using a
  // non-rewiring connector as in with RRG.
  const bool biDirectionalEdges = m_growGoals or
      (!m_ncLabel.empty() and !this->GetMPLibrary()->GetConnector(m_ncLabel)->IsRewiring());
  if(biDirectionalEdges)
    g->AddEdge(_source, _target, _lpOutput.m_edge);
  // Use a one-way edge for uni-directional RRT because superfluous back edges
  // serve no useful purpose and increase query time. This is also required for
  // problems with one-way extenders such as KinodynamicExtender.
  else
    g->AddEdge(_source, _target, _lpOutput.m_edge.first);
}


void
BasicRRTStrategy::
ConnectNeighbors(const VID _newVID) {
  // Make sure _newVID is valid and we have a connector.
  if(_newVID == INVALID_VID or m_ncLabel.empty())
    return;

  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ConnectNeighbors");

  // Try to connect _newVID to its neighbors using the connector.
  this->GetMPLibrary()->GetConnector(m_ncLabel)->Connect(this->GetRoadmap(), _newVID);
}


void
BasicRRTStrategy::
TryGoalExtension(const VID _newVID) {
  // Make sure _newVID is valid.
  if(m_growGoals or m_goalDmLabel.empty() or _newVID == INVALID_VID)
    return;

  // Make sure we have goals.
  const auto& goalConstraints = this->GetTask()->GetGoalConstraints();
  if(goalConstraints.empty())
    return;

  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::TryGoalExtension");

  if(this->m_debug)
    std::cout << "Checking goal extension for new node " << _newVID << " at "
              << this->GetRoadmap()->GetVertex(_newVID).PrettyPrint()
              << std::endl;

  for(const auto& constraint : goalConstraints)
    TryGoalExtension(_newVID, constraint->GetBoundary());
}


void
BasicRRTStrategy::
TryGoalExtension(const VID _newVID, const Boundary* const _boundary) {
  if(!_boundary)
    throw RunTimeException(WHERE) << "Constraints which do not produce a "
                                  << "boundary are not supported.";

  // First check if _newVID is already in the goal region.
  auto g = this->GetRoadmap();
  const Cfg& cfg = g->GetVertex(_newVID);
  const bool inGoal = _boundary->InBoundary(cfg);
  if(inGoal) {
    if(this->m_debug)
      std::cout << "\tNode is already in this goal boundary." << std::endl;
    return;
  }

  // Get the nearest point to _newVID within this goal region.
  std::vector<double> data = cfg.GetData();
  _boundary->PushInside(data);
  Cfg target(cfg.GetRobot());
  target.SetData(data);

  // Check the nearest point to _newVID in each goal region. If it lies within
  // the goal threshold, try to extend towards it.
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_goalDmLabel);
  const double distance = dm->Distance(cfg, target),
               range = m_goalThreshold == 0.
                     ? this->GetMPLibrary()->GetExtender(m_exLabel)->GetMaxDistance()
                     : m_goalThreshold;


  if(this->m_debug)
    std::cout << "\tNearest goal configuration is " << distance << " / "
              << range << " units away at " << target.PrettyPrint()
              << "."
              << std::endl;

  // If we are out of range, do not attempt to extend.
  if(distance > range) {
    if(this->m_debug)
      std::cout << "\tNot attempting goal extension." << std::endl;
    return;
  }

  // Try to extend towards the target.
  const VID extended = this->Extend(_newVID, target);
  if(extended == INVALID_VID)
    return;

  // Check if we reached the goal boundary.
  const Cfg& extendedCfg = g->GetVertex(extended);
  const bool reached = _boundary->InBoundary(extendedCfg);
  if(reached) {
    if(this->m_debug)
      std::cout << "\tExtension reached goal boundary." << std::endl;
    return;
  }

  // Some extenders use a variable distance, so retry if we got part-way there.
  // Note that the original Cfg reference may have been invalidated by the graph
  // expanding!
  const double extendedDistance = dm->Distance(g->GetVertex(_newVID),
      extendedCfg);
  if(extendedDistance < distance) {
    if(this->m_debug)
      std::cout << "\tExtension made progress but did not reach goal, retrying."
                << std::endl;
    TryGoalExtension(extended, _boundary);
  }
  else if(this->m_debug)
    std::cout << "\tExtension did not make progress." << std::endl;
}

/*------------------------------ Tree Helpers --------------------------------*/

typename BasicRRTStrategy::VID
BasicRRTStrategy::
ExpandTree(const Cfg& _target) {
  const VID nearestVID = FindNearestNeighbor(_target);
  if(nearestVID == INVALID_VID)
    return INVALID_VID;

  return this->ExpandTree(nearestVID, _target);
}


typename BasicRRTStrategy::VID
BasicRRTStrategy::
ExpandTree(const VID _nearestVID, const Cfg& _target) {
  if(this->m_debug)
    std::cout << "Trying expansion from node " << _nearestVID << " "
         << this->GetRoadmap()->GetVertex(_nearestVID).PrettyPrint()
         << std::endl;

  // Try to extend from the _nearestVID to _target
  const VID newVID = this->Extend(_nearestVID, _target);
  if(newVID == INVALID_VID)
    return INVALID_VID;

  // Connect neighbors if we are using a connector.
  ConnectNeighbors(newVID);

  // Expand to other directions
  for(size_t i = 1; i < m_numDirections; ++i) {
    if(this->m_debug)
      std::cout << "Expanding to other directions (" << i << "/"
                << m_numDirections - 1 << "):: ";

    // Select a dispersed target and expand towards it.
    const Cfg randCfg = SelectDispersedTarget(_nearestVID);
    const VID additionalNewVID = this->Extend(_nearestVID, randCfg);

    // If we failed, move on to the next attempt.
    if(additionalNewVID == INVALID_VID)
      continue;

    // Connect neighbors if we are using a connector.
    ConnectNeighbors(additionalNewVID);
  }

  return newVID;
}


void
BasicRRTStrategy::
ConnectTrees(const VID _recentlyGrown) {
  auto g = this->GetRoadmap();
  auto ccTracker = g->GetCCTracker();
  const size_t ccCount = ccTracker->GetNumCCs();

  if(!m_growGoals or _recentlyGrown == INVALID_VID or ccCount == 1)
    return;

  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ConnectTrees");

  // Get the configuration by value in case the graph's vertex vector gets
  // re-allocated.
  const Cfg qNew = g->GetVertex(_recentlyGrown);

  if(this->m_debug)
    std::cout << "Trying to connect " << ccCount - 1 << " other trees "
              << "to node " << _recentlyGrown << "."
              << std::endl;

  // Try to connect qNew to each other CC.
  VertexSet representatives = ccTracker->GetRepresentatives();
  while(representatives.size())
  {
    // Get the next representative and remove it from the set.
    const VID representative = *representatives.begin();
    representatives.erase(representatives.begin());

    // If the new vertex and this are in the same CC, there is nothing to do.
    const bool sameCC = ccTracker->InSameCC(_recentlyGrown, representative);
    if(sameCC)
      continue;

    // Get the CC associated with this representative.
    const VertexSet* const cc = ccTracker->GetCC(representative);

    // Find nearest neighbor to qNew in the other tree.
    const VID nearestVID = FindNearestNeighbor(qNew, cc);
    if(nearestVID == INVALID_VID)
      continue;

    // Try to extend from the other tree to qNew.
    this->Extend(nearestVID, qNew, false);
  }
}

/*----------------------------------------------------------------------------*/
