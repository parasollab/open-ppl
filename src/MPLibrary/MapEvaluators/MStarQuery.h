#ifndef PPL_M_STAR_QUERY_H_
#define PPL_M_STAR_QUERY_H_

#include "MapEvaluatorMethod.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "Utilities/SSSP.h"

////////////////////////////////////////////////////////////////////////////////
/// Follows the optimal policy for each individual robot until collisions
/// are encountered. Then, considers more of the composite space as needed.
///
/// Reference:
///   G. Wagner and H. Choset, "M*: A complete multirobot path planning 
///   algorithm with performance bounds," 2011 IEEE/RSJ International 
///   Conference on Intelligent Robots and Systems, San Francisco, CA, USA, 
///   2011, pp. 3260-3267, doi: 10.1109/IROS.2011.6095022.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class MStarQuery : public MapEvaluatorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType   RoadmapType;
    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::Path          Path;
    typedef typename RoadmapType::VID        VID;
    typedef typename RoadmapType::EI         EI;

    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::GroupWeightType  GroupWeightType;

    ///}
    ///@name Internal Types
    ///@{

    struct Node {

      VID vid{INVALID_VID};
      double cost{MAX_DBL};
      double h{0.0};
      Node* backPtr{nullptr};
      std::unordered_set<Node*> backSet;

      Node() {}

      Node(VID _vid, double _cost, double _h, Node* _backPtr=nullptr) : 
        vid(_vid), cost(_cost), h(_h), backPtr(_backPtr) {}

      bool operator>(const Node& _n) const noexcept {
        return cost + h > _n.cost + _n.h;
      }
    };

    struct NodeComp {
      bool operator()(const Node* _n, const Node* _m) const  {
        return (*_n) > (*_m);
      }
    };

    ///@}
    ///@name Construction
    ///@{

    MStarQuery();

    MStarQuery(XMLNode& _node);

    virtual ~MStarQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    virtual bool operator()() override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    void GenerateOptimalPolicies();

    double CompositeCostToGo(GroupCfgType& _gcfg);

    bool IsGoal(GroupCfgType& _gcfg);

    void SetPath(Node* _goalNode);

    std::unordered_set<Robot*> GetCollisions(GroupCfgType& _gcfg);

    std::unordered_set<Node*> GetLimitedNeighbors(Node* _node, 
        std::set<Node*, NodeComp>& _openSet);

    void BackpropCollisions(Node* _current, Node* _neighbor, 
        std::set<Node*, NodeComp>& _openSet);

    std::unordered_set<Robot*> ValidateEdge(VID _source, VID _target);

    ///@}
    ///@name Internal State
    ///@{
  
    std::string m_vcLabel{""};

    std::string m_connLabel{""};

    VID m_compositeStart{INVALID_VID};

    std::unordered_map<Robot*, VID> m_goalMap;

    std::unordered_map<Robot*, std::unordered_map<VID, double>> m_costToGoMap;

    std::map<VID, Node> m_nodeMap;

    std::map<Node*, std::unordered_set<Robot*>> m_collisionSets;

    std::unordered_map<std::pair<VID, VID>, std::unordered_set<Robot*>> m_failedEdges;

    ///@}

};

template <typename MPTraits>
MStarQuery<MPTraits>::
MStarQuery() {
  this->SetName("MStarQuery");
}


template <typename MPTraits>
MStarQuery<MPTraits>::
MStarQuery(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("MStarQuery");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "The validity checker for conflict detection. Must be a CD type.");
  
  m_connLabel = _node.Read("connLabel", true, "",
      "The connector to use to connect composite configurations.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MStarQuery<MPTraits>::
Initialize() {
  auto vc = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(
      this->GetValidityChecker(m_vcLabel)
  );
  if(!vc)
    throw RunTimeException(WHERE) << "Validity checker " << m_vcLabel
                                  << " is not of type "
                                  << "CollisionDetectionValidityMethod.";

  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::FoundPath", 0);
}

template <typename MPTraits>
bool
MStarQuery<MPTraits>::
operator()() {
  m_nodeMap.clear();
  m_collisionSets.clear();

  if(this->m_debug)
    std::cout << "Starting M*..." << std::endl;

  // Construct the initial vertex in the group roadmap
  auto goalTracker = this->GetGoalTracker();
  auto initialGcfg = GroupCfgType(this->GetGroupRoadmap());
  for(auto& task : *(this->GetGroupTask())) {
    auto robot = task.GetRobot();
    auto groupIdx = this->GetGroupRoadmap()->GetGroup()->GetGroupIndex(robot);
    auto roadmap = this->GetGroupRoadmap()->GetIndividualGraph(groupIdx);

    // Get the start VID
    auto startVIDs = goalTracker->GetStartVIDs(roadmap, &task);
    if(startVIDs.size() < 1) {
      if(this->m_debug)
        std::cout << "No start VIDs for robot " << robot->GetLabel() << std::endl;
      return false;
    }
    auto start = *startVIDs.begin();
    initialGcfg.SetRobotCfg(robot, start);

    // Make sure there is at least one goal VID
    auto goalVIDs = goalTracker->GetGoalVIDs(roadmap, &task, 0);
    if(goalVIDs.size() < 1) {
      if(this->m_debug)
        std::cout << "No goal VIDs for robot " << robot->GetLabel() << std::endl;
      return false;
    }
    m_goalMap[robot] = *goalVIDs.begin();
  }
  auto startVID = m_compositeStart;
  if(startVID == INVALID_VID) {
    startVID = this->GetGroupRoadmap()->AddVertex(initialGcfg);
    m_compositeStart = startVID;
  }

  GenerateOptimalPolicies();

  // Make the initial node
  std::set<Node*, NodeComp> openSet;
  auto heur = CompositeCostToGo(initialGcfg);
  m_nodeMap.emplace(std::make_pair(startVID, Node(startVID, 0, heur)));
  openSet.insert(&m_nodeMap[startVID]);

  if(this->m_debug)
    std::cout << "Using group start vid " << startVID
              << " with heuristic " << heur << std::endl;

  while(!openSet.empty()) {
    auto current = *openSet.begin();
    openSet.erase(openSet.begin());
    auto vid = current->vid;
    auto gcfg = this->GetGroupRoadmap()->GetVertex(vid);

    if(this->m_debug)
      std::cout << "Expanding group roadmap vid " << vid << std::endl;

    // Check if we've reached the goal
    if(IsGoal(gcfg)) {
      if(this->m_debug)
        std::cout << "Found the goal at vid " << current->vid << std::endl;

      SetPath(current);
      return true;
    }

    // Check if this gcfg is in collision
    auto collisions = GetCollisions(gcfg);
    if(collisions.size() > 0) {
      if(this->m_debug)
        std::cout << "Found " << collisions.size() 
                  << " collisions, continuing." << std::endl;

      continue;
    }

    // Backpropagate collisions from the neighbors
    auto limitedNeighbors = GetLimitedNeighbors(current, openSet);
    for(auto neighbor : limitedNeighbors) {
      neighbor->backSet.insert(current);

      auto neighborGcfg = this->GetGroupRoadmap()->GetVertex(neighbor->vid);
      auto collided = GetCollisions(neighborGcfg);
      m_collisionSets[neighbor].insert(collided.begin(), collided.end());

      BackpropCollisions(current, neighbor, openSet);

      auto edge = this->GetGroupRoadmap()->GetEdge(current->vid, neighbor->vid);
      if(current->cost + edge.GetWeight() < neighbor->cost) {
        // We have found a lower cost path to the neighbor
        if(this->m_debug)
          std::cout << "\t\tFound better path to vid " << neighbor->vid << std::endl;

        openSet.erase(neighbor);
        neighbor->cost = current->cost + edge.GetWeight();
        neighbor->backPtr = current;
        openSet.insert(neighbor); // Reinsert to ensure proper ordering
      }
    }
  }

  if(this->m_debug)
    std::cout << "Exhausted search and failed to find solution." << std::endl;

  return false; // return false if no solution found
}


template <typename MPTraits>
void
MStarQuery<MPTraits>::
GenerateOptimalPolicies() {
  m_costToGoMap.clear();

  if(this->m_debug)
    std::cout << "Running Dijkstras to get optimal policies." << std::endl;

  // Use Dijkstra's to get the cost to go from each vertex in each roadmap
  SSSPTerminationCriterion<RoadmapType> termination(
    [](typename RoadmapType::VI& _vi, const SSSPOutput<RoadmapType>& _sssp) {
      return SSSPTermination::Continue;
    }
  );

  SSSPPathWeightFunction<RoadmapType> weight = [&](
    typename RoadmapType::EI& _ei, const double _sourceDistance, 
    const double _targetDistance) {

    return _sourceDistance + _ei->property().GetWeight();
  };

  auto robots = this->GetGroupRoadmap()->GetGroup()->GetRobots();
  for(size_t i = 0; i < robots.size(); i++) {
    auto robot = robots[i];
    auto roadmap = this->GetGroupRoadmap()->GetIndividualGraph(i);

    std::vector<VID> vds = {m_goalMap.at(robot)};
    auto output = DijkstraSSSP(roadmap, vds, weight, termination);
    m_costToGoMap[robot] = output.distance;
  }
}

template <typename MPTraits>
double
MStarQuery<MPTraits>::
CompositeCostToGo(GroupCfgType& _gcfg) {
  auto total = 0.0;
  for(auto robot : _gcfg.GetRobots()) {
    auto vid = _gcfg.GetVID(robot);
    total += m_costToGoMap.at(robot).at(vid);
  }

  return total;
}

template <typename MPTraits>
bool
MStarQuery<MPTraits>::
IsGoal(GroupCfgType& _gcfg) {
  bool goal = true;
  for(auto robot : _gcfg.GetRobots()) {
    auto vid = _gcfg.GetVID(robot);
    if(vid != m_goalMap[robot]) {
      goal = false;
      break;
    }
  }

  return goal;
}

template <typename MPTraits>
void
MStarQuery<MPTraits>::
SetPath(Node* _goalNode) {
  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::FoundPath", 1);

  auto path = this->GetGroupPath();
  if(path)
    path->Clear();

  // Backtrack from the goal node until reaching the start
  Node* _node = _goalNode;
  std::vector<VID> vids = {_goalNode->vid};
  while(_node->backPtr != nullptr) {
    vids.push_back(_node->backPtr->vid);
    _node = _node->backPtr;
  }
  std::reverse(vids.begin(), vids.end());

  // Add vids to the solution path
  *this->GetGroupPath() += vids;
}

template <typename MPTraits>
std::unordered_set<Robot*>
MStarQuery<MPTraits>::
GetCollisions(GroupCfgType& _gcfg) {
  std::unordered_set<Robot*> collided;
  auto robots = _gcfg.GetRobots();
  auto vc = static_cast<CollisionDetectionValidityMethod<MPTraits>*>(
              this->GetMPLibrary()->GetValidityChecker(m_vcLabel));

  for(size_t i = 0; i < robots.size(); i++) {
    auto cfg1 = _gcfg.GetRobotCfg(robots[i]);
    cfg1.ConfigureRobot();
    auto mb1 = robots[i]->GetMultiBody();

    for(size_t j = i+1; j < robots.size(); j++) {
      auto cfg2 = _gcfg.GetRobotCfg(robots[j]);
      cfg2.ConfigureRobot();
      auto mb2 = robots[j]->GetMultiBody();

      CDInfo cdInfo;
      auto collision = vc->IsMultiBodyCollision(cdInfo,
                          mb1, mb2, this->GetNameAndLabel());
      if(collision) {
        collided.insert(robots[i]);
        collided.insert(robots[j]);
      }
    }
  }

  return collided;
}

template <typename MPTraits>
std::unordered_set<typename MStarQuery<MPTraits>::Node*>
MStarQuery<MPTraits>::
GetLimitedNeighbors(Node* _node, std::set<Node*, NodeComp>& _openSet) {
  std::unordered_map<Robot*, std::unordered_set<VID>> options;
  auto gcfg = this->GetGroupRoadmap()->GetVertex(_node->vid);

  if(this->m_debug) {
    std::cout << "Computing the limited neighbors of " << _node->vid << std::endl;
    std::cout << "\tCollision set has size: " 
              << (m_collisionSets.count(_node) ? m_collisionSets[_node].size() : 0)
              << std::endl;
  }

  auto robots = this->GetGroupRoadmap()->GetGroup()->GetRobots();
  for(size_t i = 0; i < robots.size(); i++) {
    auto robot = robots[i];
    auto vid = gcfg.GetVID(robot);
    auto roadmap = this->GetGroupRoadmap()->GetIndividualGraph(i);
    auto vit = roadmap->find_vertex(vid);
    
    // If this robot is in the collision set, consider all possible neighbors
    if(m_collisionSets.count(_node) and m_collisionSets.at(_node).count(robot)) {
      for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
        options[robot].insert(eit->target());
      }
    } else {
      // Not in the collision set, only consider the best next step
      auto minCost = std::numeric_limits<double>::max();
      VID bestTarget = INVALID_VID;
      for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
        if(m_costToGoMap.at(robot).at(eit->target()) < minCost) {
          minCost = m_costToGoMap[robot][eit->target()];
          bestTarget = eit->target();
        }
      }
      options[robot].insert(bestTarget);
    }
  }

  // Construct all combinations of options
  std::vector<std::unordered_map<Robot*, VID>> neighborSets;
  neighborSets.push_back(std::unordered_map<Robot*, VID>());
  for(auto robot : robots) {
    std::vector<std::unordered_map<Robot*, VID>> newNeighbors;

    for(auto option : options.at(robot)) {
      for(auto set : neighborSets) {
        set[robot] = option;
        newNeighbors.push_back(set);
      }
    }

    neighborSets = newNeighbors;
  }

  // Construct a group cfg and node for each neighbor
  std::unordered_set<Node*> limitedNeighbors;
  for(auto set : neighborSets) {
    auto newGcfg = GroupCfgType(this->GetGroupRoadmap());

    for(auto iter : set) {
      newGcfg.SetRobotCfg(iter.first, iter.second);
    }

    auto newVID = this->GetGroupRoadmap()->GetVID(newGcfg);
    auto newNode = (newVID == INVALID_VID);
    if(newNode) {
      newVID = this->GetGroupRoadmap()->AddVertex(newGcfg);
    }
    if(!m_nodeMap.count(newVID)) {
      auto heur = CompositeCostToGo(newGcfg);
      m_nodeMap[newVID] = Node(newVID, std::numeric_limits<double>::max(), heur, _node);
    }
        
    // Check if this edge is valid, if not, do not add to neighbors
    if(!this->GetGroupRoadmap()->IsEdge(_node->vid, newVID)) {
      auto collided = ValidateEdge(_node->vid, newVID);
      if(collided.size() > 0) {
        // Add the collided robots to the collision set if it's new
        for(auto robot : collided) {
          if(!m_collisionSets.count(_node) or !m_collisionSets[_node].count(robot)) {
            m_collisionSets[_node].insert(robot);
            _openSet.insert(_node);
          }
        }
        continue;
      }
    }

    limitedNeighbors.insert(&m_nodeMap[newVID]);
  }

  if(this->m_debug)
    std::cout << "\tFound " << limitedNeighbors.size() << " neighbors." << std::endl;

  return limitedNeighbors;
}

template <typename MPTraits>
void
MStarQuery<MPTraits>::
BackpropCollisions(Node* _current, Node* _neighbor, 
            std::set<Node*, NodeComp>& _openSet) {
  auto& currentSet = m_collisionSets[_current];
  auto& neighborSet = m_collisionSets[_neighbor];

  auto changed = false;
  for(auto node : neighborSet) {
    if(!currentSet.count(node)) {
      currentSet.insert(node);
      changed = true;
    }
  }

  if(!changed)
    return;

  if(!_openSet.count(_current)) {
    _openSet.insert(_current);
  }

  for(auto back : _current->backSet) {
    BackpropCollisions(_current, back, _openSet);
  }
}

template <typename MPTraits>
std::unordered_set<Robot*>
MStarQuery<MPTraits>::
ValidateEdge(VID _source, VID _target) {
  if(m_failedEdges.count(std::make_pair(_source, _target)))
    return m_failedEdges.at(std::make_pair(_source, _target));

  auto c = this->GetConnector(m_connLabel);

  std::vector<GroupCfgType> collision;
  auto collisionInsert = std::back_inserter(collision);
  const std::unordered_set<VID> target = {_target};
  c->Connect(this->GetGroupRoadmap(), _source, &target, 
                                &collisionInsert);
  
  if(this->GetGroupRoadmap()->IsEdge(_source, _target))
    return {};

  auto collided = GetCollisions(collision.at(0));
  m_failedEdges[std::make_pair(_source, _target)] = collided;
  return collided;
}

#endif
