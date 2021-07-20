#include "HCRQuery.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"
#include "TMPLibrary/Solution/Plan.h"

#include "Utilities/Hypergraph.h"

/*----------------------- Construction -----------------------*/

HCRQuery::
HCRQuery() {
  this->SetName("HCRQuery");
}

HCRQuery::
HCRQuery(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("HCRQuery");
  m_sgLabel = _node.Read("sgLabel", true, "", 
           "Temp till stategraph is embedded in plan.");

  m_vcLabel = _node.Read("vcLabel", true, "", 
           "Validity checker to use in validating paths.");
}

HCRQuery::
~HCRQuery() {}

/*------------------------ Interface -------------------------*/

bool
HCRQuery::
Run(Plan* _plan) {

  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());
  
  if(m_debug) {
    hcr->GetHypergraph()->Print();
  }

  //auto path = PerformHyperpathQuery();
  auto node = PerformCBSQuery();
  
  if(node.solutionMap.empty())
    return false;

  ExtractPlan(node);

  return true;
}

/*--------------------- Helper Functions ---------------------*/
    
HCRQuery::Node 
HCRQuery::
PerformCBSQuery() {
  
  CBSLowLevelPlanner<Robot,CT,Path> lowlevel(
    [this](Node _node, Robot* _robot) {
      auto hyperpath = this->PerformHyperpathQuery();
      if(hyperpath.empty())
        return false;

      auto individualPaths = this->ExtractPaths(hyperpath);
    
      for(auto path : individualPaths) {
        _node.solutionMap[path->GetRobot()] = path;
      }

      return true;
    }
  );

  CBSValidationFunction<Robot,CT,Path> validate(
    [this](Node _node) {
      return this->Validate(_node);
    }
  );

  CBSSplitNodeFunction<Robot,CT,Path> split(
    [this](Node _node, std::vector<std::pair<Robot*,CT>> _constraints,
           CBSLowLevelPlanner<Robot,CT,Path> _lowlevel) {
      return this->SplitNode(_node,_constraints,_lowlevel);
    }
  );

  CBSCostFunction<Robot,CT,Path> cost(
    [this](Node _node) {
      double cost = 0;
      for(auto path : _node.solutionMap) {
        if(!path.second)
          continue;

        double length = path.second->Length();

        if(this->m_soc) 
          cost += length;
        else 
          cost = std::max(cost,length);
      }
      return cost;
    }
  );

  CBSInitialFunction<Robot,CT,Path> initial(
    [lowlevel](Node _node) {
      return lowlevel(_node,nullptr);
    }
  );

  std::vector<Robot*> robots; 
  auto c = this->GetPlan()->GetCoordinator();
  for(auto group : c->GetInitialRobotGroups()) {
    for(auto robot : group.first->GetRobots())
      robots.push_back(robot);
  }

  auto node = CBS(robots,validate,split,lowlevel,cost,initial);

  return node;
}

std::vector<std::pair<Robot*,HCRQuery::CT>> 
HCRQuery::
Validate(Node _node) {

  // Recreate each path at resolution level.
  std::unordered_map<Robot*,vector<Cfg>> cfgPaths;
  for(auto sol : _node.solutionMap) {
    auto robot = sol.first;
    auto path = sol.second;
    cfgPaths[robot] = path->FullCfgs(this->GetMPLibrary());
  }

  // Find the latest timestep in which each robot is still moving.
  size_t lastTimestep = 0;
  for(auto& path : cfgPaths) {
    lastTimestep = std::max(lastTimestep,path.second.size());
  }

  auto vc = static_cast<CollisionDetectionValidity<MPTraits<Cfg,DefaultWeight<Cfg>>>*>(
    this->GetMPLibrary()->GetValidityChecker(m_vcLabel));

  // Step through each timestep
  for(size_t t = 0; t < lastTimestep; t++) {
    // Collision check each robot path against all others at this timestep.
    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end();) {
      // Configur the first robot at the appropriate configuration.
      auto robot1        = iter1->first;
      const auto& path1  = iter1->second;
      const size_t step1 = std::min(t,path1.size() - 1);
      const auto& cfg1   = path1[step1];
      auto multibody1    = robot1->GetMultiBody();
      multibody1->Configure(cfg1);

      // Compare to all remaining robots.
      for(auto iter2 = ++iter1; iter2 != cfgPaths.end(); ++iter2) {
        // Configure the second robot at the appropriate configuration.
        auto robot2        = iter2->first;
        const auto& path2  = iter2->second;
        const size_t step2 = std::min(t,path2.size() - 1);
        const auto& cfg2   = path2[step2];
        auto multibody2    = robot2->GetMultiBody();
        multibody2->Configure(cfg2);

        // Check for collision. If none, move on.
        CDInfo cdInfo;
        const bool collision = vc->IsMultiBodyCollision(cdInfo,
          multibody1,multibody2, this->GetNameAndLabel());
        if(!collision)
          continue;

        if(this->m_debug) {
          std::cout << "\t\tConflict detected at timestep " << t
                    << " (time " << this->GetMPProblem()->GetEnvironment()->GetTimeRes() * t
                    << ")."
                    << "\n\t\t\tRobot " << robot1->GetLabel() << ": "
                    << cfg1.PrettyPrint()
                    << "\n\t\t\tRobot " << robot2->GetLabel() << ": "
                    << cfg2.PrettyPrint()
                    << std::endl;
        }
        
        // Make cbs constraints.
        CT constraint1 = std::make_pair(t,cfg2);
        CT constraint2 = std::make_pair(t,cfg1);
    
        std::vector<std::pair<Robot*,CT>> constraintSet;
        constraintSet.push_back(std::make_pair(robot1,constraint1));
        constraintSet.push_back(std::make_pair(robot2,constraint2));
  
        return constraintSet;
      }
    }
  }

  if(this->m_debug)
    std::cout << "No conflict detected." << std::endl;

  return {};
}

std::vector<HCRQuery::Node> 
HCRQuery::
SplitNode(Node _node, 
          std::vector<std::pair<Robot*,CT>> _constraints,
          CBSLowLevelPlanner<Robot,CT,Path> _lowlevel) {

  std::vector<Node> children;

  for(const auto& constraint : _constraints) {
    Robot* robot =  constraint.first;
    auto c = constraint.second;

    // Verify that this is not a duplicate constraint.
    if(_node.constraintMap.at(robot).count(c))
      throw RunTimeException(WHERE) << "Duplicaiting cbs constraint in HCRQuery."
                                    << "\nRobot: " << robot->GetLabel()
                                    << "\nConstraint: {Robot: " 
                                    << c.second.GetRobot()->GetLabel()
                                    << ", Cfg: " 
                                    << c.second.PrettyPrint()
                                    << ", Timestep: " << c.first 
                                    << "}" << std::endl;

    Node child = _node;
    child.constraintMap[robot].insert(c);
    children.push_back(std::move(child));
  }

  return children;
}

std::vector<HCRQuery::Path*>
HCRQuery::
ExtractPaths(const std::vector<HPElem>& _hyperpath) {
  return {};
}


void
HCRQuery::
ExtractPlan(Node _node) {
  auto plan = this->GetPlan();

  //TODO:: Convert hyperpath into solution representation.

  if(m_debug)
    plan->Print();
}

std::vector<HCRQuery::HPElem>
HCRQuery::
PerformHyperpathQuery() {

  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());
  auto hypergraph = hcr->GetHypergraph();

  // Perform quick sanity check that the goal has been connected
  // to anything else in the graph.
  if(hypergraph->GetIncomingHyperarcs(1).size() == 0)
    return {};

  SSSHPTerminationCriterion termination(
    [](size_t& _vid, const MBTOutput& _mbt) {
      if(_vid == 1)
        return SSSHPTermination::EndSearch;
      return SSSHPTermination::Continue;
  });

  SSSHPPathWeightFunction<TMPVertex,TMPHyperarc> weight(
    [](const typename Hypergraph<TMPVertex,TMPHyperarc>::Hyperarc& _hyperarc,
       const std::unordered_map<size_t,double> _weightMap,
       const size_t _target) {

      double hyperarcWeight; 
      //if(_hyperarc.property.semantic) {
      //  hyperarcWeight = 0;
      //}
      //else {
        hyperarcWeight = 0;
        for(auto& path : _hyperarc.property.paths) {
          //hyperarcWeight = std::max(hyperarcWeight,path->Length());
          hyperarcWeight = std::max(hyperarcWeight,double(path->TimeSteps()));
        }
      //}

      double tailWeight = 0;

      for(auto vid : _hyperarc.tail) {
        tailWeight = std::max(tailWeight, _weightMap.at(vid));
      }

      return hyperarcWeight + tailWeight;
    }); 

  auto output = SBTDijkstra(hypergraph,0,weight,termination);


  // Check if query found a path.
  auto last = output.ordering.back();
  if(last != 1)
    return {};

  HPElem source = std::make_pair(true,0);
  std::set<HPElem> parents = {source};

  auto path = ConstructPath(last,parents,output);
  path = AddDanglingNodes(path,parents);

  if(m_debug) {
    std::cout << "Full Path" << std::endl;
    for(auto e : path) {
      if(e.first) 
        std::cout << "v";
      else
        std::cout << "h";

      std::cout << e.second << ", ";
    }
    std::cout << std::endl;
  }

  return path;
}

std::vector<HCRQuery::HPElem>
HCRQuery::
ConstructPath(size_t _sink, std::set<HPElem>& _parents, MBTOutput& _mbt) {

  std::vector<HPElem> path;

  HPElem current;
  current.first = true;
  current.second = _sink;

  path.push_back(current);

  while(!_parents.count(current)) {
    _parents.insert(current);
    if(current.first) {
      current.second = _mbt.vertexParentMap[current.second];
    }
    else {
      current.second = _mbt.hyperarcParentMap[current.second];
    }

    current.first = !current.first;

    path.push_back(current);
  }
    
  _parents.insert(current);
  std::reverse(path.begin(), path.end());

  path = AddBranches(path, _parents, _mbt);

  return path;
}

std::vector<HCRQuery::HPElem>
HCRQuery::
AddBranches(std::vector<HPElem> _path, std::set<HPElem>& _parents, MBTOutput& _mbt) {
  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());
  auto hypergraph = hcr->GetHypergraph();

  std::vector<HPElem> finalPath = _path;

  size_t offset = 0;

  for(size_t i = 0; i < _path.size(); i++) {
    auto elem = _path[i];

    if(elem.first) 
      continue;

    auto arc = hypergraph->GetHyperarc(elem.second);

    // Check if tail set is accounted for.
    for(auto vid : arc.tail) {

      HPElem e;
      e.first = true;
      e.second = vid;

      if(!_parents.count(e)) {
        // If tail vertex not in path, compute its branch back to parents.
        auto branch = ConstructPath(vid,_parents,_mbt);

        // Add branch to final path.
        auto iter = finalPath.begin();
        auto branchStart = branch.begin();
        branchStart++;
        finalPath.insert(iter+(i+offset),branchStart,branch.end());

        // Update offset
        offset += branch.size();
      }
    }
  }

  return finalPath;
}

std::vector<HCRQuery::HPElem>
HCRQuery::
AddDanglingNodes(std::vector<HPElem> _path, std::set<HPElem>& _parents) {

  auto hcr = dynamic_cast<CombinedRoadmap*>(
              this->GetStateGraph(m_sgLabel).get());
  auto hypergraph = hcr->GetHypergraph();
  
  std::vector<HPElem> finalPath = _path;

  size_t offset = 0;

  for(size_t i = 0; i < _path.size(); i++) {
    auto elem = _path[i];
    if(elem.first) 
      continue;

    const auto arc = hypergraph->GetHyperarc(elem.second);
    
    for(auto vid : arc.head) {
      HPElem e;
      e.first = true;
      e.second = vid;

      if(_parents.count(e))
        continue;

      _parents.insert(e);

      auto iter = finalPath.begin();
      finalPath.insert(iter+(i+offset+1),e);
      
      offset++;
    }
  }

  return finalPath; 
}

bool
HCRQuery::
Soc() {
  return m_soc;
}
/*------------------------------------------------------------*/
