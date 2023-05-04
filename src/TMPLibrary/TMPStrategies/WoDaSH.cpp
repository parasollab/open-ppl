#include "WoDaSH.h"

#include "MPLibrary/MapEvaluators/MapEvaluatorMethod.h"
#include "MPLibrary/MPSolution.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/ActionSpace/Action.h"
#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/ActionSpace/ProximityCondition.h"
#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"
#include "TMPLibrary/StateGraphs/GroundedHypergraph.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"
#include "TMPLibrary/TaskEvaluators/ScheduledCBS.h"

#include "MPLibrary/MPStrategies/CompositeDynamicRegionRRT.h"
#include "Utilities/CBS.h"
#include "Utilities/SSSP.h"
#include "Workspace/PropertyMap.h"

/*----------------------- Construction -----------------------*/

WoDaSH::
WoDaSH() {
  this->SetName("WoDaSH");
}

WoDaSH::
WoDaSH(XMLNode& _node) : TMPStrategyMethod(_node) {
  this->SetName("WoDaSH");

  m_drStrategy = _node.Read("drStrat", true, "", 
          "The dynamic regions stategy to ground hyperskeleton arcs");

  m_trajStrategy = _node.Read("trajStrat", true, "",
          "The fallback strategy to use to plan trajectories between hyperskeleton arcs");

  m_sampler = _node.Read("sampler", true, "",
          "The sampler to use to generate spawn vertices on edges");

  m_replanMethod = _node.Read("replanMethod", false, "global", 
          "How to get the next best MAPF solution (global, resume, or lazy");

  m_skeletonType = _node.Read("skeletonType", true, "",
      "the type of skeleton to use, Available options are reeb and mcs "
      "for 3d, ma for 2d");

  m_skeletonIO = _node.Read("skeletonIO", false, "", "read of write the "
      "skeleton file");

  m_skeletonFilename = _node.Read("skeletonFile", m_skeletonIO != "", "",
      "the skeleton file to read from or write to");

  // If using a reeb skeleton, we need a decomposition to build it.
  m_decompositionLabel = _node.Read("decompositionLabel",
      m_skeletonType == "reeb", "",
      "The workspace decomposition to use.");

  m_scuLabel = _node.Read("scuLabel", false, "", "The skeleton clearance utility "
      "to use. If not specified, we use the hack-fix from wafr16.");

  m_regionFactor = _node.Read("regionFactor", true,
      m_regionFactor, 1., std::numeric_limits<double>::max(),
      "Regions are this * robot's bounding sphere radius");

  m_penetrationFactor = _node.Read("penetration", true,
      m_penetrationFactor, std::numeric_limits<double>::min(), 1.,
      "Fraction of bounding sphere penetration that is considered touching");

  m_edgeQueryLabel = _node.Read("edgeQuery",true,"",
      "Label of the query method to extract paths alongs grounded edges.");

  m_groundedHypergraphLabel = _node.Read("groundedHypergraph",true,"",
      "Label of the grounded hypergraph to use.");

  m_motionEvaluator = _node.Read("motionEvaluator",true,"",
              "Evaluator label for motion planning.");
}

WoDaSH::
~WoDaSH() {}

/*------------------------ Interface -------------------------*/

void
WoDaSH::
Initialize() {

  {
    auto plan = this->GetPlan();
    auto decomp = plan->GetDecomposition();
    auto st = decomp->GetGroupMotionTasks()[0];
    auto groupTask = st->GetGroupMotionTask();
    this->GetMPLibrary()->SetGroupTask(groupTask.get());
    this->GetMPLibrary()->SetMPSolution(this->GetMPSolution());
  }

  // TODO actually use this (for now just using global)
  if(!(m_replanMethod == "global" or m_replanMethod == "resume" or m_replanMethod == "lazy"))
    throw RunTimeException(WHERE) << "Invalid MAPF replan method given. Options are global, resume, and lazy.";

  // Initialize the skeleton and regions.
  BuildSkeleton();
  m_skeleton = std::unique_ptr<HypergraphSkeletonType>(new HypergraphSkeletonType());

  // Get the radius of each robot's region
  auto groupTask = this->GetMPLibrary()->GetGroupTask();
  for(auto& task : *groupTask) {
    auto robot = task.GetRobot();
    m_taskMap.emplace(std::make_pair(robot, &task));
    const double robotRadius = robot->GetMultiBody()->
                               GetBoundingSphereRadius() * m_regionFactor;
    m_regionRadius.insert(std::make_pair(robot, robotRadius));

    // Get the goal skeleton vertex for each robot
    if(task.GetGoalConstraints().size() != 1) 
      throw RunTimeException(WHERE) << "Exactly one goal is required.";

    const auto goal_center = task.GetGoalConstraints()[0]->GetBoundary()->GetCenter();
    const bool threeD = robot->GetMultiBody()->GetBaseType()
                      == Body::Type::Volumetric;

    double centerVec[3];
    centerVec[0] = goal_center[0];
    centerVec[1] = goal_center[1];
    if(threeD)
      centerVec[2] = goal_center[2];
    else
      centerVec[2] = 0.;

    auto skelIter = m_indSkeleton.FindNearestVertex(Vector<double, 3>(centerVec));
    m_skeletonGoals[robot] = skelIter->descriptor();

    // Get the start skeleton vertex for each robot
    const auto start_center = task.GetStartConstraint()->GetBoundary()->GetCenter();

    centerVec[0] = start_center[0];
    centerVec[1] = start_center[1];
    if(threeD)
      centerVec[2] = start_center[2];
    else
      centerVec[2] = 0.;

    skelIter = m_indSkeleton.FindNearestVertex(Vector<double, 3>(centerVec));
    m_skeletonStarts[robot] = skelIter->descriptor();
  }

  // Set up for running MAPF solution
  InitializeCostToGo();
}

void
WoDaSH::
PlanTasks() {
  auto te = this->GetTaskEvaluator(m_teLabel);
  te->Initialize();

  auto me = dynamic_cast<ScheduledCBS*>(this->GetTaskEvaluator(m_motionEvaluator).get());
  me->Initialize();

  auto plan = this->GetPlan();

  do {
    bool success = false;
    while(!success) {
      auto mapfSol = MAPFSolution();
      ConstructHyperpath(mapfSol);
      success = GroundHyperskeleton();
    }
    std::cout << "successfully found a path for each robot." << std::endl;

  } while(!te->operator()() or !me->operator()(plan));
}

/*--------------------- Helper Functions ---------------------*/

void
WoDaSH::
BuildSkeleton() {
  // auto stats = this->GetStatClass();
  // MethodTimer mt(stats, this->GetNameAndLabel() + "::BuildSkeleton");

  // Get robots.
  auto robots = this->GetMPLibrary()->GetGroupTask()->GetRobotGroup()->GetRobots();

  // Determine if we need a 2d or 3d skeleton.
  auto env = this->GetMPProblem()->GetEnvironment();
  const bool threeD = robots[0]->GetMultiBody()->GetBaseType() ==
      Body::Type::Volumetric;


  if(m_skeletonIO == "read") {
    m_indSkeleton.Read(m_skeletonFilename);
    m_indSkeleton.DoubleEdges();
  }
  else {
    if(threeD) {
      if(m_skeletonType == "mcs") {
        if(this->m_debug)
          std::cout << "Building a Mean Curvature skeleton." << std::endl;
        MeanCurvatureSkeleton3D mcs;
        mcs.SetEnvironment(this->GetMPProblem()->GetEnvironment());
        mcs.BuildSkeleton();

        // Create the workspace skeleton.
        auto sk = mcs.GetSkeleton();
        m_indSkeleton = sk.first;
        m_indSkeleton.DoubleEdges();
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
        m_indSkeleton = reeb.GetSkeleton();
        m_indSkeleton.DoubleEdges();
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
      m_indSkeleton = get<0>(ma.GetSkeleton(1)); // 1 for free space.
      m_indSkeleton.DoubleEdges();
    }
  }

  // Get the clearance annotations
  auto annot = ClearanceAnnotatedSkeleton(this->GetMPLibrary(), &m_indSkeleton, true);

  std::vector<WorkspaceSkeleton*> ws;
  for (size_t i = 0; i < robots.size(); ++i) {
    ws.push_back(&m_indSkeleton);
    m_annotationMap.insert(std::make_pair(robots[i], annot));
    if(this->m_debug)
      std::cout << "Added workspace skeleton for " << robots.at(i)->GetLabel() << "." << endl;
  }

  if(m_skeletonIO == "write")
    m_indSkeleton.Write(m_skeletonFilename);
}

void
WoDaSH::
InitializeCostToGo() {
  ///@todo find a way to use the implementation in cdr
  // Assumes the same skeleton for each of the robots
  auto ws = m_indSkeleton;
  auto robots = this->GetMPLibrary()->GetGroupTask()->GetRobotGroup()->GetRobots();
  for (auto r : robots) {

    SSSPTerminationCriterion<WorkspaceSkeleton> termination(
        [](typename WorkspaceSkeleton::vertex_iterator& _vi,
          const SSSPOutput<WorkspaceSkeleton>& _sssp) {
          return SSSPTermination::Continue;
        }
    );

    SSSPPathWeightFunction<WorkspaceSkeleton> weight = [&](
        typename WorkspaceSkeleton::adj_edge_iterator& _ei,
        const double _sourceDistance,
        const double _targetDistance) {

      const auto source = _ei->source();
      const auto target = _ei->target();

      const auto start = ws.find_vertex(source);
      const auto goal = ws.find_vertex(target);

      return _sourceDistance + (goal->property() - start->property()).norm();
    };

    std::vector<VID> vds = {m_skeletonGoals.at(r)};
    auto output = DijkstraSSSP(&ws, vds, weight, termination);
    m_distanceMap.insert(std::make_pair(r, output.distance));
  }
}

std::vector<std::pair<Robot*, typename WoDaSH::CBSConstraint>>
WoDaSH::
ValidationFunction(CBSNodeType& _node) {
  ///@todo find a way to use the implementation in cdr
  // auto stats = this->GetStatClass();
  // MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  size_t maxTimestep = 0;
  for(auto kv : _node.solutionMap) {
    maxTimestep = std::max(maxTimestep,kv.second->size());
  }

  std::unordered_map<VID, std::unordered_map<size_t, double>> vertexCapacity;
  std::unordered_map<std::pair<VID, VID>, 
      std::unordered_map<size_t, double>> edgeCapacity;

  // Check the global constraints from edges that failed to ground
  std::unordered_map<std::pair<VID, VID>, size_t> edgeOccupancy;

  // new constraints of form (robot, ((vid, vid), time))
  std::vector<std::pair<Robot*, CBSConstraint>> constraints;

  // iterate through robots first to get the remaining capacity
  for(size_t i = 0; i < maxTimestep; i++) {
    for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {
      auto robot = iter->first;
      auto path = *(iter->second);

      // source vertex (current timestep)
      auto s = std::min(i, path.size()-1);
      auto source = path[s];

      // target vertex (next timestep)
      auto t = std::min(i+1,path.size()-1);
      auto target = path[t];

      // Get the edge in the individual skeleton
      if(source != target) {
        // order such that source < target
        auto sVID = std::min(source, target);
        auto tVID = std::max(source, target);

        // Check if this robot is in a failed group
        for(auto iter : m_failedEdges) {
          auto edge = iter.first;
          auto group = iter.second;

          if(edge.first == sVID and edge.second == tVID and group->VerifyRobotInGroup(robot)) {
            if(edgeOccupancy.count(edge))
              edgeOccupancy[edge]++;
            else
              edgeOccupancy[edge] = 1;
          }
        }

        WorkspaceSkeleton::adj_edge_iterator ei;
        m_indSkeleton.GetEdge(sVID, tVID, ei);
        auto eid = ei->descriptor();
        auto edgePair = std::make_pair(sVID, tVID);

        // Check if this edge has been added
        if(edgeCapacity.find(edgePair) != edgeCapacity.end()) {
          // Check if this timestep has been added
          if(edgeCapacity.at(edgePair).find(i) != edgeCapacity.at(edgePair).end()) {
            // This edge and time are in the map, decrease the capacity as needed
            edgeCapacity[edgePair][i] -= robot->GetMultiBody()->GetBoundingSphereRadius();
          } else {
            // This time is not in the map, add it with the remaining capacity
            auto ds = m_annotationMap.at(robot)->GetEdgeProperty(eid);
            auto d = *std::min_element(ds.begin(), ds.end());
            d -= robot->GetMultiBody()->GetBoundingSphereRadius();
            edgeCapacity[edgePair].emplace(std::make_pair(i, d));
          }
        } else {
          // This edge needs to be added
          auto ds = m_annotationMap.at(robot)->GetEdgeProperty(eid);
          auto d = *std::min_element(ds.begin(), ds.end());
          d -= robot->GetMultiBody()->GetBoundingSphereRadius();

          std::unordered_map<size_t, double> eMap;
          eMap.emplace(i, d);
          edgeCapacity.emplace(std::make_pair(edgePair, eMap));
        }
      }
      
      // Check if this vid has been added yet
      if(vertexCapacity.find(source) != vertexCapacity.end()) {
        // Check if this timestep has been added
        if(vertexCapacity.at(source).find(i) != vertexCapacity.at(source).end()) {
          // This vid and time are in the map, decrease the capacity as needed
          vertexCapacity[source][i] -= robot->GetMultiBody()->GetBoundingSphereRadius();
        } else {
          // This time is not in the map, add it with the remaining capacity
          auto d = m_annotationMap.at(robot)->GetVertexProperty(source);
          d -= robot->GetMultiBody()->GetBoundingSphereRadius();
          vertexCapacity[source].emplace(std::make_pair(i, d));
        }
      } else {
        // This vid needs to be added
        auto d = m_annotationMap.at(robot)->GetVertexProperty(source);
        d -= robot->GetMultiBody()->GetBoundingSphereRadius();

        std::unordered_map<size_t, double> vMap;
        vMap.emplace(i, d);
        vertexCapacity.emplace(std::make_pair(source, vMap));
      }
    }
  }

  // Form a constraint for every robot, vid, timestep that has below 0 capacity remaining
  for(size_t i = 0; i < maxTimestep; i++) {
    for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

      // Find the VID of this robot at time i
      auto robot = iter->first;
      auto path = *(iter->second);

      auto s = std::min(i, path.size()-1);
      auto source = path[s];
      auto t = std::min(i+1,path.size()-1);
      auto target = path[t];

      // order such that source < target
      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);
      auto edgePair = std::make_pair(sVID, tVID);

      // Check for an edge
      if(source != target) {
        // Check the capacity of the vertex
        if(edgeCapacity.at(edgePair).at(i) < 0) {
          constraints.push_back(std::make_pair(robot, std::make_pair(edgePair, i)));
        } else if(edgeOccupancy.count(edgePair)) {
          auto group = m_failedEdges.at(edgePair);
          if(group->VerifyRobotInGroup(robot))
            constraints.push_back(std::make_pair(robot, std::make_pair(edgePair, i)));
        }
      }

      // Check the capacity of the source vertex
      if(vertexCapacity.at(source).at(i) < 0) {
        auto vPair = std::make_pair(source, SIZE_MAX);
        constraints.push_back(std::make_pair(robot, std::make_pair(vPair, i)));
      }
    }
  }

  return constraints;
}

std::vector<typename WoDaSH::CBSNodeType> 
WoDaSH::
SplitNodeFunction(CBSNodeType& _node,
        std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
        CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
        CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {
          ///@todo find a way to use the implementation in cdr
  // auto stats = this->GetStatClass();
  // MethodTimer mt(stats,this->GetNameAndLabel() + "::SplitNodeFunction");

  std::vector<CBSNodeType> newNodes;

  for(auto pair : _constraints) {
    // Unpack constraint info
    auto robot = pair.first;
    auto constraint = pair.second;

    // Copy parent node
    CBSNodeType child = _node;
  
    // Add new constraint
    child.constraintMap[robot].insert(constraint);

    // Replan tasks affected by constraint. Skip if no valid replanned path is found
    if(!_lowLevel(child,robot)) 
      continue;

    // Update the cost and add to set of new nodes
    double cost = _cost(child);
    child.cost = cost;
    newNodes.push_back(child);
  }

  return newNodes;
}

bool
WoDaSH::
LowLevelPlanner(CBSNodeType& _node, Robot* _robot) {
  ///@todo find a way to use the implementation in cdr
  // auto stats = this->GetStatClass();
  // MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");

  auto constraints = _node.constraintMap[_robot];
  auto h = std::shared_ptr<HeuristicSearch>(new HeuristicSearch(_robot));

  // Get the start and goal vids in the individual skeleton
  auto start = m_skeletonStarts.at(_robot);
  auto goal = m_skeletonGoals.at(_robot);

  std::cout << _robot->GetLabel() << " s: " << start << " g: " << goal << std::endl;

  // Check that start does not violate a constraint and get min end time
  size_t minEndTimestep = 0;
  for(auto constraint : constraints) {

    minEndTimestep = std::max(minEndTimestep,constraint.second);

    if(constraint.first.second != SIZE_MAX or start != constraint.first.first)
      continue;

    if(constraint.second <= 0)
      return false;
  }

  // Distance from each skeleton vertex to the goal
  auto dist2go = m_distanceMap.at(_robot);
  auto g = &m_indSkeleton;

  auto startVertex = std::make_pair(start,0);
  auto startVID = h->AddVertex(startVertex);

  SSSPTerminationCriterion<HeuristicSearch> termination(
    [goal,minEndTimestep](typename HeuristicSearch::vertex_iterator& _vi,
           const SSSPOutput<HeuristicSearch>& _sssp) {
      
      auto vertex = _vi->property();

      if(goal == vertex.first and minEndTimestep <= vertex.second)
        return SSSPTermination::EndSearch;

      return SSSPTermination::Continue;
    }
  );

  SSSPPathWeightFunction<HeuristicSearch> weight(
    [constraints,h](typename HeuristicSearch::adj_edge_iterator& _ei,
       const double _sourceDistance,
       const double _targetDistance) {
     
      auto source = h->GetVertex(_ei->source()).first;
      auto target = h->GetVertex(_ei->target()).first;
      auto timestep = h->GetVertex(_ei->source()).second;

      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);

      for(auto constraint : constraints) {
        // Check if the edge violates a constraint
        if(sVID == constraint.first.first and tVID == constraint.first.second) {
          if (timestep == constraint.second) {
            return std::numeric_limits<double>::infinity();
          }
        }

        // Ignore any other edges
        if(constraint.first.second != SIZE_MAX)
          continue;

        // Check for vertex constraint
        if(source != constraint.first.first)
          continue;
    
        if(timestep == constraint.second)
          return std::numeric_limits<double>::infinity();
      }

      return _sourceDistance + _ei->property();
    }
  );

  SSSPHeuristicFunction<HeuristicSearch> heuristic(
    [dist2go](const HeuristicSearch* _h, 
       typename HeuristicSearch::vertex_descriptor _source,
       typename HeuristicSearch::vertex_descriptor _target) {

      // Distance to go heuristic
      auto vertex = _h->GetVertex(_target);
      double toGo = dist2go.at(vertex.first);

      return std::max(0.0,toGo);
    }
  );

  SSSPNeighborsFunction<HeuristicSearch> neighbors(
    [g](HeuristicSearch* _h, typename HeuristicSearch::vertex_descriptor _vid) {
      auto vertex = _h->GetVertex(_vid);
      auto gvid = vertex.first;
      auto timestep = vertex.second;
      
      auto vit = g->find_vertex(gvid);

      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        auto target = eit->target();
        auto neighbor = std::make_pair(target,timestep+1);
        // auto edge = eit->property();

        // Use atomic edges
        auto nvid = _h->AddVertex(neighbor);
        _h->AddEdge(_vid,nvid,1.0);
      }

      // Add a waiting edge
      auto neighbor = std::make_pair(gvid,timestep+1);
      auto nvid = _h->AddVertex(neighbor);
      _h->AddEdge(_vid,nvid,1.0);
    }
  );

  std::vector<size_t> starts = {startVID};
  std::vector<size_t> goals = {goal};

  auto sssp = AStarSSSP(h.get(),starts,goals,weight,heuristic,neighbors,termination);

  // Check that a path was found
  const size_t last = sssp.ordering.back();
  if(h->GetVertex(last).first != goal or h->GetVertex(last).second < minEndTimestep) {
    if(this->m_debug) {
      std::cout << "Failed to find a path for " << _robot->GetLabel() << std::endl;
    }
    return false;
  }

  // Reconstruct the path
  std::vector<size_t> path = {h->GetVertex(last).first};
  auto current = last;
  do {
    current = sssp.parent.at(current);
    path.push_back(h->GetVertex(current).first);
  } while(current != startVID);
  std::reverse(path.begin(),path.end());

  // Save path in solution
  _node.solutionMap[_robot] = new vector<size_t>();
  *(_node.solutionMap[_robot]) = path;

  return true;
}

double
WoDaSH::
CostFunction(CBSNodeType& _node) {
  ///@todo find a way to use the implementation in cdr
  // auto stats = this->GetStatClass();
  // MethodTimer mt(stats,this->GetNameAndLabel() + "::CostFunction");

  // For now, treat an edge as having cost 1, use makespan
  double cost = 0;
  for(auto kv : _node.solutionMap) {
    cost = std::max(cost,double(kv.second->size()));
  }

  return cost;
}

std::unordered_map<Robot*, typename WoDaSH::CBSSolution*>
WoDaSH::
MAPFSolution() {
  ///@todo find a way to use the implementation in cdr
  // auto stats = this->GetStatClass();
  // MethodTimer mt(stats, this->GetNameAndLabel() + "::ComputeMAPFHeuristic");

  // Configure CBS Functions
  CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution> lowLevel(
    [this](CBSNodeType& _node, Robot* _task) {
      return this->LowLevelPlanner(_node,_task);
    }
  );

  CBSValidationFunction<Robot,CBSConstraint,CBSSolution> validation(
    [this](CBSNodeType& _node) {
      return this->ValidationFunction(_node);
    }
  );

  CBSCostFunction<Robot,CBSConstraint,CBSSolution> cost(
    [this](CBSNodeType& _node) {
      return this->CostFunction(_node);
    }
  );

  CBSSplitNodeFunction<Robot,CBSConstraint,CBSSolution> splitNode(
    [this](CBSNodeType& _node, std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
           CBSLowLevelPlanner<Robot,CBSConstraint,CBSSolution>& _lowLevel,
           CBSCostFunction<Robot,CBSConstraint,CBSSolution>& _cost) {
      return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
    }
  );

  auto robots = this->GetMPLibrary()->GetGroupTask()->GetRobotGroup()->GetRobots();
  CBSNodeType solution = CBS(robots,validation,splitNode,lowLevel,cost);

  if(this->m_debug) {
    std::cout << "Heuristic Paths" << std::endl;
    for(auto kv : solution.solutionMap) {
      auto robot = kv.first;
      auto path = *kv.second;
      std::cout << "\t" << robot->GetLabel() << ": ";
      for(auto vid : path) {
        std::cout << vid << ", ";
      }
      std::cout << std::endl;
    }
    std::cout << "Heuristic Cost: " << solution.cost << std::endl;
  }

  return solution.solutionMap;
}

void
WoDaSH::
ConstructHyperpath(std::unordered_map<Robot*, CBSSolution*> _mapfSolution) {
  // First construct the "movement" hyperarcs along skeleton edges
  size_t maxTimestep = 0;
  for(auto kv : _mapfSolution) {
    maxTimestep = std::max(maxTimestep,kv.second->size());
  }

  m_path.Reset();

  std::vector<std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>>> hyperarcGroups;
  for(size_t t = 0; t < maxTimestep - 1; t++) {
    std::unordered_map<Robot*, SkeletonEdgeDescriptor> individualEdges;
    std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>> edgeGroups;

    for(auto kv : _mapfSolution) {
      auto robot = kv.first;
      auto path = *kv.second;

      std::cout << robot->GetLabel() << std::endl;
      std::cout << path << std::endl;

      auto source = path.size() > t ? path.at(t) : path.at(path.size() - 1);
      auto target = path.size() > t + 1 ? path.at(t+1) : path.at(path.size() - 1);

      std::cout << source << " " << target << std::endl;

      // group with robots going both ways on this edge
      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);

      edgeGroups[std::make_pair(sVID, tVID)].push_back(robot);
      std::cout << edgeGroups.size() << std::endl;

      SkeletonEdgeIterator ei;
      auto found = m_indSkeleton.GetEdge(source, target, ei);

      SkeletonEdgeDescriptor ed;
      if(found) {
        ed = ei->descriptor();
      } else {
        // Add self edge
        ed = m_indSkeleton.AddEdge(source, target);
      }
      individualEdges.emplace(robot, ed);
      std::cout << individualEdges.size() << std::endl;
    }
    hyperarcGroups.push_back(edgeGroups);

    // Construct the hyperskeleton locally
    for(auto iter : edgeGroups) {
      auto group = AddGroup(iter.second);
      auto svertex = CompositeSkeletonVertex(group, &m_indSkeleton);
      auto tvertex = CompositeSkeletonVertex(group, &m_indSkeleton);
      auto cedge = CompositeSkeletonEdge(group, &m_indSkeleton);

      std::cout << "starting" << std::endl;

      for(auto robot : group->GetRobots()) {
        auto ed = individualEdges.at(robot);
        auto source = ed.source();
        auto target = ed.target();

        svertex.SetRobotCfg(robot, source);
        tvertex.SetRobotCfg(robot, target);

        cedge.SetEdge(robot, ed);

        std::cout << robot->GetLabel() << ": " << source << " " << target << std::endl;
      }

      // Construct the corresponding hyperskeleton vertices and edges
      auto svid = m_skeleton->AddVertex(svertex);
      auto tvid = m_skeleton->AddVertex(tvertex);

      // If source and target are the same, the group is waiting
      if(svid == tvid) {
        for(auto robot : group->GetRobots())
          m_hidPaths[t][robot] = std::make_pair(false, svid);
        
        continue;
      }

      auto arc = HyperskeletonArc(cedge, HyperskeletonArcType::Movement);
      auto hid = m_skeleton->AddHyperarc({tvid}, {svid}, arc);
      m_path.movementHyperarcs.insert(hid);

      // For ease of future use, set predecessor and successor to self
      for(auto robot : group->GetRobots()) {
        m_path.predecessors[hid].emplace(robot, std::make_pair(true, hid));
        m_path.successors[hid].emplace(robot, std::make_pair(true, hid));
        m_hidPaths[t][robot] = std::make_pair(true, hid);
      }
    }
  }

  // Now construct the "composition" hyperperarcs to transition b/w groups
  for(size_t t = 0; t < maxTimestep - 2; t++) {
    auto edgeGroup = hyperarcGroups.at(t);
    std::unordered_map<VID, std::set<VID>> incidentHVIDs;

    std::cout << "TIME " << t << std::endl;
    
    // Split hyperarcs that contain robots going opposite directions ("decouple" 
    // hyperarcs - do nothing, just for connectivity)
    for(auto iter : edgeGroup) {
      auto robots = iter.second;
      auto shid = m_hidPaths[t][robots[0]];

      // If currently waiting, skip this group since we can't decouple, but still
      // add the waiting vertex to incident vids so that we can merge/split
      // if necessary
      if(!shid.first) {
        auto path = *_mapfSolution[robots.at(0)];
        auto target = path.size() > t + 1 ? path.at(t+1) : path.at(path.size() - 1);
        incidentHVIDs[target].insert(shid.second);
        continue;
      }

      auto svertex = *(m_skeleton->GetHyperarc(shid.second).head.begin());

      std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>> splitGroups;
      for(auto robot : robots) {
        auto path = *_mapfSolution[robot];

        auto source = path.size() > t ? path.at(t) : path.at(path.size() - 1);
        auto target = path.size() > t + 1 ? path.at(t+1) : path.at(path.size() - 1);

        auto edgePair = std::make_pair(source, target);
        splitGroups[edgePair].push_back(robot);
      }

      if(splitGroups.size() == 1) {
        auto robot = (*splitGroups.begin()).second.at(0);
        auto path = *_mapfSolution[robot];
        auto target = path.size() > t + 1 ? path.at(t+1) : path.at(path.size() - 1);
        incidentHVIDs[target].insert(svertex);
        std::cout << "Not splitting here" << std::endl;
        continue;
      }

      std::set<HID> tvids;
      for(auto sg : splitGroups) {
        auto edge = sg.first;
        auto group = AddGroup(sg.second);
        auto tvertex = CompositeSkeletonVertex(group, &m_indSkeleton);

        for(auto robot : group->GetRobots())
          tvertex.SetRobotCfg(robot, edge.second);
        
        auto tvid = m_skeleton->AddVertex(tvertex);
        incidentHVIDs[edge.second].insert(tvid);
        tvids.insert(tvid);

        std::cout << "nop splitting now" << std::endl;
      }
      auto cgroup = m_skeleton->GetVertexType(svertex).GetGroup();
      auto cedge = CompositeSkeletonEdge(cgroup, &m_indSkeleton);
      auto arc = HyperskeletonArc(cedge, HyperskeletonArcType::Decouple);
      auto hid = m_skeleton->AddHyperarc(tvids, {svertex}, arc);
      m_path.decoupleHyperarcs.insert(hid);

      // Set the predecessor movement hyperarc
      for(auto ih : m_skeleton->GetIncomingHyperarcs(svertex)) {
        if(m_path.movementHyperarcs.count(ih)) {
          for(auto robot : cgroup->GetRobots())
            m_path.predecessors[hid].emplace(robot, std::make_pair(true, ih));
          
          break;
        }
      }
    }

    // Merge groups going into the same vertex ("merge" hyperarcs - sample 
    // trajectories for passing into/though the same vertex)
    std::cout << "incident: " << incidentHVIDs << std::endl;
    std::unordered_map<VID, VID> mergedVIDs;
    for(auto iter : incidentHVIDs) {
      auto skelVID = iter.first;
      auto predVIDs = iter.second;
      std::vector<Robot*> robots;

      if(predVIDs.size() == 1) {
        auto incomingVID = *predVIDs.begin();
        mergedVIDs[skelVID] = incomingVID;

        std::cout << "only one incoming, not merging" << std::endl;
        continue;
      }

      for(auto hid : predVIDs) {
        auto grobots = m_skeleton->GetHyperarcType(hid).edge.GetGroup()->GetRobots();
        robots.insert(robots.end(), grobots.begin(), grobots.end());
      }

      auto group = AddGroup(robots);
      auto tvertex = CompositeSkeletonVertex(group, &m_indSkeleton);
      auto cedge = CompositeSkeletonEdge(group, &m_indSkeleton);

      for(auto robot : group->GetRobots())
        tvertex.SetRobotCfg(robot, skelVID);
      
      auto tvid = m_skeleton->AddVertex(tvertex);
      auto arc = HyperskeletonArc(cedge, HyperskeletonArcType::Merge);
      auto hid = m_skeleton->AddHyperarc({tvid}, predVIDs, arc);
      m_path.mergeHyperarcs.insert(hid);
      mergedVIDs[skelVID] = tvid;
      std::cout << "merged " << predVIDs.size() << "incoming groups" << std::endl;

      // Set the predecessor map for each robot
      for(auto t : predVIDs) {
        bool foundSplit = false;
        for(auto ih : m_skeleton->GetIncomingHyperarcs(t)) {
          // First check if any split hyperarcs are incoming (this means we're
          // currently waiting)
          if(m_path.splitHyperarcs.count(ih)) {
            // We are waiting, check if we have a representative vertex here
            // if not, sample one
            if(!m_waitingReps.count(tvid)) {
              auto vertex = m_skeleton->GetVertexType(tvid);
              auto subgroup = vertex.GetGroup();
              auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(subgroup);
              auto vid = SpawnVertex(grm, vertex);
              m_waitingReps.emplace(tvid, std::make_pair(grm, vid));
            }

            // Set the successor of the incoming split to this vertex
            for(auto robot : group->GetRobots()) {
              m_path.successors[ih].emplace(robot, std::make_pair(false, tvid));
              m_path.predecessors[hid].emplace(robot, std::make_pair(false, tvid));
            }

            foundSplit = true;
            break;
          }
        }

        // If there's no split, then not waiting, proceed as usual
        if(!foundSplit) {
          for(auto ih : m_skeleton->GetIncomingHyperarcs(t)) {
            if(m_path.movementHyperarcs.count(ih) or m_path.decoupleHyperarcs.count(ih)) {
              for(auto robot : group->GetRobots()) {
                auto boolhid = m_path.predecessors.at(ih).at(robot);
                m_path.predecessors[hid].emplace(robot, boolhid);
              }
              
              break;
            }
          }
        }
      }

      // Push the intermediates on all of the incoming hyperskeleton arcs to the merge
      for(auto t : predVIDs) {
        for(auto h : m_skeleton->GetIncomingHyperarcs(t)) {
          auto& arc = m_skeleton->GetHyperarcType(h);

          if(arc.type == HyperskeletonArcType::Movement)
            arc.pushTarget = true;

          else if(arc.type == HyperskeletonArcType::Decouple) {
            // Should only have one incoming hyperarc
            for(auto tt : m_skeleton->GetHyperarc(h).tail) {
              for(auto hh : m_skeleton->GetIncomingHyperarcs(tt)) {
                auto& arc = m_skeleton->GetHyperarcType(hh);
                arc.pushTarget = true;
              }
            }
          }
        }
      }
    }

    // Split groups going down separate edges ("split" hyperarcs - sample 
    // trajectories moving away from same vertex onto different edges)
    std::unordered_map<std::pair<VID, VID>, std::set<VID>> incidentEdges;
    for(auto iter : mergedVIDs) {
      auto scvertex = m_skeleton->GetVertexType(iter.second);
      auto robots = scvertex.GetGroup()->GetRobots();

      std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>> splitGroups;
      for(auto robot : robots) {
        auto path = *_mapfSolution[robot];

        auto source = path.size() > t + 1 ? path.at(t+1) : path.at(path.size() - 1);
        auto target = path.size() > t + 2 ? path.at(t+2) : path.at(path.size() - 1);

        auto edgePair = std::make_pair(source, target);
        splitGroups[edgePair].push_back(robot);
      }

      if(splitGroups.size() == 1) {
        std::cout << "only one outgoing, not splitting" << std::endl;
        auto thid = m_hidPaths[t+1][robots[0]];

        // Check if waiting
        if(!thid.first)
          continue;

        auto svid = *(m_skeleton->GetHyperarc(thid.second).tail.begin());
        auto tvid = *(m_skeleton->GetHyperarc(thid.second).head.begin());

        auto skelStart = m_skeleton->GetVertexType(svid).GetVID(robots[0]);
        auto skelTarget = m_skeleton->GetVertexType(tvid).GetVID(robots[0]);
        auto s = std::min(skelStart, skelTarget);
        auto t = std::max(skelStart, skelTarget);

        auto edgePair = std::make_pair(s, t);
        incidentEdges[edgePair].insert(iter.second);
        continue;
      }

      std::set<HID> tvids;
      std::set<HID> pushvids;
      for(auto sg : splitGroups) {
        for(auto robot : sg.second) {
          auto thid = m_hidPaths[t+1][robot];

          auto path = *_mapfSolution[robot];
          auto source = path.size() > t + 1 ? path.at(t+1) : path.at(path.size() - 1);
          auto target = path.size() > t + 2 ? path.at(t+2) : path.at(path.size() - 1);

          // If this is waiting, get the hvid of the waiting vertex
          if(!thid.first) {
            auto wvid = thid.second;
            tvids.insert(wvid);
            auto edgePair = std::make_pair(source, target);
            incidentEdges[edgePair].insert(wvid);
            continue;
          }

          auto svid = *(m_skeleton->GetHyperarc(thid.second).tail.begin());
          tvids.insert(svid);
          pushvids.insert(svid);

          auto s = std::min(source, target);
          auto t = std::max(source, target);

          auto edgePair = std::make_pair(s, t);
          incidentEdges[edgePair].insert(svid);
        }
      }
      std::cout << "splitting outgoing groups" << std::endl;
      auto cedge = CompositeSkeletonEdge(scvertex.GetGroup(), &m_indSkeleton);
      auto arc = HyperskeletonArc(cedge, HyperskeletonArcType::Split);
      auto hid = m_skeleton->AddHyperarc(tvids, {iter.second}, arc);
      m_path.splitHyperarcs.insert(hid);

      // Push start intermediate for outgoing hyperskeleton arcs
      for(auto hd : pushvids) {
        for(auto h : m_skeleton->GetOutgoingHyperarcs(hd)) {
          auto& arc = m_skeleton->GetHyperarcType(h);
          if(arc.type == HyperskeletonArcType::Movement)
            arc.pushStart = true;

          else if(arc.type == HyperskeletonArcType::Couple) {
            // Should only have one outgoing hyperarc
            for(auto dd : m_skeleton->GetHyperarc(h).head) {
              for(auto hh : m_skeleton->GetOutgoingHyperarcs(dd)) {
                auto& arc = m_skeleton->GetHyperarcType(hh);
                arc.pushStart = true;
              }
            }
          }
        }
      }

      // Set the successor map for each robot
      // This will be overwritten for merges going back into a split
      for(auto robot : robots) {
        m_path.successors[hid][robot] = m_hidPaths[t+1][robot];
      }
    }

    // Merge together groups going opposite directions down a skeleton edge
    for(auto iter : incidentEdges) {
      if(iter.second.size() == 1) {
        std::cout << "only one group incoming to edge, not merging" << std::endl;
        continue;
      }

      std::cout << "merging together " << iter.second.size() << " opp dir groups" << std::endl;

      std::vector<Robot*> robots;
      for(auto v : iter.second) {
        auto rs = m_skeleton->GetVertexType(v).GetGroup()->GetRobots();
        robots.insert(robots.end(), rs.begin(), rs.end());
      }
      auto group = AddGroup(robots);
      auto cedge = CompositeSkeletonEdge(group, &m_indSkeleton);

      auto hid = m_hidPaths[t+1][robots[0]];
      auto tvid = *(m_skeleton->GetHyperarc(hid.second).tail.begin());
      auto arc = HyperskeletonArc(cedge, HyperskeletonArcType::Couple);
      auto mhid = m_skeleton->AddHyperarc({tvid}, iter.second, arc);
      m_path.coupleHyperarcs.insert(mhid);

      // Set the successor map for each robot in this couple arc
      for(auto robot : group->GetRobots()) {
        m_path.successors[mhid][robot] = std::make_pair(true, hid.second);
      }
    }
  }
}

bool
WoDaSH::
GroundHyperskeleton() {
  // TODO cast this to avoid having to change MPStratgy interface?
  auto s = this->GetMPLibrary()->GetMPStrategy(m_drStrategy);
  
  auto originalTask = this->GetMPLibrary()->GetGroupTask();

  for(auto hid : m_path.movementHyperarcs) {
    auto& hyperarc = m_skeleton->GetHyperarc(hid);
    auto& cedge = hyperarc.property.edge;

    // Should only have one head and tail
    auto svid = *(hyperarc.tail.begin());
    auto tvid = *(hyperarc.head.begin());

    auto svertex = m_skeleton->GetVertexType(svid);
    auto tvertex = m_skeleton->GetVertexType(tvid);

    auto pushStart = hyperarc.property.pushStart;
    auto pushTarget = hyperarc.property.pushTarget;

    cedge.SetIntermediates(ComputeIntermediates(svertex, tvertex, cedge, pushStart, pushTarget));

    // Set the task
    ///@todo Does it matter what task this is? There could be many
    auto groupTask = this->GetMPProblem()->GetTasks(cedge.GetGroup()).at(0);
    this->GetMPLibrary()->SetGroupTask(groupTask.get());

    // Check if we're waiting at a vertex before this, if so, take representative
    if(m_waitingReps.count(svid) and !pushStart) {
      hyperarc.property.startRep = m_waitingReps.at(svid);
    } else {
      auto grm = this->GetMPLibrary()->GetGroupRoadmap();
      auto startVID = SpawnVertex(grm, cedge);
      hyperarc.property.startRep = std::make_pair(grm, startVID);
    }
    
    s->GroundEdge(cedge);

    // Check if this was successful
    auto grm = this->GetMPLibrary()->GetGroupRoadmap();
    auto startVID = hyperarc.property.startRep.second;
    auto lastVID = grm->GetLastVID();
    auto lastVertex = grm->GetVertex(lastVID);
    if(!FinishedEdge(cedge, lastVertex)) {
      std::cout << "Failed to ground HID " << hid << ", replanning..." << std::endl;
      // Couldn't finish edge, need to replan hyperpath.
      auto ed = cedge.GetEdgeDescriptors()[0];
      m_failedEdges.emplace(std::make_pair(ed.source(), ed.target()), cedge.GetGroup());
      return false;
    }
    
    std::cout << "Successfully grounded HID " << hid << std::endl;
    hyperarc.property.endRep = std::make_pair(grm, lastVID);

    // Convert vids into a task and path to save in grounded hypergraph

    // Configure group task
    auto task = std::shared_ptr<GroupTask>(new GroupTask(grm->GetGroup()));
    auto startGcfg = grm->GetVertex(startVID);
    auto endGcfg = grm->GetVertex(lastVID);
    for(auto robot : grm->GetGroup()->GetRobots()) {
      MPTask t(robot);
      t.SetStartConstraint(std::move(std::unique_ptr<CSpaceConstraint>(
          new CSpaceConstraint(robot,startGcfg.GetRobotCfg(robot)))));
      t.AddGoalConstraint(std::move(std::unique_ptr<CSpaceConstraint>(
          new CSpaceConstraint(robot,endGcfg.GetRobotCfg(robot)))));
      task->AddTask(t);
    }
    this->GetMPLibrary()->GetGoalTracker()->AddMap(grm, task.get());

    // Solve for the task
    this->GetMPLibrary()->SetGroupTask(task.get());
    auto q = this->GetMPLibrary()->GetMapEvaluator(m_edgeQueryLabel);
    if(!(*q)())
      throw RunTimeException(WHERE) << "Falsely reported this edge as grounded.";

    // Save grounded edge
    auto path = this->GetMPLibrary()->GetGroupPath(grm->GetGroup());
    
    AddTransitionToGroundedHypergraph(hyperarc.tail,hyperarc.head,path,task);
  }

  this->GetMPLibrary()->SetGroupTask(originalTask);

  std::cout << "Successfully grounded " << m_path.movementHyperarcs.size() 
            << " edges, sampling trajectories..." << std::endl;

  return SampleTrajectories();
}

std::vector<typename WoDaSH::CompositeSkeletonVertex>
WoDaSH::
ComputeIntermediates(const CompositeSkeletonVertex _source, 
                     const CompositeSkeletonVertex _target,
                     CompositeSkeletonEdge _edge,
                     const bool pushStart, const bool pushTarget) {
  // Assume straight lines. Per existing merging rules, robots should only
  // be merged if they are on the same edge (going either direction), so we
  // don't need to consider different individual edge lengths

  auto robots = _edge.GetGroup()->GetRobots();
  auto& edgeDescriptors = _edge.GetEdgeDescriptors();
  
  auto robotRadius = robots[0]->GetMultiBody()->GetBoundingSphereRadius();
  auto intLength = robotRadius * m_intermediateFactor;

  std::vector<Point3d> starts;
  std::vector<Point3d> displacements;
  for(size_t i = 0; i < robots.size(); ++i) {
    auto start = m_indSkeleton.find_vertex(edgeDescriptors[i].source())->property();
    auto target = m_indSkeleton.find_vertex(edgeDescriptors[i].target())->property();

    auto disp = target - start;
    auto unitDisp = disp / disp.norm();
    if(pushStart) {
      auto push = m_regionRadius.at(robots[i]) * sqrt(2.);
      push = std::min(push, disp.norm() / 2.);
      start += unitDisp * push;
    }

    if(pushTarget) {
      auto push = m_regionRadius.at(robots[i]) * sqrt(2.);
      push = std::min(push, disp.norm() / 2.);
      target -= unitDisp * push;
    }

    starts.push_back(start);
    disp = target - start;

    if(disp.norm() < intLength)
      displacements.push_back({0., 0., 0.});
    else
      displacements.push_back(disp);
  }
  auto edgeDist = std::max(displacements[0].norm(), 1e-7);
  auto numInter = (int)ceil(edgeDist / intLength);

  // Set the intermediate values along the edge
  std::vector<CompositeSkeletonVertex> inters;
  for(int i = 0; i <= numInter; ++i){
    auto v = CompositeSkeletonVertex(_edge.GetGroup());

    for(size_t r = 0; r < robots.size(); r++) {
      Point3d d = {i * displacements[r][0]/numInter, 
                  i * displacements[r][1]/numInter,
                  i * displacements[r][2]/numInter};
      v.SetRobotCfg(r, starts[r] + d);
    }

    inters.push_back(v);
  }

  return inters;
}

size_t
WoDaSH::
SpawnVertex(GroupRoadmapType* _grm, CompositeSkeletonEdge _edge) {
  // Make the boundary at the first intermediate
  auto inter = _edge.GetIntermediates().at(0);
  auto robots = _edge.GetGroup()->GetRobots();

  std::vector<CSpaceBoundingSphere> bounds;
  for(auto robot : robots) {
    auto indV = inter.GetRobotCfg(robot);
    bounds.push_back(MakeBoundary(robot, indV));
  }

  BoundaryMap bMap;
  for(size_t i = 0; i < robots.size(); i++)
    bMap[robots[i]] = &bounds[i];
  
  // Get the sampler.
  auto s = this->GetMPLibrary()->GetSampler(m_sampler);

  std::vector<GroupCfgType> samples, collision;
  while(samples.empty()) {
    s->Sample(1, 100, bMap, std::back_inserter(samples),
      std::back_inserter(collision));
  }

  auto& target = samples.front();
  auto vid = _grm->AddVertex(target);
  return vid;
}

size_t
WoDaSH::
SpawnVertex(GroupRoadmapType* _grm, CompositeSkeletonVertex _vertex) {
  auto robots = _vertex.GetGroup()->GetRobots();

  std::vector<CSpaceBoundingSphere> bounds;
  for(auto robot : robots) {
    auto indV = _vertex.GetRobotCfg(robot);
    bounds.push_back(MakeBoundary(robot, indV));
  }

  BoundaryMap bMap;
  for(size_t i = 0; i < robots.size(); i++)
    bMap[robots[i]] = &bounds[i];
  
  // Get the sampler.
  auto s = this->GetMPLibrary()->GetSampler(m_sampler);

  std::vector<GroupCfgType> samples, collision;
  while(samples.empty()) {
    s->Sample(1, 100, bMap, std::back_inserter(samples),
      std::back_inserter(collision));
  }

  auto& target = samples.front();
  auto vid = _grm->AddVertex(target);
  return vid;
}

bool
WoDaSH::
FinishedEdge(CompositeSkeletonEdge _edge, const GroupCfgType& _groupCfg) {
  auto inter = _edge.GetIntermediates().at(_edge.GetNumIntermediates() - 1);
  auto robots = _edge.GetGroup()->GetRobots();

  bool touching = true;
  for (auto r : robots) {
    auto cfg = _groupCfg.GetRobotCfg(r);

    // Compute the penetration distance required. We want the robot's bounding
    // sphere to penetrate the region by the fraction m_penetrationThreshold.
    const double robotRadius  = cfg.GetMultiBody()->GetBoundingSphereRadius(),
                 threshold    = 2 * robotRadius * m_penetrationFactor;

    // Get the region boundary.
    auto center = inter.GetRobotCfg(r);
    auto boundary = MakeBoundary(r, center);

    // Compute the penetration distance (maximally enclosed bounding diameter).
    const Point3d robotCenter = cfg.GetPoint();
    const double penetration = boundary.GetClearance(robotCenter) + robotRadius;

    // The configuration is touching if the penetration exceeds the threshold.
    if(penetration < threshold) {
      touching = false;
      break;
    }
  }

  return touching;
}

CSpaceBoundingSphere
WoDaSH::
MakeBoundary(Robot* _robot, const Point3d _indV) {
  const bool threeD = _robot->GetMultiBody()->GetBaseType()
                   == Body::Type::Volumetric;

  // I'm not sure what the boundary code might do with a negative radius. Bound
  // it below at zero just in case.
  const double radius = std::max(0., m_regionRadius.at(_robot));

  if (threeD)
    return CSpaceBoundingSphere({_indV[0], _indV[1], _indV[2]}, radius);
  else
    return CSpaceBoundingSphere({_indV[0], _indV[1]}, radius);
}

void
WoDaSH::
ConnectToSkeleton() {
  auto group = this->GetMPLibrary()->GetGroupTask()->GetRobotGroup();

  // Connect the start position to the first skeleton vertex for each of the robots
  auto s = this->GetMPLibrary()->GetMPStrategy(m_trajStrategy);
  auto sVID = s->GenerateStart(m_sampler);
  auto start = this->GetMPLibrary()->GetGroupRoadmap()->GetVertex(sVID);

  auto stask = new GroupTask(group);
  for(auto r : group->GetRobots()) {
    auto t = MPTask(r);

    // Get the first vertex on a skeleton edge
    // TODO for now assuming that each robot must move at least once
    auto hid = m_hidPaths[0][r];
    auto outHID = hid.second;
    if(!hid.first) {
      for(auto oh : m_skeleton->GetOutgoingHyperarcs(hid.second)) {
        auto arc = m_skeleton->GetHyperarcType(oh);
        if(arc.type == HyperskeletonArcType::Movement) {
          outHID = oh;
          break;
        }
      }
    }

    auto& arc = m_skeleton->GetHyperarcType(outHID);
    auto target = arc.startRep.first->GetVertex(arc.startRep.second);

    // Add start constraints
    auto startCfg = start.GetRobotCfg(r);
    auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
    t.SetStartConstraint(std::move(startConstraint));

    // Add goal constraints
    auto goalCfg = target.GetRobotCfg(r);
    auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,goalCfg));
    t.AddGoalConstraint(std::move(goalConstraint));

    stask->AddTask(t);
  } 

  this->GetMPLibrary()->Solve(this->GetMPProblem(), stask, 
      this->GetMPSolution(), m_trajStrategy, LRand(), "ConnectToSkeleton");
  delete stask;

  // Connect the goal position to the last skeleton vertex for each of the robots
  auto gVIDs = s->GenerateGoals(m_sampler);
  auto gVID = *gVIDs.begin();
  auto goal = this->GetMPLibrary()->GetGroupRoadmap()->GetVertex(gVID);

  auto gtask = new GroupTask(group);
  for(auto r : group->GetRobots()) {
    auto t = MPTask(r);

    // Get the last vertex on a skeleton edge
    auto time = m_hidPaths.size() - 1;
    while(!m_hidPaths[time].count(r))
      time--;

    // TODO for now assuming that each robot moves at least once
    auto hid = m_hidPaths[time][r];
    auto inHID = hid.second;
    if(!hid.first) {
      for(auto ih : m_skeleton->GetIncomingHyperarcs(hid.second)) {
        auto arc = m_skeleton->GetHyperarcType(ih);
        if(arc.type == HyperskeletonArcType::Movement) {
          inHID = ih;
          break;
        }
      }
    }

    auto& arc = m_skeleton->GetHyperarcType(inHID);
    auto endSkel = arc.endRep.first->GetVertex(arc.endRep.second);

    // Add start constraints
    auto startCfg = endSkel.GetRobotCfg(r);
    auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
    t.SetStartConstraint(std::move(startConstraint));

    // Add goal constraints
    auto goalCfg = goal.GetRobotCfg(r);
    auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,goalCfg));
    t.AddGoalConstraint(std::move(goalConstraint));

    gtask->AddTask(t);
  } 

  this->GetMPLibrary()->Solve(this->GetMPProblem(), gtask, 
      this->GetMPSolution(), m_trajStrategy, LRand(), "ConnectToSkeleton");
  delete gtask;
}

bool
WoDaSH::
SampleTrajectories() {

  // TODO::Add caching for repeated queries

  // First, split apart groups that have finished moving in opposite
  // directions down the same skeleton edge. Single node paths.
  for(auto hid : m_path.decoupleHyperarcs) {
    if(this->m_debug)
      std::cout << "Constructing trajectory for opp split hid " << hid << std::endl;

    auto hyp = m_skeleton->GetHyperarc(hid);
    auto cedge = hyp.property.edge;
    auto group = cedge.GetGroup();

    auto tail = *hyp.tail.begin();
    auto ihs = *(m_skeleton->GetIncomingHyperarcs(tail).begin());
    auto inArc = m_skeleton->GetHyperarcType(ihs);

    auto start = inArc.endRep.first->GetVertex(inArc.endRep.second);
    auto target = start;

    // Plan a path between the start and target - should be empty path
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto r : group->GetRobots()) {
      auto t = MPTask(r);

      // Add start constraints
      auto startCfg = start.GetRobotCfg(r);
      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
      t.SetStartConstraint(std::move(startConstraint));

      // Add goal constraints
      auto goalCfg = target.GetRobotCfg(r);
      auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,goalCfg));
      t.AddGoalConstraint(std::move(goalConstraint));

      task->AddTask(t);
    } 

    this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
        this->GetMPSolution(), m_trajStrategy, LRand(), "SampleTrajectories");

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size())
      return false;

    // Extract path from solution
    AddTransitionToGroundedHypergraph(hyp.tail,hyp.head,path,task);
  }

  // Next, merge together groups going into the same skeleton vertex
  for(auto hid : m_path.mergeHyperarcs) {
    auto hyp = m_skeleton->GetHyperarc(hid);
    if(this->m_debug) {
      std::cout << "Sampling trajectory for merge hid " << hid << std::endl;
      std::cout << "Tail: " << hyp.tail << ", Head: " << hyp.head << std::endl;
    }

    auto cedge = hyp.property.edge;
    auto group = cedge.GetGroup();
    auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(group);

    auto start = GroupCfgType(grm);
    for(auto robot : group->GetRobots()) {
      auto inHID = m_path.predecessors.at(hid).at(robot);
      if(!inHID.first) {
        // Waiting, get representative from vertex
        auto rep = m_waitingReps.at(inHID.second);
        auto vertex = rep.first->GetVertex(rep.second);
        start.SetRobotCfg(robot, vertex.GetVID(robot));
      } else {
        auto& inArc = m_skeleton->GetHyperarcType(inHID.second);
        auto vertex = inArc.endRep.first->GetVertex(inArc.endRep.second);
        start.SetRobotCfg(robot, vertex.GetVID(robot));
      }
    }

    // Check if there is a split afterward or if all robots go to the same edge
    GroupCfgType target;
    auto head = *hyp.head.begin();
    for(auto oh : m_skeleton->GetOutgoingHyperarcs(head)) {
      auto& outArc = m_skeleton->GetHyperarcType(oh);
      if(outArc.type == HyperskeletonArcType::Movement and 
                                           m_path.movementHyperarcs.count(oh)) {
        target = outArc.startRep.first->GetVertex(outArc.startRep.second);
        break;
      } else if(outArc.type == HyperskeletonArcType::Split and 
                                              m_path.splitHyperarcs.count(oh)) {
        auto vid = SpawnVertex(grm, m_skeleton->GetVertexType(head));
        target = grm->GetVertex(vid);
        break;
      } else if(outArc.type == HyperskeletonArcType::Couple and 
                                             m_path.coupleHyperarcs.count(oh)) {
        auto firstRobot = group->GetRobots().at(0);
        auto outerHID = m_path.successors.at(oh).at(firstRobot);

        if(!outerHID.first)
          throw RunTimeException(WHERE) << "Found a waiting vertex coming into couple.";

        auto outerArc = m_skeleton->GetHyperarcType(outerHID.second);
        auto vertex = outerArc.startRep.first->GetVertex(outerArc.startRep.second);

        target = GroupCfgType(grm);
        for(auto robot : group->GetRobots()) {
          auto cfg = vertex.GetRobotCfg(robot);
          target.SetRobotCfg(robot, std::move(cfg));
        }
        break;
      }
    }
    
    // Add end representative in case of splitting
    auto endVID = grm->AddVertex(target);
    hyp.property.endRep = std::make_pair(grm, endVID);

    // Plan a path between the start and target
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto r : group->GetRobots()) {
      auto t = MPTask(r);

      // Add start constraints
      auto startCfg = start.GetRobotCfg(r);
      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
      t.SetStartConstraint(std::move(startConstraint));

      // Add goal constraints
      auto goalCfg = target.GetRobotCfg(r);
      auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,goalCfg));
      t.AddGoalConstraint(std::move(goalConstraint));

      std::cout << "Robot: " << r->GetLabel() << std::endl;
      std::cout << "Start: " << startCfg << std::endl;
      std::cout << "Goal: " << goalCfg << std::endl;

      task->AddTask(t);
    } 

    // Set the boundary to be a tighter region to avoid growing tree from 
    // other nodes in the roadmap
    // auto origBounds = this->GetMPProblem()->GetEnvironment()->GetBoundary()->Clone();
    // auto dim = origBounds->GetDimension();
    // auto newBounds = std::unique_ptr<Boundary>(new WorkspaceBoundingSphere(dim));
    // this->GetMPProblem()->GetEnvironment()->SetBoundary(newBounds);

    this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
        this->GetMPSolution(), m_trajStrategy, LRand(),
        this->GetNameAndLabel()+"::SampleTrajectories");

    // this->GetMPProblem()->GetEnvironment()->SetBoundary(std::move(origBounds));

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size())
      return false;

    AddTransitionToGroundedHypergraph(hyp.tail,hyp.head,path,task);
  }

  // Next, split apart groups going onto different skeleton edges
  for(auto hid : m_path.splitHyperarcs) {
    if(this->m_debug)
      std::cout << "Sampling trajectory for split hid " << hid << std::endl;

    auto hyp = m_skeleton->GetHyperarc(hid);
    auto cedge = hyp.property.edge;
    auto group = cedge.GetGroup();
    auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(group);

    auto tail = *hyp.tail.begin();
    auto ihs = *(m_skeleton->GetIncomingHyperarcs(tail).begin());
    auto& arc = m_skeleton->GetHyperarcType(ihs);
    auto start = arc.endRep.first->GetVertex(arc.endRep.second);

    auto target = GroupCfgType(grm);
    for(auto robot : group->GetRobots()) {
      auto outHID = m_path.successors.at(hid).at(robot);
      if(!outHID.first) {
        // Going back into a waiting vertex
        auto rep = m_waitingReps.at(outHID.second);
        auto vertex = rep.first->GetVertex(rep.second);
        target.SetRobotCfg(robot, vertex.GetVID(robot));
      } else {
        auto& outArc = m_skeleton->GetHyperarcType(outHID.second);
        auto vertex = outArc.startRep.first->GetVertex(outArc.startRep.second);
        target.SetRobotCfg(robot, vertex.GetVID(robot));
      }
    }

    // Plan a path between the start and target
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto r : group->GetRobots()) {
      auto t = MPTask(r);

      // Add start constraints
      auto startCfg = start.GetRobotCfg(r);
      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
      t.SetStartConstraint(std::move(startConstraint));

      // Add goal constraints
      auto goalCfg = target.GetRobotCfg(r);
      auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,goalCfg));
      t.AddGoalConstraint(std::move(goalConstraint));

      task->AddTask(t);
    } 

    this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
        this->GetMPSolution(), m_trajStrategy, LRand(), "SampleTrajectories");

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size())
      return false;

    AddTransitionToGroundedHypergraph(hyp.tail,hyp.head,path,task);
  }

  // Finally, merge together groups that are going in the opposite direction
  // along the same edge. Single node paths.
  for(auto hid : m_path.coupleHyperarcs) {
    if(this->m_debug)
      std::cout << "Constructing trajectory for opp mereg hid " << hid << std::endl;

    auto hyp = m_skeleton->GetHyperarc(hid);
    auto cedge = hyp.property.edge;
    auto group = cedge.GetGroup();

    auto head = *hyp.head.begin();
    auto ohs = *(m_skeleton->GetOutgoingHyperarcs(head).begin());
    auto outArc = m_skeleton->GetHyperarcType(ohs);

    auto start = outArc.startRep.first->GetVertex(outArc.startRep.second);
    auto target = start;

    // Plan a path between the start and target - should be empty path
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto r : group->GetRobots()) {
      auto t = MPTask(r);

      // Add start constraints
      auto startCfg = start.GetRobotCfg(r);
      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
      t.SetStartConstraint(std::move(startConstraint));

      // Add goal constraints
      auto goalCfg = target.GetRobotCfg(r);
      auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,goalCfg));
      t.AddGoalConstraint(std::move(goalConstraint));

      task->AddTask(t);
    } 

    this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
        this->GetMPSolution(), m_trajStrategy, LRand(), "SampleTrajectories");

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size())
      return false;

    // Extract path from solution
    AddTransitionToGroundedHypergraph(hyp.tail,hyp.head,path,task);
  }

  return true;
}

RobotGroup*
WoDaSH::
AddGroup(std::vector<Robot*> _robots) {
  auto group = this->GetMPProblem()->AddRobotGroup(_robots, "");
  this->GetMPLibrary()->GetMPSolution()->AddRobotGroup(group);

  if(this->GetMPProblem()->GetTasks(group).size() < 1) {
    std::unique_ptr<GroupTask> gt = std::unique_ptr<GroupTask>(new GroupTask(group));
    for (auto robot : _robots)
      gt->AddTask(*m_taskMap[robot]);
    this->GetMPProblem()->AddTask(std::move(gt));
  }

  return group;
}

WoDaSH::HID
WoDaSH::
AddTransitionToGroundedHypergraph(std::set<VID> _tail, std::set<VID> _head,
  GroupPathType* _path, std::shared_ptr<GroupTask> _task) {

  if(this->m_debug) {
    std::cout << "Adding grounded hyperarc between " << _tail << " and " << _head << std::endl;
  }

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_groundedHypergraphLabel).get());

  std::set<VID> groundedTail;
  std::set<VID> groundedHead;

  auto hid = m_skeleton->GetHID(_head, _tail);
  auto arc = m_skeleton->GetHyperarcType(hid);

  if(arc.type == HyperskeletonArcType::Movement) {
    if(this->m_debug)
      std::cout << "Adding hyperskeleton edge to grounded hypergraph" << std::endl;

    auto vertex = arc.startRep;
    auto gVID = gh->AddVertex(vertex);
    groundedTail.insert(gVID);

    vertex = arc.endRep;
    gVID = gh->AddVertex(vertex);
    groundedHead.insert(gVID);
  }
  else {
    if(this->m_debug)
      std::cout << "Adding composition arc to grounded hypergraph" << std::endl;

    auto grm = _path->GetRoadmap();
    auto vid = _path->VIDs().front();
    auto fullTail = grm->GetVertex(vid);

    // Split the vertex at the front of the path to get a grounded tail set
    for(auto t : _tail) {
      auto compState = m_skeleton->GetVertexType(t);
      auto group = compState.GetGroup();
      auto subgrm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(group);

      auto subgcfg = GroupCfgType(subgrm);
      for(auto robot : group->GetRobots())
        subgcfg.SetRobotCfg(robot, fullTail.GetVID(robot));
      
      auto subvid = subgrm->AddVertex(subgcfg);
      auto vertex = std::make_pair(subgrm, subvid);
      auto gVID = gh->AddVertex(vertex);
      groundedTail.insert(gVID);
    }

    vid = _path->VIDs().back();
    auto fullHead = grm->GetVertex(vid);

    // Split the vertex at the back of the path to get a grounded head set
    for(auto h : _head) {
      auto compState = m_skeleton->GetVertexType(h);
      auto group = compState.GetGroup();
      auto subgrm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(group);

      auto subgcfg = GroupCfgType(subgrm);
      for(auto robot : group->GetRobots())
        subgcfg.SetRobotCfg(robot, fullHead.GetVID(robot));
      
      auto subvid = subgrm->AddVertex(subgcfg);
      auto vertex = std::make_pair(subgrm, subvid);
      auto gVID = gh->AddVertex(vertex);
      groundedHead.insert(gVID);
    }
  }

  // Add hyperarc
  GroundedHypergraph::Transition transition;
  transition.taskSet = {{_task}};
  // Decide if cost is length or timesteps
  auto pathCost = _path->TimeSteps();
  auto pathStart = _path->VIDs().front();
  auto pathEnd = _path->VIDs().back();
  transition.compositeImplicitPath = std::make_pair(pathStart,pathEnd);
  transition.cost = pathCost;
  return gh->AddTransition(groundedTail,groundedHead,transition);
}
/*------------------------------------------------------------*/
