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
#include "TMPLibrary/TaskEvaluators/SubmodeQuery.h"

#include "MPLibrary/MPStrategies/CDRRRTLite.h"
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

  m_mapf = _node.Read("mapf", false, "cbs", "MAPF method (cbs or pbs)");

  m_replanMethod = _node.Read("replanMethod", false, "global", 
          "How to get the next best MAPF solution (global, resume, or lazy");

  m_skeletonType = _node.Read("skeletonType", true, "",
      "the type of skeleton to use, Available options are reeb and mcs "
      "for 3d, ma for 2d");

  m_skeletonIO = _node.Read("skeletonIO", false, "", "read of write the "
      "skeleton file");

  m_skeletonFilename = _node.Read("skeletonFile", m_skeletonIO != "", "",
      "the skeleton file to read from or write to");

  m_split = _node.Read("split", false, 0., 0., std::numeric_limits<double>::max(), 
      "Split the skeleton into equal pieces?");
  
  m_intermediateFactor = _node.Read("intermediateFactor", false, 
      m_intermediateFactor, 1., std::numeric_limits<double>::max(), 
      "Multiple of robot radius between skeleton edge intermediates");

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

  m_clearance = _node.Read("clearance", false, true, "Use edge clearance in validity?");
}

WoDaSH::
~WoDaSH() {}

/*------------------------ Interface -------------------------*/

void
WoDaSH::
Initialize() {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Initialize");

  {
    auto plan = this->GetPlan();
    auto decomp = plan->GetDecomposition();
    auto st = decomp->GetGroupMotionTasks()[0];
    auto groupTask = st->GetGroupMotionTask();
    this->GetMPLibrary()->SetGroupTask(groupTask.get());
    this->GetMPLibrary()->SetMPSolution(this->GetMPSolution());
    m_wholeGroup = groupTask->GetRobotGroup();
    m_wholeTask = groupTask.get();
    m_groupTaskMap[m_wholeGroup] = m_wholeTask;
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
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::PlanTasks");

  auto me = dynamic_cast<ScheduledCBS*>(this->GetTaskEvaluator(m_motionEvaluator).get());
  me->Initialize();

  // Get start and goal in the grounded hypergraph
  GroundStartAndGoal();

  auto plan = this->GetPlan();

  do {
    bool success = false;
    while(!success) {
      // Reset the task, maybe this does something idk
      this->GetMPLibrary()->SetGroupTask(m_wholeTask);

      auto mapfSol = MAPFSolution();
      ConstructHyperpath(mapfSol);
      success = GroundHyperskeleton();
      success = success and ConnectToSkeleton(); // TODO impose vertex constraints if this fails
    }
    if(this->m_debug)
      std::cout << "successfully found a path for each robot." << std::endl;

    ConvertToPlan(m_path.groundedSolution);
    this->GetMPLibrary()->SetGroupTask(m_wholeTask);
    
  } while(!me->operator()(plan));
}

/*--------------------- Helper Functions ---------------------*/

void
WoDaSH::
BuildSkeleton() {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::BuildSkeleton");

  // Get robots.
  auto robots = this->GetMPLibrary()->GetGroupTask()->GetRobotGroup()->GetRobots();

  // Determine if we need a 2d or 3d skeleton.
  auto env = this->GetMPProblem()->GetEnvironment();
  const bool threeD = robots[0]->GetMultiBody()->GetBaseType() ==
      Body::Type::Volumetric;


  if(m_skeletonIO == "read") {
    m_indSkeleton.Read(m_skeletonFilename);
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
    }
  }

  if(m_split > 0) {
    double maxLength = m_split * robots[0]->GetMultiBody()->GetBoundingSphereRadius();
    m_indSkeleton.RefineEdges(maxLength);
    // m_individualSkeleton.Write(this->GetBaseFilename() + ".split.graph");
  }
  m_indSkeleton.DoubleEdges();

  // Add self edges for waiting
  for(auto vid : m_indSkeleton.GetAllVIDs()) {
    std::vector<Point3d> p = {};
    m_indSkeleton.AddEdge(vid, vid, p);
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
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::InitializeCostToGo");

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
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  size_t maxTimestep = 0;
  for(auto kv : _node.solutionMap) {
    maxTimestep = std::max(maxTimestep,kv.second->size());
  }

  std::unordered_map<VID, std::unordered_map<size_t, double>> vertexCapacity;
  std::unordered_map<std::pair<VID, VID>,
      std::unordered_map<size_t, double>> edgeCapacity;

  // new constraints of form (robot, ((vid, vid), time))
  std::vector<std::pair<Robot*, CBSConstraint>> constraints;

  // Check the global constraints from edges that failed to ground
  std::unordered_map<size_t, std::unordered_map<size_t, size_t>> edgeOccupancy;

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

      // order such that source < target
      VID sVID;
      VID tVID;
      if(source < target) {
        sVID = source;
        tVID = target;
      } else {
        sVID = target;
        tVID = source;
      }

      // if waiting, get the last different vertex
      // auto preVID = SIZE_MAX;
      // if(tVID == sVID) {
      //   int pt = std::min(s - 1, path.size()-1);
      //   while (pt >= 0 and path.at(pt) == source) {
      //     pt -= 1;
      //   }
      //   if(path.at(pt) != source) {
      //     preVID = path.at(pt);
      //     auto pVID = std::min(preVID, source);
      //     auto qVID = std::max(preVID, source);

      //     // Check if this robot is waiting in a failed group
      //     for(size_t j = 0; j < m_failedEdges.size(); j++) {
      //       if(m_failedEdges[j].count(robot)) {
      //         if(m_failedEdges[j][robot].first == pVID and 
      //               m_failedEdges[j][robot].second == qVID) {
      //           if(edgeOccupancy.count(i) and edgeOccupancy[i].count(j))
      //             edgeOccupancy[i][j]++;
      //           else
      //             edgeOccupancy[i][j] = 1;
      //         }
      //       }
      //     }
      //   }
      // }

      // Check if this robot is in a failed group
      for(size_t j = 0; j < m_failedEdges.size(); j++) {
        if(m_failedEdges[j].count(robot)) {
          if(m_failedEdges[j][robot].first == sVID and 
                m_failedEdges[j][robot].second == tVID) {
            if(edgeOccupancy.count(i) and edgeOccupancy[i].count(j))
              edgeOccupancy[i][j]++;
            else
              edgeOccupancy[i][j] = 1;
          }
        }
      }

      // Get the edge in the individual skeleton
      if(source != target) {
      
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
    std::unordered_set<Robot*> addedEdgeConstraint;
    std::unordered_set<Robot*> addedVertexConstraint;

    for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

      // Find the VID of this robot at time i
      auto robot = iter->first;
      auto path = *(iter->second);

      auto s = std::min(i, path.size()-1);
      auto source = path[s];
      auto t = std::min(i+1, path.size()-1);
      auto target = path[t];

      // order such that source < target
      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);
      auto edgePair = std::make_pair(sVID, tVID);

      // Check the capacity of the source vertex
      if(vertexCapacity.at(source).at(i) < 0 and m_clearance) {
        auto vPair = std::make_pair(source, SIZE_MAX);
        constraints.push_back(std::make_pair(robot, std::make_pair(vPair, i)));
        addedVertexConstraint.insert(robot);
      }

      // Check for an edge
      if(source != target and m_clearance) {
        // Check the capacity of the edge
        if(edgeCapacity.at(edgePair).at(i) < 0) {
          constraints.push_back(std::make_pair(robot, std::make_pair(edgePair, i)));
          addedEdgeConstraint.insert(robot);
        }
      }
    
      if(!edgeOccupancy.count(i))
        continue;

      for(auto ed : edgeOccupancy.at(i)) {
        if(ed.second == m_failedEdges.at(ed.first).size() and m_failedEdges[ed.first].count(robot)) {
          auto edgePair = m_failedEdges[ed.first][robot];
          auto source = edgePair.first;
          auto target = edgePair.second;

          // Add vertex constraint if waiting on failed edge
          if(source == target and !addedVertexConstraint.count(robot)) {
            auto vPair = std::make_pair(source, SIZE_MAX);
            constraints.push_back(std::make_pair(robot, std::make_pair(vPair, i)));
          }

          // Otherwise add edge constraint
          if(source != target and !addedEdgeConstraint.count(robot)) {
            constraints.push_back(std::make_pair(robot, std::make_pair(edgePair, i)));
          }
        }
      }
    }
  }

  return constraints;
}


std::vector<typename WoDaSH::CBSNodeType> 
WoDaSH::
SplitNodeFunction(CBSNodeType& _node,
        std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
        CBSLowLevelPlanner<Robot*,CBSConstraint,CBSSolution*>& _lowLevel,
        CBSCostFunction<Robot*,CBSConstraint,CBSSolution*>& _cost) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SplitNodeFunction");

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
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");

  auto constraints = _node.constraintMap[_robot];
  auto h = std::shared_ptr<HeuristicSearch>(new HeuristicSearch(_robot));

  // Get the start and goal vids in the individual skeleton
  auto start = m_skeletonStarts.at(_robot);
  auto goal = m_skeletonGoals.at(_robot);

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
          if(timestep == constraint.second) {
            return std::numeric_limits<double>::infinity();
          }
        }

        if(source == constraint.first.first and target == constraint.first.second) {
          if(timestep == constraint.second) {
            return std::numeric_limits<double>::infinity();
          }
        }

        // Ignore any other edges
        if(constraint.first.second != SIZE_MAX)
          continue;

        // Check for a conflict at the source
        if(source == constraint.first.first and timestep == constraint.second) {
          return std::numeric_limits<double>::infinity();
        }

        // Check for a conflict at the target
        if(target == constraint.first.first and timestep+1 == constraint.second) {
          return std::numeric_limits<double>::infinity();
        }
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

        // Edge weights assume straight line edges
        auto targetVI = g->find_vertex(target);
        auto dist = (targetVI->property() - vit->property()).norm();

        auto nvid = _h->AddVertex(neighbor);
        _h->AddEdge(_vid,nvid,1.0+dist);
      }
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


template <typename NodeType>
double
WoDaSH::
CostFunction(NodeType& _node) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CostFunction");

  // For now, treat an edge as having cost 1, use makespan
  double cost = 0;
  for(auto kv : _node.solutionMap) {
    cost = std::max(cost,double(kv.second->size()));
  }

  return cost;
}

std::vector<typename WoDaSH::PBSNodeType> 
WoDaSH::
SplitNodeFunction(PBSNodeType& _node,
        std::vector<std::pair<Robot*,OrderingConstraint>> _constraints,
        CBSLowLevelPlanner<Robot*, OrderingConstraint, CBSSolution*>& _lowLevel,
        CBSCostFunction<Robot*, OrderingConstraint, CBSSolution*>& _cost) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SplitNodeFunction");

  std::vector<PBSNodeType> children;

  for(auto constraintPair : _constraints) {
    auto robot = constraintPair.first;
    auto constraint = constraintPair.second;
    auto child = _node;

    // Double check that constraints don't make a circular dependency
    if(CheckCircularDependency(robot,constraint,child.constraintMap))
      continue;
    
    child.constraintMap[robot].insert(constraint);

    if(!_lowLevel(child, robot))
      continue;

    child.cost = _cost(child);
    children.push_back(child);
  }

  return children;
}

bool
WoDaSH::
CheckCircularDependency(Robot* _robot, const OrderingConstraint& _newConstraint, 
                const std::map<Robot*,std::set<OrderingConstraint>>& _constraints) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CheckCircularDependency");

  // Check for duplicate constraints
  if(_constraints.at(_robot).count(_newConstraint))
    return true;

  // Grow tree from new constraint and see if see if a cycle is formed
  std::queue<OrderingConstraint> queue;
  queue.push(_newConstraint);

  while(!queue.empty()) {
    auto r = queue.front();
    queue.pop();
    auto constraints = _constraints.at(r);

    for(auto c : constraints) {
      if(c == _robot)
        return true;

      queue.push(c);
    }
  }

  return false;
}

bool
WoDaSH::
LowLevelPlanner(PBSNodeType& _node, Robot* _robot) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");

  std::set<Robot*> needsToReplan,hasReplanned;
  needsToReplan.insert(_robot);
  std::vector<VID>* pathCopy = nullptr;

  AddDependencies(needsToReplan,_robot,_node);

  Robot* robot = nullptr;
  while(hasReplanned.size() != needsToReplan.size()) {

    for(auto r : needsToReplan) {
      if(hasReplanned.count(r))
        continue;

      bool ready = true;
      for(auto c : _node.constraintMap.at(r)) {
        if(!needsToReplan.count(c) or hasReplanned.count(c))
          continue;

        ready = false;
        break;
      }

      if(!ready)
        continue;

      robot = r;
      break;
    }


    auto h = std::shared_ptr<HeuristicSearch>(new HeuristicSearch(robot));

    // Get the start and goal vids in the individual skeleton
    auto start = m_skeletonStarts.at(_robot);
    auto goal = m_skeletonGoals.at(_robot);

    // Check that start does not violate a constraint and get min end time
    size_t minEndTimestep = 0;
    for(auto kv : _node.solutionMap) {
      auto path = kv.second;
      if(path)
        minEndTimestep = std::max(minEndTimestep, path->size() - 1);
    }

    // Distance from each skeleton vertex to the goal
    auto dist2go = m_distanceMap.at(robot);
    auto g = &m_indSkeleton;
    auto annot = m_annotationMap.at(robot);

    auto startVertex = std::make_pair(start,0);
    auto startVID = h->AddVertex(startVertex);

    auto f = m_failedEdges;

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
      [_node,robot,annot,f,g,h](typename HeuristicSearch::adj_edge_iterator& _ei,
        const double _sourceDistance,
        const double _targetDistance) {
      
        auto source = h->GetVertex(_ei->source()).first;
        auto target = h->GetVertex(_ei->target()).first;
        auto timestep = h->GetVertex(_ei->source()).second;

        double edgeUsage = robot->GetMultiBody()->GetBoundingSphereRadius();
        double sourceUsage = robot->GetMultiBody()->GetBoundingSphereRadius();
        double targetUsage = robot->GetMultiBody()->GetBoundingSphereRadius();

        // Preserve transitivity by adding dependencies of dependencies
        auto constraints = _node.constraintMap.at(robot);
        std::queue<OrderingConstraint> addConst;
        for(auto c : constraints)
          addConst.push(c);

        while(!addConst.empty()) {
          auto c = addConst.front();
          for(auto d : _node.constraintMap.at(c)) {
            if(!constraints.count(d)) {
              constraints.insert(d);
              addConst.push(d);
            }
          }
          addConst.pop();
        }

        for(auto r : constraints) {
          auto path = *_node.solutionMap.at(r);

          auto time = timestep >= path.size() ? path.size() - 1 : timestep;
          auto nextTime = timestep + 1 >= path.size() ? path.size() - 1 : timestep + 1;
          auto ps = path.at(time);
          auto pt = path.at(nextTime);

          // Check the capacity at the start vertex and target vertex
          if(ps == source)
            sourceUsage += r->GetMultiBody()->GetBoundingSphereRadius();

          if(pt == target)
            targetUsage += r->GetMultiBody()->GetBoundingSphereRadius();

          // Check the capacity on the edge
          if(ps == pt or source == target)
            continue;

          if((source == ps and target == pt) or (source == pt and target == ps))
            edgeUsage += r->GetMultiBody()->GetBoundingSphereRadius();
        }

        if(sourceUsage > annot->GetVertexProperty(source))
          return std::numeric_limits<double>::infinity();
        
        if(targetUsage > annot->GetVertexProperty(target))
          return std::numeric_limits<double>::infinity();
        
        if(source != target) {
          WorkspaceSkeleton::adj_edge_iterator ei;
          g->GetEdge(source, target, ei);
          auto eid = ei->descriptor();

          auto dists = annot->GetEdgeProperty(eid);
          auto minDist = *std::min_element(dists.begin(), dists.end());
          
          if(edgeUsage > minDist)
            return std::numeric_limits<double>::infinity();
        }

        // Check that not all of the robots are trying to traverse a failed edge
        for(auto ed : f) {
          if(!ed.count(robot))
            continue;

          if(ed[robot].first != source)
              continue;
          if(ed[robot].second != target)
              continue;

          size_t matchedConstraint = 1;

          for(auto kv : _node.solutionMap) {
            auto r = kv.first;

            if(r == robot or kv.second == nullptr or !ed.count(r))
              continue;

            auto path = *kv.second;
            auto s = std::min(timestep, path.size()-1);
            auto rsource = path[s];
            auto t = std::min(timestep+1, path.size()-1);
            auto rtarget = path[t];

            if(ed[r].first != rsource)
              continue;
            if(ed[r].second != rtarget)
              continue;

            matchedConstraint++;
          }

          if(matchedConstraint == ed.size())
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
        auto sourceProp = vit->property();

        for(auto eit = vit->begin(); eit != vit->end(); eit++) {
          auto target = eit->target();
          auto targetProp = g->find_vertex(target)->property();
          auto neighbor = std::make_pair(target,timestep+1);

          // Add a small cost for waiting so no edge has 0 weight
          auto nvid = _h->AddVertex(neighbor);
          auto dist = (targetProp - sourceProp).norm();
          _h->AddEdge(_vid,nvid,1.0+dist);
        }
      }
    );

    std::vector<size_t> starts = {startVID};
    std::vector<size_t> goals = {goal};

    auto sssp = AStarSSSP(h.get(),starts,goals,weight,heuristic,neighbors,termination);

    // Check that a path was found
    const size_t last = sssp.ordering.back();
    if(h->GetVertex(last).first != goal or h->GetVertex(last).second < minEndTimestep) {
      if(this->m_debug) {
        std::cout << "Failed to find a path for " << robot->GetLabel() << std::endl;
      }
      pathCopy = nullptr;
      break;
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
    _node.solutionMap[robot] = new vector<size_t>();
    *(_node.solutionMap[robot]) = path;
    pathCopy = _node.solutionMap[robot];

    hasReplanned.insert(robot);
  }

  return pathCopy != nullptr and pathCopy->size();
}

void
WoDaSH::
AddDependencies(std::set<Robot*>& _needsToReplan, Robot* _robot, const PBSNodeType& _node) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::AddDependencies");

  for(auto iter : _node.constraintMap) {
     auto r = iter.first;
     auto dep = iter.second;
     if(dep.count(_robot)) {
      if(!_needsToReplan.count(r)) {
        _needsToReplan.insert(r);
        AddDependencies(_needsToReplan, r, _node);
      }
     }
  }
}

std::vector<std::pair<Robot*, typename WoDaSH::OrderingConstraint>>
WoDaSH::
ValidationFunction(PBSNodeType& _node) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  size_t maxTimestep = 0;
  for(auto kv : _node.solutionMap) {
    maxTimestep = std::max(maxTimestep,kv.second->size());
  }

  std::unordered_map<VID, std::unordered_map<size_t, double>> vertexCapacity;
  std::unordered_map<std::pair<VID, VID>, 
      std::unordered_map<size_t, double>> edgeCapacity;

  // new constraints of form (robot, robot)
  std::vector<std::pair<Robot*, OrderingConstraint>> constraints;

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
      
      // Check if this source vid has been added yet
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

  // Form a constraint for every robot, vid that has below 0 capacity remaining
  for(size_t i = 0; i < maxTimestep; i++) {

    VID conflictSource = INVALID_VID;
    VID conflictTarget = INVALID_VID;
    for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

      // Find the VID of this robot at time i
      auto path = *(iter->second);

      auto s = std::min(i, path.size()-1);
      auto source = path[s];
      auto t = std::min(i+1, path.size()-1);
      auto target = path[t];

      // order such that source < target
      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);
      auto edgePair = std::make_pair(sVID, tVID);

      // Check the capacity of the source vertex
      if(vertexCapacity.at(source).at(i) < 0 and m_clearance) {
        conflictSource = source;
        break;
      }

      // Check for an edge
      else if(source != target and m_clearance) {
        // Check the capacity of the vertex
        if(edgeCapacity.at(edgePair).at(i) < 0) {
          conflictSource = sVID;
          conflictTarget = tVID;
          break;
        }
      }
    }

    std::vector<Robot*> robots;
    if(conflictSource != INVALID_VID and conflictTarget == INVALID_VID) {
      for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

        // Find the VID of this robot at time i
        auto robot = iter->first;
        auto path = *(iter->second);

        auto s = std::min(i, path.size()-1);
        auto source = path[s];

        if(source == conflictSource)
          robots.push_back(robot);
      }
    }

    else if(conflictSource != INVALID_VID and conflictTarget != INVALID_VID) {
      for(auto iter = _node.solutionMap.begin(); iter != _node.solutionMap.end(); iter++) {

        // Find the VID of this robot at time i
        auto robot = iter->first;
        auto path = *(iter->second);

        auto s = std::min(i, path.size()-1);
        auto source = path[s];
        auto t = std::min(i+1, path.size()-1);
        auto target = path[t];

        // order such that source < target
        auto sVID = std::min(source, target);
        auto tVID = std::max(source, target);

        if(sVID == conflictSource and tVID == conflictTarget)
          robots.push_back(robot);
      }
    }

    if(robots.size()) {
      for(size_t k = 0; k < robots.size(); k++) {
        for(size_t l = k+1; l < robots.size(); l++) {
          constraints.push_back(std::make_pair(robots.at(k), robots.at(l)));
          constraints.push_back(std::make_pair(robots.at(l), robots.at(k)));
        }
      }
      break;
    }
  }

  return constraints;
}

std::unordered_map<Robot*, typename WoDaSH::CBSSolution*>
WoDaSH::
MAPFSolution() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::MAPFSolution");

  std::unordered_map<Robot*, CBSSolution*> solution;
  auto robots = this->GetMPLibrary()->GetGroupTask()->GetRobotGroup()->GetRobots();

  // Configure CBS/PBS Functions
  if(m_mapf == "cbs") {
    CBSLowLevelPlanner<Robot*,CBSConstraint,CBSSolution*> lowLevel(
      [this](CBSNodeType& _node, Robot* _task) {
        return this->LowLevelPlanner(_node,_task);
      }
    );

    CBSValidationFunction<Robot*,CBSConstraint,CBSSolution*> validation(
      [this](CBSNodeType& _node) {
        return this->ValidationFunction(_node);
      }
    );

    CBSCostFunction<Robot*,CBSConstraint,CBSSolution*> cost(
      [this](CBSNodeType& _node) {
        return this->CostFunction<CBSNodeType>(_node);
      }
    );

    CBSSplitNodeFunction<Robot*,CBSConstraint,CBSSolution*> splitNode(
      [this](CBSNodeType& _node, std::vector<std::pair<Robot*,CBSConstraint>> _constraints,
            CBSLowLevelPlanner<Robot*,CBSConstraint,CBSSolution*>& _lowLevel,
            CBSCostFunction<Robot*,CBSConstraint,CBSSolution*>& _cost) {
        return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
      }
    );

    auto robots = this->GetMPLibrary()->GetGroupTask()->GetRobotGroup()->GetRobots();
    CBSNodeType sol = CBS(robots,validation,splitNode,lowLevel,cost);
    solution = sol.solutionMap;

  } else if(m_mapf == "pbs") {
    CBSLowLevelPlanner<Robot*, OrderingConstraint, CBSSolution*> lowLevel(
      [this](PBSNodeType& _node, Robot* _task) {
        return this->LowLevelPlanner(_node,_task);
      }
    );

    CBSValidationFunction<Robot*, OrderingConstraint, CBSSolution*> validation(
      [this](PBSNodeType& _node) {
        return this->ValidationFunction(_node);
      }
    );

    CBSSplitNodeFunction<Robot*, OrderingConstraint, CBSSolution*> splitNode(
      [this](PBSNodeType& _node, std::vector<std::pair<Robot*,OrderingConstraint>> _constraints,
            CBSLowLevelPlanner<Robot*, OrderingConstraint, CBSSolution*>& _lowLevel,
            CBSCostFunction<Robot*, OrderingConstraint, CBSSolution*>& _cost) {
        return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
      }
    );

    CBSCostFunction<Robot*, OrderingConstraint, CBSSolution*> cost(
      [this](PBSNodeType& _node) {
        return this->CostFunction<PBSNodeType>(_node);
      }
    );

    auto sol = CBS(robots,validation,splitNode,lowLevel,cost);
    solution = sol.solutionMap;
  } else {
    throw RunTimeException(WHERE) << "Unknown MAPF method. Options are cbs and pbs.";
  }

  if(this->m_debug) {
    std::cout << "Heuristic Paths" << std::endl;
    for(auto kv : solution) {
      auto robot = kv.first;
      auto path = *kv.second;
      std::cout << "\t" << robot->GetLabel() << ": ";
      for(auto vid : path) {
        std::cout << vid << ", ";
      }
      std::cout << std::endl;
    }
  }

  return solution;
}


void
WoDaSH::
GroundStartAndGoal() {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GroundStartAndGoal");

  // this->GetMPSolution()->AddRobotGroup(m_wholeTask->GetRobotGroup());
  // this->GetMPLibrary()->SetGroupTask(m_wholeTask);
  // auto wholeGRM = this->GetMPLibrary()->GetGroupRoadmap();

  auto gh = dynamic_cast<GroundedHypergraph*>(
    this->GetStateGraph(m_groundedHypergraphLabel).get());

  // Get the virtual source and sink
  // GroundedHypergraph::Transition transition;
  // transition.cost = 0;
  m_virtualSource = gh->AddVertex(std::make_pair(nullptr, 0));
  m_virtualSink = gh->AddVertex(std::make_pair(nullptr, 1));

  // Get the start position for all robots
  // auto s = this->GetMPLibrary()->GetMPStrategy(m_trajStrategy);
  // auto sVID = s->GenerateStart(m_sampler);
  // m_groundedStartVID = gh->AddVertex(std::make_pair(wholeGRM, sVID));
  // gh->AddTransition({vsource}, {m_groundedStartVID}, transition);

  // // Get the goal position for all robots
  // auto gVIDs = s->GenerateGoals(m_sampler);
  // auto gVID = *gVIDs.begin();
  // m_groundedGoalVID = gh->AddVertex(std::make_pair(wholeGRM, gVID));
  // gh->AddTransition({m_groundedGoalVID}, {vsink}, transition);
}


void
WoDaSH::
ConstructHyperpath(std::unordered_map<Robot*, CBSSolution*> _mapfSolution) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConstructHyperpath");

  // First construct the "movement" hyperarcs along skeleton edges
  size_t maxTimestep = 0;
  for(auto kv : _mapfSolution) {
    maxTimestep = std::max(maxTimestep,kv.second->size());
  }

  m_path.Reset();
  m_hidPaths.clear();

  this->GetMPLibrary()->SetGroupTask(m_wholeTask);

  std::vector<std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>>> movementGroups;
  for(size_t t = 0; t < maxTimestep - 1; t++) {
    std::unordered_map<Robot*, SkeletonEdgeDescriptor> individualEdges;
    std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>> edgeGroups;

    for(auto kv : _mapfSolution) {
      auto robot = kv.first;
      auto path = *kv.second;

      auto source = path.size() > t ? path.at(t) : path.at(path.size() - 1);
      auto target = path.size() > t + 1 ? path.at(t+1) : path.at(path.size() - 1);

      // group with robots going both ways on this edge
      auto sVID = std::min(source, target);
      auto tVID = std::max(source, target);

      edgeGroups[std::make_pair(sVID, tVID)].push_back(robot);

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
    }
    movementGroups.push_back(edgeGroups);
  }

  // Now construct the "composition" hyperperarcs to transition b/w groups
  std::vector<std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>>> decoupledGroups;
  std::vector<std::unordered_map<VID, std::vector<Robot*>>> mergedGroups;
  std::vector<std::unordered_map<VID, std::unordered_map<VID, std::vector<Robot*>>>> splitGroups;
  // std::vector<std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>>> coupledGroups;

  for(size_t t = 0; t < maxTimestep - 2; t++) {
    // Decouple groups going the same direction down an edge
    std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>> dGroups;
    for(auto iter : movementGroups[0]) {
      auto robots = iter.second;

      for(auto robot : robots) {
        auto path = *_mapfSolution[robot];

        auto stime = std::min(t, path.size() - 1);
        auto ttime = std::min(t + 1, path.size() - 1);

        auto source = path.at(stime);
        auto target = path.at(ttime);

        dGroups[std::make_pair(source, target)].push_back(robot);
      }
    }
    decoupledGroups.push_back(dGroups);

    // Merge groups going into the same vertex
    std::unordered_map<VID, std::vector<Robot*>> mGroups;
    for(auto iter : dGroups) {
      auto edge = iter.first;
      auto robots = iter.second;

      for(auto robot : robots) {
        mGroups[edge.second].push_back(robot);
      }
    }
    mergedGroups.push_back(mGroups);

    // Split groups going onto different edges
    std::unordered_map<VID, std::unordered_map<VID, std::vector<Robot*>>> sGroups;
    for(auto iter : mGroups) {
      auto robots = iter.second;
      
      for(auto robot : robots) {
        auto path = *_mapfSolution[robot];
        
        auto stime = std::min(t + 1, path.size() - 1);
        auto ttime = std::min(t + 2, path.size() - 1);

        auto source = path.at(stime);
        auto target = path.at(ttime);

        sGroups[source][target].push_back(robot);
      }
    }
    splitGroups.push_back(sGroups);

    // Couple groups going in different directions on the same edge
    // std::unordered_map<std::pair<VID, VID>, std::vector<Robot*>> cGroups;
    // for(auto iter : sGroups) {
    //   auto edge = iter.first;
    //   auto robots = iter.second;

    //   for(auto robot : robots) {
    //     auto vs = std::min(edge.first, edge.second);
    //     auto vt = std::max(edge.first, edge.second);

    //     cGroups[std::make_pair(vs, vt)].push_back(robot);
    //   }
    // }
    // coupledGroups.push_back(cGroups);
  }

  // Now, actually make the movement and composition hyperarcs
  std::unordered_map<size_t, std::unordered_set<VID>> pushStart;
  std::unordered_map<size_t, std::unordered_set<VID>> pushTarget;

  // Track if we need to push the start and/or target to account for merging
  // or splitting
  for(size_t t = 0; t < maxTimestep - 2; t++) {
    for(auto iter : movementGroups.at(t)) {
      auto source = iter.first.first;
      auto target = iter.first.second;
      auto edge = iter.first;
      auto oppEdge = std::make_pair(target, source);
      
      if(mergedGroups.at(t).count(target) and decoupledGroups.at(t).count(edge) and
        mergedGroups.at(t).at(target).size() > decoupledGroups.at(t).at(edge).size())
        pushTarget[t].insert(target);

      if(mergedGroups.at(t).count(source) and decoupledGroups.at(t).count(oppEdge) and
        mergedGroups.at(t).at(source).size() > decoupledGroups.at(t).at(oppEdge).size())
        pushTarget[t].insert(source);

      if(splitGroups.at(t).count(target) and splitGroups.at(t).at(target).size() > 1)
        pushStart[t+1].insert(target);
      
      if(splitGroups.at(t).count(source) and splitGroups.at(t).at(source).size() > 1)
        pushStart[t+1].insert(source);
    }
  }

  // Track the hyperskeleton vertices incoming to the composition step
  std::vector<std::unordered_map<std::pair<VID, VID>, VID>> shvids;
  shvids.reserve(maxTimestep - 1);

  std::vector<std::unordered_map<std::pair<VID, VID>, VID>> thvids;
  thvids.reserve(maxTimestep - 1);

  // Construct the movement hyperarcs
  for(size_t t = 0; t < maxTimestep - 1; t++) {
    shvids.push_back(std::unordered_map<std::pair<VID, VID>, VID>());
    thvids.push_back(std::unordered_map<std::pair<VID, VID>, VID>());
    for(auto iter : movementGroups.at(t)) {
      auto edge = iter.first;
      auto robots = iter.second;
      auto source = edge.first;
      auto target = edge.second;

      auto sourceVal = m_indSkeleton.GetVertex(source);
      auto targetVal = m_indSkeleton.GetVertex(target);
      auto disp = targetVal - sourceVal;
      auto unitDisp = disp / disp.norm();

      auto group = AddGroup(robots);
      auto compSource = CompositeSkeletonVertex(group, &m_indSkeleton);
      auto compTarget = CompositeSkeletonVertex(group, &m_indSkeleton);

      // Push the starts (or don't) of robots going along the edge
      if(source != target and pushStart.count(t) and pushStart[t].count(source)) {
        for(auto robot : robots) {
          // Get the point in workspace of the source skeleton vertex
          auto tt = std::min(t, _mapfSolution[robot]->size() - 1);
          auto skelVID = (*_mapfSolution[robot]).at(tt);
          if(skelVID != source)
            continue;

          auto push = m_regionRadius.at(robot) * sqrt(2);
          push = std::min(push, disp.norm() / 2.0);
          auto skelStart = sourceVal + push * unitDisp;
          compSource.SetRobotCfg(robot, std::move(skelStart));
        }
      } else {
        for(auto robot : robots) {
          // Get the point in workspace of the source skeleton vertex
          auto tt = std::min(t, _mapfSolution[robot]->size() - 1);
          auto skelVID = (*_mapfSolution[robot]).at(tt);
          if(skelVID != source)
            continue;

          compSource.SetRobotCfg(robot, skelVID);
        }
      }

      // Push the starts (or don't) of robots going in the opposite direction
      if(source != target and pushStart.count(t) and pushStart[t].count(target)) {
        for(auto robot : robots) {
          // Get the point in workspace of the source skeleton vertex
          auto tt = std::min(t, _mapfSolution[robot]->size() - 1);
          auto skelVID = (*_mapfSolution[robot]).at(tt);
          if(skelVID != target)
            continue;

          auto push = m_regionRadius.at(robot) * sqrt(2);
          push = std::min(push, disp.norm() / 2.0);
          auto skelStart = targetVal - push * unitDisp;
          compSource.SetRobotCfg(robot, std::move(skelStart));
        }
      } else {
        for(auto robot : robots) {
          // Get the point in workspace of the source skeleton vertex
          auto tt = std::min(t, _mapfSolution[robot]->size() - 1);
          auto skelVID = (*_mapfSolution[robot]).at(tt);
          if(skelVID != target)
            continue;

          compSource.SetRobotCfg(robot, skelVID);
        }
      }

      // Push the targets (or don't) of robots going along the edge
      if(source != target and pushTarget.count(t) and pushTarget[t].count(target)) {
        for(auto robot : robots) {
          // Get the point in workspace of the source skeleton vertex
          auto path = *_mapfSolution[robot];
          auto time = std::min(t+1, path.size() - 1);
          auto skelVID = path.at(time);
          if(skelVID != target)
            continue;

          auto push = m_regionRadius.at(robot) * sqrt(2);
          push = std::min(push, disp.norm() / 2.0);
          auto skelTarget = targetVal - push * unitDisp;
          compTarget.SetRobotCfg(robot, std::move(skelTarget));
        }
      } else {
        for(auto robot : robots) {
          // Get the point in workspace of the source skeleton vertex
          auto path = *_mapfSolution[robot];
          auto time = std::min(t+1, path.size() - 1);
          auto skelVID = path.at(time);
          if(skelVID != target)
            continue;

          compTarget.SetRobotCfg(robot, skelVID);
        }
      }

      // Push the targets (or don't) of robots going in the opposite direction
      if(source != target and pushTarget.count(t) and pushTarget[t].count(source)) {
        for(auto robot : robots) {
          // Get the point in workspace of the source skeleton vertex
          auto path = *_mapfSolution[robot];
          auto time = std::min(t+1, path.size() - 1);
          auto skelVID = path.at(time);
          if(skelVID != source)
            continue;

          auto push = m_regionRadius.at(robot) * sqrt(2);
          push = std::min(push, disp.norm() / 2.0);
          auto skelTarget = sourceVal + push * unitDisp;
          compTarget.SetRobotCfg(robot, std::move(skelTarget));
        }
      } else {
        for(auto robot : robots) {
          // Get the point in workspace of the source skeleton vertex
          auto path = *_mapfSolution[robot];
          auto time = std::min(t+1, path.size() - 1);
          auto skelVID = path.at(time);
          if(skelVID != source)
            continue;

          compTarget.SetRobotCfg(robot, skelVID);
        }
      }

      // Add the source and target to the hyperskeleton
      auto svid = m_skeleton->AddVertex(compSource);
      auto tvid = m_skeleton->AddVertex(compTarget);
      shvids[t].emplace(edge, svid);
      thvids[t].emplace(edge, tvid);

      // If source and target are the same, the group is waiting
      if(svid == tvid) {
        for(auto robot : group->GetRobots())
          m_hidPaths[t][robot] = std::make_pair(false, svid);
        
        continue;
      }

      // Connect the source and target with a hyperarc
      auto compEdge = CompositeSkeletonEdge(group, &m_indSkeleton);
      ComputeIntermediates(_mapfSolution, t, compSource, compTarget, compEdge);
      auto arc = HyperskeletonArc(compEdge, HyperskeletonArcType::Movement);
      auto hid = m_skeleton->AddHyperarc({tvid}, {svid}, arc, false, true);
      m_path.movementHyperarcs.insert(hid);
      m_path.buildMovementHyperarcs.push_back(hid);

      // For ease of future use, set predecessor and successor to self
      for(auto robot : group->GetRobots()) {
        m_path.predecessors[hid].emplace(robot, std::make_pair(true, hid));
        m_path.successors[hid].emplace(robot, std::make_pair(true, hid));
        m_hidPaths[t][robot] = std::make_pair(true, hid);
      }
    }
  }

  // Create rest of the hyperskeleton vertices and hyperarcs
  std::vector<std::unordered_map<VID, std::set<VID>>> dvids;
  dvids.reserve(maxTimestep - 1);

  std::vector<std::unordered_map<VID, VID>> mvids;
  mvids.reserve(maxTimestep - 1);

  std::vector<std::unordered_map<std::pair<VID, VID>, VID>> svids;
  svids.reserve(maxTimestep - 1);

  for(size_t t = 0; t < maxTimestep - 2; t++) {
    // Construct the decouple hyperarcs at this timestep
    for(auto iter : movementGroups.at(t)) {
      dvids.push_back(std::unordered_map<VID, std::set<VID>>());

      auto edge = iter.first;
      auto robots = iter.second;
      auto source = edge.first;
      auto target = edge.second;
      auto oppEdge = std::make_pair(target, source);

      auto group = AddGroup(robots);
      auto svid = thvids[t][edge];
      auto svertex = m_skeleton->GetVertexType(svid);

      if(edge != oppEdge and decoupledGroups[t].count(edge) and decoupledGroups[t].count(oppEdge)) {
        auto r1 = decoupledGroups[t][edge];
        auto r2 = decoupledGroups[t][oppEdge];

        auto g1 = AddGroup(r1);
        auto g2 = AddGroup(r2);

        // Construct vertices for each of the decoupled groups
        auto compV1 = CompositeSkeletonVertex(g1);
        for(auto r : r1) {
          Point3d cfg = svertex.GetRobotCfg(r);
          compV1.SetRobotCfg(r, std::move(cfg));
        }

        auto compV2 = CompositeSkeletonVertex(g2);
        for(auto r : r2) {
          Point3d cfg = svertex.GetRobotCfg(r);
          compV2.SetRobotCfg(r, std::move(cfg));
        }

        auto tvid1 = m_skeleton->AddVertex(compV1);
        auto tvid2 = m_skeleton->AddVertex(compV2);

        dvids[t][target].insert(tvid1);
        dvids[t][source].insert(tvid2);

        auto compEdge = CompositeSkeletonEdge(group);
        auto arc = HyperskeletonArc(compEdge, HyperskeletonArcType::Decouple);
        auto hid = m_skeleton->AddHyperarc({tvid1, tvid2}, {svid}, arc, false, true);
        m_path.decoupleHyperarcs.insert(hid);

        if(this->m_debug)
          std::cout << "Decoupled 2 groups at time " << t << std::endl;

        // Set the predecessor movement hyperarc
        for(auto ih : m_skeleton->GetIncomingHyperarcs(svid)) {
          if(m_path.movementHyperarcs.count(ih)) {
            for(auto robot : group->GetRobots())
              m_path.predecessors[hid].emplace(robot, std::make_pair(true, ih));
            
            break;
          }
        }
      } else {
        // No decoupling, pass vertex along to next step, just need to know
        // which direction the robots are moving
        if(decoupledGroups[t].count(edge))
          dvids[t][target].insert(svid);
        else
          dvids[t][source].insert(svid);
      }
    }

    // Construct the merge hyperarcs
    for(auto iter : dvids.at(t)) {
      mvids.push_back(std::unordered_map<VID, VID>());

      auto skelVID = iter.first;
      auto hvids = iter.second;

      if(hvids.size() == 1) {
        // No need to merge, only 1 incoming group
        mvids[t].emplace(skelVID, *hvids.begin());
        continue;
      }

      std::vector<Robot*> robots;
      std::unordered_set<Robot*> waiting;
      for(auto vid : hvids) {
        auto vertex = m_skeleton->GetVertexType(vid);
        auto rs = vertex.GetGroup()->GetRobots();
        robots.insert(robots.end(), rs.begin(), rs.end());

        // Preliminary check if this group is waiting
        for(auto robot : rs) {
          if(t >= _mapfSolution[robot]->size()-1) {
            if(this->m_debug)
              std::cout << "Found end-of-path waiting into merge at " << t 
                        << " " << vid << std::endl;

            if(!m_waitingReps.count(vid)) {
              auto vertex = m_skeleton->GetVertexType(vid);
              auto subgroup = vertex.GetGroup();
              auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(subgroup);
              auto wvid = SpawnVertex(grm, vertex);
              m_waitingReps.emplace(vid, std::make_pair(grm, wvid));
            }
          }

          // Also check if this merge is the first movement for a robot (i.e. 
          // waiting at start)
          bool startWaiting = true;
          for (int tt = t-1; tt >= 0; tt--) {
            if (_mapfSolution[robot]->at(std::min((size_t)tt, _mapfSolution[robot]->size()-1)) != 
                _mapfSolution[robot]->at(std::min((size_t)tt+1, _mapfSolution[robot]->size()-1))) {
              startWaiting = false;
              break;
            }
          }
          
          if (startWaiting) {
            waiting.insert(robot);
            if(!m_waitingReps.count(vid)) {
              auto vertex = m_skeleton->GetVertexType(vid);
              auto subgroup = vertex.GetGroup();
              auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(subgroup);
              auto wvid = SpawnVertex(grm, vertex);
              m_waitingReps.emplace(vid, std::make_pair(grm, wvid));
            }
          }
        }
      }

      auto group = AddGroup(robots);
      auto compV = CompositeSkeletonVertex(group, &m_indSkeleton);
      for(auto r : robots) {
        compV.SetRobotCfg(r, skelVID);
      }
      auto tvid = m_skeleton->AddVertex(compV);
      mvids[t].emplace(skelVID, tvid);

      auto compEdge = CompositeSkeletonEdge(group);
      auto arc = HyperskeletonArc(compEdge, HyperskeletonArcType::Merge);
      auto hid = m_skeleton->AddHyperarc({tvid}, hvids, arc, false, true);
      m_path.mergeHyperarcs.insert(hid);

      if(this->m_debug)
        std::cout << "Merged " << hvids.size() << " groups at time " << t << std::endl;

      // Check if there is a start of path wait coming into this merge
      if(t == 0) {
        for(auto v : hvids) {
          auto rs = m_skeleton->GetVertexType(v).GetGroup()->GetRobots();
          for(auto r : rs) {
            auto path = *_mapfSolution[r];
            auto ssvid = path.at(std::min(t, path.size() - 1));
            auto stvid = path.at(std::min(t+1, path.size() - 1));

            if(ssvid == stvid) {
              if(this->m_debug)
                std::cout << "Found " << r->GetLabel() << " waiting at start into merge" << std::endl;

              m_path.predecessors[hid].emplace(r, std::make_pair(false, v));
              if(!m_waitingReps.count(v)) {
                auto vertex = m_skeleton->GetVertexType(v);
                auto subgroup = vertex.GetGroup();
                auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(subgroup);
                auto vid = SpawnVertex(grm, vertex);
                m_waitingReps.emplace(v, std::make_pair(grm, vid));
              }
            }
          }
        }
      } else {
        for(auto v : hvids) {
          auto rs = m_skeleton->GetVertexType(v).GetGroup()->GetRobots();
          for(auto r : rs) {
            if (waiting.count(r)) {
              m_path.predecessors[hid].emplace(r, std::make_pair(false, v));
            }
          }
        }
      }

      // Set the predecessor map for each robot
      for(auto v : hvids) {
        bool foundSplit = false;
        for(auto ih : m_skeleton->GetIncomingHyperarcs(v)) {
          // First check if any split hyperarcs are incoming (this means we're
          // currently waiting)
          if(m_path.splitHyperarcs.count(ih)) {
            if(this->m_debug)
              std::cout << "found split incoming to merge " << tvid << std::endl;
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
          for(auto ih : m_skeleton->GetIncomingHyperarcs(v)) {
            if(m_path.movementHyperarcs.count(ih) or m_path.decoupleHyperarcs.count(ih)) {
              for(auto robot : group->GetRobots()) {
                if(!m_path.predecessors.at(ih).count(robot))
                  continue;
                  
                auto boolhid = m_path.predecessors.at(ih).at(robot);
                m_path.predecessors[hid].emplace(robot, boolhid);
              }
              
              break;
            }
          }
        }
      }
    }

    // Construct the split hyperarcs
    for(auto iter : mvids.at(t)) {
      svids.push_back(std::unordered_map<std::pair<VID, VID>, VID>());

      auto skelVID = iter.first;
      auto hvid = iter.second;

      if(splitGroups[t][skelVID].size() == 1) {
        // No need to split, only 1 group outgoing
        auto target = (*splitGroups[t][skelVID].begin()).first;
        svids[t].emplace(std::make_pair(skelVID, target), hvid);
        continue;
      }

      std::set<VID> splitVIDs;
      std::vector<Robot*> robots;
      std::unordered_map<Robot*, VID> splitVIDMap;
      for(auto iter : splitGroups[t][skelVID]) {
        auto target = iter.first;
        auto rs = iter.second;
        robots.insert(robots.end(), rs.begin(), rs.end());

        // Get the position on the outgoing skeleton edge
        auto svid = std::min(skelVID, target);
        auto tvid = std::max(skelVID, target);
        auto nextV = shvids[t+1][std::make_pair(svid, tvid)];
        auto nextVertex = m_skeleton->GetVertexType(nextV);

        auto group = AddGroup(rs);
        auto compV = CompositeSkeletonVertex(group, &m_indSkeleton);

        for(auto r : rs) {
          Point3d cfg = nextVertex.GetRobotCfg(r);
          compV.SetRobotCfg(r, std::move(cfg));
        }

        auto spvid = m_skeleton->AddVertex(compV);
        splitVIDs.insert(spvid);
        for(auto r : rs)
          splitVIDMap[r] = spvid;

        svids[t].emplace(std::make_pair(skelVID, target), spvid);
      }

      auto group = AddGroup(robots);
      auto compEdge = CompositeSkeletonEdge(group);
      auto arc = HyperskeletonArc(compEdge, HyperskeletonArcType::Split);
      auto hid = m_skeleton->AddHyperarc(splitVIDs, {hvid}, arc, false, true);
      m_path.splitHyperarcs.insert(hid);

      if(this->m_debug)
        std::cout << "Split " << splitVIDs.size() << " groups at time " << t << std::endl;

      // Set the successor map for each robot
      // This will be overwritten for merges going back into a split
      for(auto iter : splitVIDMap) {
        auto robot = iter.first;
        auto svid = iter.second;

        if(m_hidPaths.at(t+1).at(robot).first) {
          m_path.successors[hid][robot] = m_hidPaths.at(t+1).at(robot);
          continue;
        }

        // Make sure we are actually waiting, if hidPaths says waiting
        auto found = false;
        for(auto oh : m_skeleton->GetOutgoingHyperarcs(svid)) {
          auto arc = m_skeleton->GetHyperarcType(oh);
          if(arc.type == HyperskeletonArcType::Movement and m_path.movementHyperarcs.count(oh)) {
            found = true;
            m_path.successors[hid][robot] = std::make_pair(true, oh);
            break;
          } else if(arc.type == HyperskeletonArcType::Couple and m_path.coupleHyperarcs.count(oh)) {
            auto head = *m_skeleton->GetHyperarc(oh).head.begin();
            for(auto ooh : m_skeleton->GetOutgoingHyperarcs(head)) {
              auto outArc = m_skeleton->GetHyperarcType(ooh);
              if(outArc.type == HyperskeletonArcType::Movement and m_path.movementHyperarcs.count(ooh)) {
                found = true;
                m_path.successors[hid][robot] = std::make_pair(true, ooh);
                break;
              }
            }
            if(found)
              break;
          }
        }

        if(!found) {
          m_path.successors[hid][robot] = m_hidPaths.at(t+1).at(robot);
        }
      }
    }

    // Construct the couple hyperarcs
    for(auto iter : movementGroups.at(t+1)) {
      auto edge = iter.first;
      auto source = edge.first;
      auto target = edge.second;
      auto oppEdge = std::make_pair(target, source);
      auto robots = iter.second;

      if(edge != oppEdge and svids[t].count(edge) and svids[t].count(oppEdge)) {
        // Need to couple groups going in opposite directions
        auto group = AddGroup(robots);
        auto compEdge = CompositeSkeletonEdge(group);
        auto arc = HyperskeletonArc(compEdge, HyperskeletonArcType::Couple);
        auto thvid = shvids[t+1][edge];

        auto s1 = svids[t].at(edge);
        auto s2 = svids[t].at(oppEdge);

        auto hid = m_skeleton->AddHyperarc({thvid}, {s1, s2}, arc, false, true);
        m_path.coupleHyperarcs.insert(hid);

        if(this->m_debug)
          std::cout << "Coupled 2 groups at time " << t << std::endl;

        // Set the successor map for each robot in this couple arc
        for(auto robot : robots)
          m_path.successors[hid][robot] = std::make_pair(true, hid);
      }
    }
  }
}

bool
WoDaSH::
GroundHyperskeleton() {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GroundHyperskeleton");

  // TODO cast this to avoid having to change MPStratgy interface?
  auto s = this->GetMPLibrary()->GetMPStrategy(m_drStrategy);

  for(auto hid : m_path.buildMovementHyperarcs) {
    auto& hyperarc = m_skeleton->GetHyperarc(hid);

    // Don't try to re-ground an edge
    if(hyperarc.property.startRep.first != nullptr and 
       hyperarc.property.endRep.first != nullptr) {
      m_path.groundedSolution.insert(m_skeletonToGrounded.at(hid));
      continue;
    }

    auto& cedge = hyperarc.property.edge;

    // Should only have one head and tail
    auto svid = *(hyperarc.tail.begin());

    // Check if we're waiting at a vertex before this, if so, take representative
    // Also check if there's a movement hyperarc before this, if so, take the
    // End representative
    if(m_waitingReps.count(svid)) {
      hyperarc.property.startRep = m_waitingReps.at(svid);
    } else {
      bool found = false;
      std::unordered_map<Robot*, VID> coupled;
      for(auto ih : m_skeleton->GetIncomingHyperarcs(svid)) {
        auto& arc = m_skeleton->GetHyperarcType(ih);
        if(arc.type == HyperskeletonArcType::Movement and 
            m_path.movementHyperarcs.count(ih)) {
          found = true;
          hyperarc.property.startRep = arc.endRep;
          break;
        } else if(arc.type == HyperskeletonArcType::Couple and
            m_path.coupleHyperarcs.count(ih)) {

          auto tail = m_skeleton->GetHyperarc(ih).tail;
          for(auto t : tail) {
            for(auto iih : m_skeleton->GetIncomingHyperarcs(t)) {
              auto& inArc = m_skeleton->GetHyperarcType(iih);
              if(inArc.type == HyperskeletonArcType::Movement and 
                  m_path.movementHyperarcs.count(iih)) {
                  
                auto vertex = inArc.endRep.first->GetVertex(inArc.endRep.second);
                for(auto r : inArc.edge.GetGroup()->GetRobots()) {
                  coupled[r] = vertex.GetVID(r);
                }
              }
            }
          }
        } else if(arc.type == HyperskeletonArcType::Decouple and 
            m_path.decoupleHyperarcs.count(ih)) {

          // Add end representative from last movement to new group roadmap
          auto dvid = *m_skeleton->GetHyperarc(ih).tail.begin();
          for(auto iih : m_skeleton->GetIncomingHyperarcs(dvid)) {
            auto inArc = m_skeleton->GetHyperarcType(iih);
            found = true;

            auto grm = this->GetMPLibrary()->GetGroupRoadmap(cedge.GetGroup());
            auto incfg = inArc.endRep.first->GetVertex(inArc.endRep.second);
            auto gcfg = GroupCfgType(grm);
            for(auto robot : cedge.GetGroup()->GetRobots()) {
              gcfg.SetRobotCfg(robot, incfg.GetVID(robot));
            }

            auto vid = grm->AddVertex(gcfg);
            hyperarc.property.startRep = std::make_pair(grm, vid);
            break;
          }
        }
        if(found)
          break;
      }

      if(coupled.size() == cedge.GetGroup()->Size()) {
        auto grm = this->GetMPLibrary()->GetGroupRoadmap(cedge.GetGroup());
        auto gcfg = GroupCfgType(grm);
        for(auto iter : coupled) {
          gcfg.SetRobotCfg(iter.first, iter.second);
        }
        auto gvid = grm->AddVertex(gcfg);
        hyperarc.property.startRep = std::make_pair(grm, gvid);
        found = true;
      }

      if(!found) {
        auto grm = this->GetMPLibrary()->GetGroupRoadmap(cedge.GetGroup());
        auto startVID = SpawnVertex(grm, cedge);
        hyperarc.property.startRep = std::make_pair(grm, startVID);
      }
    }
    
    // Set the task
    ///@todo Does it matter what task this is? There could be many
    this->GetMPLibrary()->SetGroupTask(m_groupTaskMap[cedge.GetGroup()]);
    SetVirtualExcept(cedge.GetGroup());

    s->ResetGrowthOptions();
    s->AddGrowthOption(hyperarc.property.startRep.second);
    s->GroundEdge(cedge);

    // Check if this was successful
    auto grm = this->GetMPLibrary()->GetGroupRoadmap();
    auto startVID = hyperarc.property.startRep.second;
    auto lastVID = grm->GetLastVID();
    auto lastVertex = grm->GetVertex(lastVID);
    if(!FinishedEdge(cedge, lastVertex)) {
      if(this->m_debug)
        std::cout << "Failed to ground HID " << hid << ", replanning..." << std::endl;

      // Couldn't finish edge, need to replan hyperpath.
      auto eds = cedge.GetEdgeDescriptors();

      // Keep track of failed edges
      std::unordered_map<Robot*, std::pair<VID, VID>> edges;
      auto rs = cedge.GetGroup()->GetRobots();
      for(size_t r = 0; r < rs.size(); r++) {
        // Add the ordered edge to failed edges
        auto startVID = std::min(eds[r].source(), eds[r].target());
        auto targetVID = std::max(eds[r].source(), eds[r].target());
        edges.emplace(rs[r], std::make_pair(startVID, targetVID));
      }
      
      // Don't want to use CBS for single robot issues
      if(edges.size() > 1)
        m_failedEdges.push_back(edges);

      return false;
    }
    
    if(this->m_debug)
      std::cout << "Successfully grounded HID " << hid << std::endl;
    hyperarc.property.endRep = std::make_pair(grm, lastVID);

    // Convert vids into a task and path to save in grounded hypergraph

    // Configure group task
    auto task = std::shared_ptr<GroupTask>(new GroupTask(grm->GetGroup()));
    auto startGcfg = grm->GetVertex(startVID);
    auto endGcfg = grm->GetVertex(lastVID);
    for(auto robot : grm->GetGroup()->GetRobots()) {
      MPTask t(robot);
      std::string l = "GroundMoveHyperarc";
      l = l + std::to_string(hid);
      t.SetLabel(l);

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
    
    auto ghid = AddTransitionToGroundedHypergraph(hyperarc.tail,hyperarc.head,path,task);
    m_skeletonToGrounded[hid] = ghid;
    m_path.groundedSolution.insert(ghid);
  }

  this->GetMPLibrary()->SetGroupTask(m_wholeTask);
  SetVirtualExcept();

  if(this->m_debug)
    std::cout << "Successfully grounded " << m_path.movementHyperarcs.size() 
              << " edges, sampling trajectories..." << std::endl;

  return SampleTrajectories();
}

void
WoDaSH::
ComputeIntermediates(std::unordered_map<Robot*, CBSSolution*> _mapfSolution, 
                     size_t _timestep,
                     CompositeSkeletonVertex& _compSource, 
                     CompositeSkeletonVertex& _compTarget, 
                     CompositeSkeletonEdge& _compEdge) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ComputeIntermediates");

  // Assume a piecewise linear edge
  auto robots = _compSource.GetGroup()->GetRobots();

  double diam = 0;
  for(auto robot : robots)
    diam += robot->GetMultiBody()->GetBoundingSphereRadius() * 2;
  double intLength = (diam / robots.size()) * m_intermediateFactor;
  double tolerance = 1e-5;

  // Set the individual edges for each robot
  std::vector<double> dists;
  std::vector<size_t> inters; // current start intermediate
  std::vector<double> useds; // distance used between start and current position
  std::vector<std::vector<Point3d>> edges;
  double maxDist = 0;
  for(auto robot : robots) {
    auto path = *_mapfSolution[robot];
    auto stime = std::min(_timestep, path.size()-1);
    auto ttime = std::min(_timestep+1, path.size()-1);
    auto skelSource = path.at(stime);
    auto skelTarget = path.at(ttime);

    if(skelSource == skelTarget)
      throw RunTimeException(WHERE) << "Tried to add edge between same vids";

    // Get the workspace skeleton edge descriptor
    WorkspaceSkeleton::adj_edge_iterator ei;
    auto found = m_indSkeleton.GetEdge(skelSource, skelTarget, ei);

    SkeletonEdgeDescriptor ed;
    if(found)
      ed = ei->descriptor();
    else
      ed = m_indSkeleton.AddEdge(skelSource, skelTarget); // self-edge

    auto edge = m_indSkeleton.GetEdge(ed);
    edges.push_back(edge);
    double total = 0.0;
    for(size_t i = 0; i < edge.size()-1; i++)
      total += (edge.at(i+1) - edge.at(i)).norm();

    // The true source and target positions may be pushed for merging/splitting
    auto start = _compSource.GetRobotCfg(robot);
    auto target = _compTarget.GetRobotCfg(robot);

    bool pushedStart = (start - m_indSkeleton.GetVertex(skelSource)).norm() > tolerance;
    bool pushedTarget = (target - m_indSkeleton.GetVertex(skelTarget)).norm() > tolerance;
    
    auto pushDist = m_regionRadius.at(robot) * sqrt(2);
    auto totalEdge = total;
    if(pushedStart) {
      auto pushed = std::min(pushDist, totalEdge / 2.0);
      total -= pushed;

      // Update the first intermediate to use
      double elapsed = 0.;
      size_t inter = 0;
      double used = 0.;
      while(fabs(elapsed - pushed) > tolerance) {
        auto iterDist = (edge.at(inter+1) - edge.at(inter)).norm();
        auto deltaUsed = std::min(iterDist, pushed - elapsed);
        elapsed += deltaUsed;
        used += deltaUsed;
        if(fabs(elapsed - pushed) > tolerance) {
          inter++;
          used = 0;
        }
      }

      inters.push_back(inter);
      useds.push_back(used);
    } else {
      inters.push_back(0);
      useds.push_back(0);
    }

    if(pushedTarget)
      total -= std::min(pushDist, totalEdge / 2.0);

    dists.push_back(total);
    if(total > maxDist)
      maxDist = total;

    _compEdge.SetEdge(robot, ed);
  }

  // Find the number of composite edge intermediates
  std::vector<CompositeSkeletonVertex> compInters;
  int numInter = (int)ceil(maxDist/intLength);

  // Make the first intermediate
  auto v = CompositeSkeletonVertex(_compSource.GetGroup());
  for(size_t r = 0; r < robots.size(); r++) {
    auto inter = inters.at(r);
    auto edge = edges.at(r);
    auto disp = edge.at(inter+1) - edge.at(inter);
    disp = disp / disp.norm();
    auto point = edge.at(inter) + (disp * useds.at(r));
    v.SetRobotCfg(r, std::move(point));
  }
  compInters.push_back(v);

  // Set the intermediate values along the edge
  for(int i = 0; i < numInter; ++i){
    auto u = CompositeSkeletonVertex(_compSource.GetGroup());

    for(size_t r = 0; r < robots.size(); r++) {
      // Get the intermediate distance for this robot
      auto intDist = dists.at(r) / numInter;
      double elapsed = 0.;

      auto inter = inters.at(r);
      auto used = useds.at(r);
      auto edge = edges.at(r);
      while(fabs(elapsed - intDist) > tolerance) {
        auto fullLength = (edge.at(inter+1) - edge.at(inter)).norm();
        auto deltaUsed = std::min(fullLength - used, intDist - elapsed);
        elapsed += deltaUsed;
        used += deltaUsed;
        if(fabs(elapsed - intDist) > tolerance) {
          inter++;
          used = 0.;
        }
      }
      useds[r] = used;
      inters[r] = inter;

      auto disp = edge.at(inter+1) - edge.at(inter);
      disp = disp / disp.norm();
      auto point = edge.at(inter) + (disp * used);
      u.SetRobotCfg(r, std::move(point));
    }

    compInters.push_back(u);
  }

  // Set the intermediates
  _compEdge.SetIntermediates(compInters);
}

size_t
WoDaSH::
SpawnVertex(GroupRoadmapType* _grm, CompositeSkeletonEdge _edge) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SpawnVertex");

  this->GetMPLibrary()->SetGroupTask(m_groupTaskMap.at(_edge.GetGroup()));
  SetVirtualExcept(_edge.GetGroup());

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

  this->GetMPLibrary()->SetGroupTask(m_wholeTask);
  SetVirtualExcept();
  return vid;
}

size_t
WoDaSH::
SpawnVertex(GroupRoadmapType* _grm, CompositeSkeletonVertex _vertex) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SpawnVertex");

  this->GetMPLibrary()->SetGroupTask(m_groupTaskMap.at(_vertex.GetGroup()));
  SetVirtualExcept(_vertex.GetGroup());

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
  _grm->DisableHooks();
  for(size_t i = 0; i < robots.size(); i++)
    _grm->GetIndividualGraph(i)->DisableHooks();

  auto vid = _grm->AddVertex(target);

  _grm->EnableHooks();
  for(size_t i = 0; i < robots.size(); i++)
    _grm->GetIndividualGraph(i)->EnableHooks();

  this->GetMPLibrary()->SetGroupTask(m_wholeTask);
  SetVirtualExcept();
  return vid;
}

bool
WoDaSH::
FinishedEdge(CompositeSkeletonEdge _edge, const GroupCfgType& _groupCfg) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::FinishedEdge");

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
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::MakeBoundary");

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

bool
WoDaSH::
ConnectToSkeleton() {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConnectToSkeleton");

  if(this->m_debug)
    std::cout << "Attempting to connect start and goal to skeleton..." << std::endl;

  auto gh = dynamic_cast<GroundedHypergraph*>(
    this->GetStateGraph(m_groundedHypergraphLabel).get());

  auto s = this->GetMPLibrary()->GetMPStrategy(m_trajStrategy);

  // Find the skeleton start for each decoupled group (per starting edge)
  std::unordered_map<RobotGroup*, RepresentativeVertex> skeletonStarts;
  for(auto robot : m_wholeGroup->GetRobots()) {
    auto hid = m_hidPaths[0].at(robot);
    
    RepresentativeVertex rep;
    if(!hid.first) {
      auto origGroup = m_skeleton->GetVertexType(hid.second).GetGroup();
      size_t idx = 1;
      while(!hid.first and !m_waitingReps.count(hid.second)) {
        hid = m_hidPaths[idx].at(robot);
        idx++;
      }
      if(!hid.first) {
        rep = m_waitingReps.at(hid.second);
        auto newGroup = m_skeleton->GetVertexType(hid.second).GetGroup();
        if(origGroup != newGroup)
          throw RunTimeException(WHERE) << "Group mismatch found at start";
      } else {
        auto& arc = m_skeleton->GetHyperarcType(hid.second);
        rep = arc.startRep;
        auto newGroup = rep.first->GetVertex(rep.second).GetGroup();
        if(origGroup != newGroup)
          throw RunTimeException(WHERE) << "Group mismatch found at start";
      }
    } else {
      auto& arc = m_skeleton->GetHyperarcType(hid.second);
      rep = arc.startRep;
    }

    auto group = rep.first->GetGroup();
    if(skeletonStarts.count(group) and skeletonStarts[group] != rep)
      throw RunTimeException(WHERE) << "Robot in two places at start!";

    skeletonStarts[group] = rep;
  }

  // Sample the starts for each group
  std::unordered_map<RobotGroup*, VID> groundedStarts;
  std::set<VID> groundedHead;
  for(auto iter : skeletonStarts) {
    auto group = iter.first;

    // Set the task for that group
    this->GetMPLibrary()->SetGroupTask(m_groupTaskMap.at(group));
    SetVirtualExcept(group);

    auto svid = s->GenerateStart(m_sampler);
    auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(group);
    
    auto gvid = gh->AddVertex(std::make_pair(grm, svid));
    groundedStarts[group] = gvid;
    groundedHead.insert(gvid);
  }

  // Create the virtual transition from the source to the starts
  GroundedHypergraph::Transition transition;
  transition.cost = 0;
  auto vhid = gh->GetHID({m_virtualSource}, groundedHead);
  if(vhid == MAX_UINT)
    vhid = gh->AddTransition({m_virtualSource}, groundedHead, transition);
  m_path.groundedSolution.insert(vhid);

  // Sample each trajectory from the start to the skeleton start per group
  for(auto iter : skeletonStarts) {
    auto group = iter.first;
    auto skelRep = iter.second;
    auto startRep = gh->GetVertex(groundedStarts.at(group));

    // Check if this hyperarc already exists
    auto head = gh->AddVertex(skelRep);
    auto tail = groundedStarts.at(group);
    auto ghid = gh->GetHID({tail}, {head});
    if(ghid != MAX_UINT) {
      m_path.groundedSolution.insert(ghid);
      continue;
    }

    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto robot : group->GetRobots()) {
      auto t = MPTask(robot);
      t.SetLabel("ConnectStart");

      auto startCfg = startRep.first->GetVertex(startRep.second).GetRobotCfg(robot);
      auto goalCfg = skelRep.first->GetVertex(skelRep.second).GetRobotCfg(robot);

      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,startCfg));
      t.SetStartConstraint(std::move(startConstraint));

      auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,goalCfg));
      t.AddGoalConstraint(std::move(goalConstraint));

      task->AddTask(t);
    }

    this->GetMPLibrary()->SetGroupTask(m_groupTaskMap.at(group));
    SetVirtualExcept(group);

    s->ResetGrowthOptions();
    this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
      this->GetMPSolution(), m_trajStrategy, LRand(), "ConnectToSkeleton");

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size()) {
      if(this->m_debug)
        std::cout << "Failed to connect the start to the skeleton." << std::endl;
      return false;
    } else {
      if(this->m_debug)
        std::cout << "Successfully connected group of size " << group->Size() << std::endl;
    }

    ghid = AddTransitionToGroundedHypergraph({startRep}, {skelRep}, path, task);
    m_path.groundedSolution.insert(ghid);
  }

  // Find the skeleton targets for each group
  std::unordered_map<RobotGroup*, RepresentativeVertex> skeletonGoals;
  for(auto robot : m_wholeGroup->GetRobots()) {
    
    auto time = m_hidPaths.size() - 1;
    auto hid = m_hidPaths.at(time).at(robot);
    
    RepresentativeVertex rep;
    if(!hid.first) {
      if(m_waitingReps.count(hid.second) and 
            (m_waitingReps.at(hid.second).first != nullptr))
        rep = m_waitingReps.at(hid.second);
      else {
        // Get the last incoming movement hyperarc
        for(auto ihs : m_skeleton->GetIncomingHyperarcs(hid.second)) {
          auto arc = m_skeleton->GetHyperarcType(ihs);
          if(arc.type == HyperskeletonArcType::Movement and 
                          m_path.movementHyperarcs.count(ihs)) {
            rep = arc.endRep;
            break;
          } else if(arc.type == HyperskeletonArcType::Decouple and 
                          m_path.decoupleHyperarcs.count(ihs)) {
            // Get the movement incoming to this decouple
            auto tailVertex = *m_skeleton->GetHyperarc(ihs).tail.begin();
            for(auto iihs : m_skeleton->GetIncomingHyperarcs(tailVertex)) {
              auto inArc = m_skeleton->GetHyperarcType(iihs);
              if(inArc.type == HyperskeletonArcType::Movement and 
                          m_path.movementHyperarcs.count(iihs)) {
                rep = inArc.endRep;
                break;
              }
            }
            if(rep.first)
              break;
          }
        }
      }
    } else {
      auto& arc = m_skeleton->GetHyperarcType(hid.second);
      rep = arc.endRep;
    }

    auto group = rep.first->GetGroup();
    if(skeletonGoals.count(group) and skeletonGoals[group] != rep)
      throw RunTimeException(WHERE) << "Robot in two places at goal!";

    skeletonGoals[group] = rep;
  }

  // Sample the starts for each group
  std::unordered_map<RobotGroup*, VID> groundedGoals;
  std::set<VID> groundedTail;
  for(auto iter : skeletonGoals) {
    auto group = iter.first;

    // Set the task for that group
    this->GetMPLibrary()->SetGroupTask(m_groupTaskMap.at(group));
    SetVirtualExcept(group);

    auto tvid = s->GenerateGoals(m_sampler).at(0);
    auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(group);
    
    auto gvid = gh->AddVertex(std::make_pair(grm, tvid));
    groundedGoals[group] = gvid;
    groundedTail.insert(gvid);
  }

  // Create the virtual transition from the source to the starts
  vhid = gh->GetHID(groundedTail, {m_virtualSink});
  if(vhid == MAX_UINT)
    gh->AddTransition(groundedTail, {m_virtualSink}, transition);
  // m_path.groundedSolution.insert(vhid);

  // Sample each trajectory from the goal to the skeleton goal per group
  for(auto iter : skeletonGoals) {
    auto group = iter.first;
    auto skelRep = iter.second;
    auto goalRep = gh->GetVertex(groundedGoals.at(group));

    // Check if this hyperarc already exists
    auto tail = gh->AddVertex(skelRep);
    auto head = groundedGoals.at(group);
    auto ghid = gh->GetHID({tail}, {head});
    if(ghid != MAX_UINT) {
      m_path.groundedSolution.insert(ghid);
      continue;
    }

    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto robot : group->GetRobots()) {
      auto t = MPTask(robot);
      t.SetLabel("ConnectGoal");

      auto startCfg = skelRep.first->GetVertex(skelRep.second).GetRobotCfg(robot);
      auto goalCfg = goalRep.first->GetVertex(goalRep.second).GetRobotCfg(robot);

      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,startCfg));
      t.SetStartConstraint(std::move(startConstraint));

      auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,goalCfg));
      t.AddGoalConstraint(std::move(goalConstraint));

      task->AddTask(t);
    }

    this->GetMPLibrary()->SetGroupTask(m_groupTaskMap.at(group));
    SetVirtualExcept(group);

    s->ResetGrowthOptions();
    this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
      this->GetMPSolution(), m_trajStrategy, LRand(), "ConnectToSkeleton");

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size()) {
      if(this->m_debug)
        std::cout << "Failed to connect the goal to the skeleton." << std::endl;
      return false;
    }

    ghid = AddTransitionToGroundedHypergraph({skelRep}, {goalRep}, path, task);
    m_path.groundedSolution.insert(ghid);
  }

  return true;
}

bool
WoDaSH::
SampleTrajectories() {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SampleTrajectories");

  // First, split apart groups that have finished moving in opposite
  // directions down the same skeleton edge. Single node paths.
  for(auto hid : m_path.decoupleHyperarcs) {
    if(m_cachedTrajs.count(hid)) {
      if(this->m_debug)
        std::cout << "Already have trajectory for hid " << hid << std::endl;

      m_path.groundedSolution.insert(m_skeletonToGrounded.at(hid));
      continue;
    }

    if(this->m_debug)
      std::cout << "Constructing trajectory for opp split hid " << hid << std::endl;

    auto hyp = m_skeleton->GetHyperarc(hid);
    auto cedge = hyp.property.edge;
    auto group = cedge.GetGroup();

    auto tail = *hyp.tail.begin();
    auto ihs = *(m_skeleton->GetIncomingHyperarcs(tail).begin());
    auto inArc = m_skeleton->GetHyperarcType(ihs);

    auto start = inArc.endRep.first->GetVertex(inArc.endRep.second);

    // Plan a path between the start and target - should be empty path
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto r : group->GetRobots()) {
      auto t = MPTask(r);
      t.SetLabel("OppSplit");

      // Add start constraints
      auto startCfg = start.GetRobotCfg(r);
      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
      t.SetStartConstraint(std::move(startConstraint));

      // TODO make this less ugly
      // Get the head vertex that has this robot
      CfgType target;
      bool found = false;
      for(auto h : hyp.head) {
        auto compVert = m_skeleton->GetVertexType(h);

        if(compVert.GetGroup()->VerifyRobotInGroup(r)) {
          for(auto oh : m_skeleton->GetOutgoingHyperarcs(h)) {
            auto& arc = m_skeleton->GetHyperarcType(oh);

            if(arc.type == HyperskeletonArcType::Movement and 
              m_path.movementHyperarcs.count(oh)) {
              
              found = true;
              auto gcfg = arc.startRep.first->GetVertex(arc.startRep.second);
              target = gcfg.GetRobotCfg(r);
              break;
            }
          }
          break;
        }
      }

      if(!found)
        target = start.GetRobotCfg(r);

      // Add goal constraints
      auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,target));
      t.AddGoalConstraint(std::move(goalConstraint));

      task->AddTask(t);
    } 

    SetVirtualExcept(group);
    auto s = this->GetMPLibrary()->GetMPStrategy(m_trajStrategy);
    s->ResetGrowthOptions();
    this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
        this->GetMPSolution(), m_trajStrategy, LRand(), "SampleTrajectories");

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size()) {
      throw RunTimeException(WHERE) << "Failed decouple :(";
    }

    // Extract path from solution
    auto ghid = AddTransitionToGroundedHypergraph(hyp.tail,hyp.head,path,task);
    m_cachedTrajs.insert(hid);
    m_skeletonToGrounded[hid] = ghid;
    m_path.groundedSolution.insert(ghid);
  }

  // Next, merge together groups going into the same skeleton vertex
  for(auto hid : m_path.mergeHyperarcs) {
    if(m_cachedTrajs.count(hid)) {
      if(this->m_debug)
        std::cout << "Already have trajectory for hid " << hid << std::endl;

      m_path.groundedSolution.insert(m_skeletonToGrounded.at(hid));
      continue;
    }

    auto& hyp = m_skeleton->GetHyperarc(hid);
    if(this->m_debug) {
      std::cout << "Sampling trajectory for merge hid " << hid << std::endl;
      std::cout << "Tail: " << hyp.tail << ", Head: " << hyp.head << std::endl;
    }

    auto cedge = hyp.property.edge;
    auto group = cedge.GetGroup();
    auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(group);

    auto start = GroupCfgType(grm);
    std::unordered_map<Robot*, std::pair<bool, HID>> inHIDs;
    for(auto robot : group->GetRobots()) {
      auto inHID = m_path.predecessors.at(hid).at(robot);
      inHIDs[robot] = inHID;
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
    auto outHID = std::make_pair(true, SIZE_MAX);
    for(auto oh : m_skeleton->GetOutgoingHyperarcs(head)) {
      auto& outArc = m_skeleton->GetHyperarcType(oh);
      if(outArc.type == HyperskeletonArcType::Movement and 
                                           m_path.movementHyperarcs.count(oh)) {
        target = outArc.startRep.first->GetVertex(outArc.startRep.second);
        outHID = std::make_pair(true, oh);
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
        outHID = outerHID;

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
      t.SetLabel("Merge");

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

    bool failed = false;

    SetVirtualExcept(group);
    auto s = this->GetMPLibrary()->GetMPStrategy(m_trajStrategy);
    s->ResetGrowthOptions();
    try {
      this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
          this->GetMPSolution(), m_trajStrategy, LRand(),
          this->GetNameAndLabel()+"::SampleTrajectories");
    } catch(std::exception& _e) {
      failed = true;
    }

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size()) {
      if(this->m_debug)
        std::cout << "Failed merge hid " << hid << std::endl;
      failed = true;
    }

    if(failed) {
      // Mark the incoming (and outgoing) hyperarcs as failed
      std::unordered_map<Robot*, std::pair<VID, VID>> edges;
      for(auto iter : inHIDs) {
        auto robot = iter.first;
        auto bh = iter.second;
        if(!bh.first) {
          auto vid = m_skeleton->GetVertexType(bh.second).GetVID(robot);
          edges[robot] = std::make_pair(vid, vid);
          continue;
        }

        auto arc = m_skeleton->GetHyperarcType(bh.second);
        auto edge = arc.edge;
        auto ed = edge.GetEdgeDescriptor(robot);
        auto startVID = std::min(ed.source(), ed.target());
        auto targetVID = std::max(ed.source(), ed.target());
        edges[robot] = std::make_pair(startVID, targetVID);
      }
      m_failedEdges.push_back(edges);

      if(outHID.second == SIZE_MAX)
        return false;

      std::unordered_map<Robot*, std::pair<VID, VID>> outEdges;
      if(!outHID.first) {
        auto vert = m_skeleton->GetVertexType(outHID.second);
        for(auto robot : vert.GetGroup()->GetRobots()) {
          auto vid = vert.GetVID(robot);
          outEdges[robot] = std::make_pair(vid, vid);
        }
      } else {
        auto arc = m_skeleton->GetHyperarcType(outHID.second);
        auto edge = arc.edge;
        for(auto robot : edge.GetGroup()->GetRobots()) {
          auto ed = edge.GetEdgeDescriptor(robot);
          auto startVID = std::min(ed.source(), ed.target());
          auto targetVID = std::max(ed.source(), ed.target());
          outEdges[robot] = std::make_pair(startVID, targetVID);
        }
      }
      m_failedEdges.push_back(outEdges);

      this->GetMPLibrary()->SetGroupTask(m_wholeTask);
      SetVirtualExcept();
      return false;
    }

    auto ghid = AddTransitionToGroundedHypergraph(hyp.tail,hyp.head,path,task);
    m_cachedTrajs.insert(hid);
    m_skeletonToGrounded[hid] = ghid;
    m_path.groundedSolution.insert(ghid);
  }

  // Next, split apart groups going onto different skeleton edges
  for(auto hid : m_path.splitHyperarcs) {
    if(m_cachedTrajs.count(hid)) {
      if(this->m_debug)
        std::cout << "Already have trajectory for hid " << hid << std::endl;

      m_path.groundedSolution.insert(m_skeletonToGrounded.at(hid));
      continue;
    }

    if(this->m_debug)
      std::cout << "Sampling trajectory for split hid " << hid << std::endl;

    auto hyp = m_skeleton->GetHyperarc(hid);
    auto cedge = hyp.property.edge;
    auto group = cedge.GetGroup();
    auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(group);

    // Get the incoming hyperarc (can be movement, decouple, or merge)
    GroupCfgType start;
    std::unordered_map<Robot*, std::pair<VID, VID>> inEdges;
    for(auto t : hyp.tail) {
      for(auto ihs : m_skeleton->GetIncomingHyperarcs(t)) {
        auto& inhyp = m_skeleton->GetHyperarc(ihs);
        auto arc = inhyp.property;
        if(arc.type == HyperskeletonArcType::Movement and 
                            m_path.movementHyperarcs.count(ihs)) {
          start = arc.endRep.first->GetVertex(arc.endRep.second);

          for(auto robot : group->GetRobots()) {
            auto ed = arc.edge.GetEdgeDescriptor(robot);
            auto startVID = std::min(ed.source(), ed.target());
            auto targetVID = std::max(ed.source(), ed.target());
            inEdges[robot] = std::make_pair(startVID, targetVID);
          }
          break;
        }
        else if(arc.type == HyperskeletonArcType::Merge and 
                              m_path.mergeHyperarcs.count(ihs)) {
          start = arc.endRep.first->GetVertex(arc.endRep.second);
          break;
        }
        else if(arc.type == HyperskeletonArcType::Decouple and   
                                    m_path.decoupleHyperarcs.count(ihs)) {
          auto tt = *inhyp.tail.begin();
          auto iihs = *m_skeleton->GetIncomingHyperarcs(tt).begin();
          auto innerArc = m_skeleton->GetHyperarcType(iihs);
          start = innerArc.endRep.first->GetVertex(innerArc.endRep.second);
          break;
        }
      }
    }

    auto target = GroupCfgType(grm);
    std::unordered_map<Robot*, std::pair<VID, VID>> outEdges;
    for(auto robot : group->GetRobots()) {
      auto outHID = m_path.successors.at(hid).at(robot);
      if(!outHID.first) {
        // Going back into a waiting vertex
        if(!m_waitingReps.count(outHID.second)) {
          if(this->m_debug)
            std::cout << "Missing waiting rep at split, sampling " << outHID.second << std::endl;

          auto vertex = m_skeleton->GetVertexType(outHID.second);
          auto subgroup = vertex.GetGroup();
          auto grm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(subgroup);
          auto wvid = SpawnVertex(grm, vertex);
          m_waitingReps.emplace(outHID.second, std::make_pair(grm, wvid));
        }

        auto rep = m_waitingReps.at(outHID.second);
        auto vertex = rep.first->GetVertex(rep.second);
        Cfg cfg = vertex.GetRobotCfg(robot);
        target.SetRobotCfg(robot, std::move(cfg));

        auto skelVertex = m_skeleton->GetVertexType(outHID.second);
        auto svid = skelVertex.GetVID(robot);
        outEdges[robot] = std::make_pair(svid, svid);
      } else {
        auto& outArc = m_skeleton->GetHyperarcType(outHID.second);
        auto vertex = outArc.startRep.first->GetVertex(outArc.startRep.second);
        Cfg cfg = vertex.GetRobotCfg(robot);
        target.SetRobotCfg(robot, std::move(cfg));

        auto edge = outArc.edge;
        auto ed = edge.GetEdgeDescriptor(robot);
        auto startVID = std::min(ed.source(), ed.target());
        auto targetVID = std::max(ed.source(), ed.target());
        outEdges[robot] = std::make_pair(startVID, targetVID);
      }
    }

    // Plan a path between the start and target
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto r : group->GetRobots()) {
      auto t = MPTask(r);
      t.SetLabel("Split");

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

    bool failed = false;

    SetVirtualExcept(group);
    auto s = this->GetMPLibrary()->GetMPStrategy(m_trajStrategy);
    s->ResetGrowthOptions();
    try {
      this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
          this->GetMPSolution(), m_trajStrategy, LRand(), "SampleTrajectories");
    } catch(std::exception& _e) {
      failed = true;
    }

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size()) {
      if(this->m_debug)
        std::cout << "Failed split hid " << hid << std::endl;
      failed = true;
    }

    if(failed) {
      m_failedEdges.push_back(outEdges);
      if(inEdges.size())
        m_failedEdges.push_back(inEdges);
      
      this->GetMPLibrary()->SetGroupTask(m_wholeTask);
      SetVirtualExcept();
      return false;
    }

    auto ghid = AddTransitionToGroundedHypergraph(hyp.tail,hyp.head,path,task);
    m_cachedTrajs.insert(hid);
    m_skeletonToGrounded[hid] = ghid;
    m_path.groundedSolution.insert(ghid);
  }

  // Finally, merge together groups that are going in the opposite direction
  // along the same edge. Single node paths.
  for(auto hid : m_path.coupleHyperarcs) {
    if(m_cachedTrajs.count(hid)) {
      if(this->m_debug)
        std::cout << "Already have trajectory for hid " << hid << std::endl;

      m_path.groundedSolution.insert(m_skeletonToGrounded.at(hid));
      continue;
    }

    auto hyp = m_skeleton->GetHyperarc(hid);

    if(this->m_debug) {
      std::cout << "Constructing trajectory for opp merge hid " << hid << std::endl;
      std::cout << "Tail: " << hyp.tail << ", Head: " << hyp.head << std::endl;
    }

    auto cedge = hyp.property.edge;
    auto group = cedge.GetGroup();

    auto head = *hyp.head.begin();
    auto ohs = *(m_skeleton->GetOutgoingHyperarcs(head).begin());
    auto outArc = m_skeleton->GetHyperarcType(ohs);

    auto target = outArc.startRep.first->GetVertex(outArc.startRep.second);

    // Plan a path between the start and target - should be empty path
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto r : group->GetRobots()) {
      auto t = MPTask(r);
      t.SetLabel("OppMerge");

      // TODO make this less ugly
      // See if there is a movement hyperarc coming into this, if so, take
      // the start from there
      CfgType start;
      bool found = false;
      for(auto t : hyp.tail) {
        auto compVert = m_skeleton->GetVertexType(t);

        if(compVert.GetGroup()->VerifyRobotInGroup(r)) {
          for(auto ih : m_skeleton->GetIncomingHyperarcs(t)) {
            auto& arc = m_skeleton->GetHyperarcType(ih);

            if(arc.type == HyperskeletonArcType::Movement and 
              m_path.movementHyperarcs.count(ih)) {
              
              found = true;
              auto gcfg = arc.endRep.first->GetVertex(arc.endRep.second);
              start = gcfg.GetRobotCfg(r);
              break;
            }
          }
          break;
        }
      }

      if(!found) {
        start = target.GetRobotCfg(r);
        // Set a waiting representative just in case it's needed in the future
        for(auto t : hyp.tail) {
          auto compVert = m_skeleton->GetVertexType(t);

          if(compVert.GetGroup()->VerifyRobotInGroup(r)) {
            auto subgrm = this->GetMPLibrary()->GetMPSolution()->GetGroupRoadmap(compVert.GetGroup());
            auto waitCfg = GroupCfgType(subgrm);
            for(auto other : compVert.GetGroup()->GetRobots()) {
              waitCfg.SetRobotCfg(other, target.GetVID(other));
            }
            
            auto waitVID = subgrm->AddVertex(waitCfg);
            if(!m_waitingReps.count(t))
              m_waitingReps.emplace(t, std::make_pair(subgrm, waitVID));
            break;
          }
        }
      }

      // Add start constraints
      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,start));
      t.SetStartConstraint(std::move(startConstraint));

      // Add goal constraints
      auto goalCfg = target.GetRobotCfg(r);
      auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,goalCfg));
      t.AddGoalConstraint(std::move(goalConstraint));

      task->AddTask(t);
    } 

    SetVirtualExcept(group);
    auto s = this->GetMPLibrary()->GetMPStrategy(m_trajStrategy);
    s->ResetGrowthOptions();
    this->GetMPLibrary()->Solve(this->GetMPProblem(), task.get(), 
        this->GetMPSolution(), m_trajStrategy, LRand(), "SampleTrajectories");

    // Check if the plan was successful
    auto path = this->GetMPSolution()->GetGroupPath(group);
    if(!path->VIDs().size())
      throw RunTimeException(WHERE) << "Failed couple :(";

    // Extract path from solution
    auto ghid = AddTransitionToGroundedHypergraph(hyp.tail,hyp.head,path,task);
    m_cachedTrajs.insert(hid);
    m_skeletonToGrounded[hid] = ghid;
    m_path.groundedSolution.insert(ghid);
  }

  this->GetMPLibrary()->SetGroupTask(m_wholeTask);
  SetVirtualExcept();
  return true;
}

void
WoDaSH::
SetVirtualExcept(RobotGroup* _group) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SetVirtualExcept");

  // If no group provided, set all to non-virtual
  if(_group == nullptr) {
    for(auto robot : m_wholeGroup->GetRobots())
      robot->SetVirtual(false);
    return;
  }

  // Otherwise, set robots not in the group to virtual
  for(auto robot : m_wholeGroup->GetRobots())
    robot->SetVirtual(true);
  
  for(auto robot : _group->GetRobots())
    robot->SetVirtual(false);
}

RobotGroup*
WoDaSH::
AddGroup(std::vector<Robot*> _robots) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::AddGroup");

  auto group = this->GetMPProblem()->AddRobotGroup(_robots, "");
  this->GetMPLibrary()->GetMPSolution()->AddRobotGroup(group);

  if(this->GetMPProblem()->GetTasks(group).size() < 1) {
    std::unique_ptr<GroupTask> gt = std::unique_ptr<GroupTask>(new GroupTask(group));
    for (auto robot : _robots)
      gt->AddTask(*m_taskMap[robot]);
    this->GetMPProblem()->AddTask(std::move(gt));

    auto task = this->GetMPProblem()->GetTasks(group).at(0);
    m_groupTaskMap[group] = task.get();
  }

  return group;
}

WoDaSH::HID
WoDaSH::
AddTransitionToGroundedHypergraph(std::set<VID> _tail, std::set<VID> _head,
  GroupPathType* _path, std::shared_ptr<GroupTask> _task) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::AddTransitionToGroundedHypergraph");

  if(this->m_debug) {
    std::cout << "Adding grounded hyperarc between " << _tail << " and " << _head << std::endl;
  }

  auto gh = dynamic_cast<GroundedHypergraph*>(
    this->GetStateGraph(m_groundedHypergraphLabel).get());

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
  auto ghid = gh->AddTransition(groundedTail,groundedHead,transition);
  return ghid;
}


WoDaSH::HID
WoDaSH::
AddTransitionToGroundedHypergraph(std::set<RepresentativeVertex> _tail, 
  std::set<RepresentativeVertex> _head, GroupPathType* _path, 
  std::shared_ptr<GroupTask> _task) {
  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::AddTransitionToGroundedHypergraph");
  
  auto gh = dynamic_cast<GroundedHypergraph*>(
    this->GetStateGraph(m_groundedHypergraphLabel).get());

  std::set<VID> groundedTail;
  for(auto vertex : _tail) {
    auto vid = gh->AddVertex(vertex);
    groundedTail.insert(vid);
  }

  std::set<VID> groundedHead;
  for(auto vertex : _head) {
    auto vid = gh->AddVertex(vertex);
    groundedHead.insert(vid);
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
  auto ghid = gh->AddTransition(groundedTail,groundedHead,transition);
  return ghid;
}

void
WoDaSH::
ConvertToPlan(std::unordered_set<HID> _path) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConvertToPlan");

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_groundedHypergraphLabel).get());

  if(this->m_debug) {
    std::cout << "Grounded Hypergraph: " << std::endl;
    gh->Print();
  }

  auto path = OrderPath(_path);

  // Note: This implementation assumes that the input path is ordered already
  // and includes hyperarcs to/from the virtual source/sink
  // Create the first task from the source
  auto top = std::shared_ptr<SemanticTask>(new SemanticTask());
  Decomposition* decomp = new Decomposition(top);

  // Map of initial tasks for each group and a flag indicating if it has 
  // been used as a precedence constraint for any tasks yet.
  std::unordered_map<RobotGroup*,std::pair<bool,SemanticTask*>> initialTasks;

  auto hyperarc = gh->GetHyperarc(path.at(0));
  for(auto vid : hyperarc.head) {
    auto vertex = gh->GetVertex(vid);
    auto grm = vertex.first;
    auto group = grm->GetGroup();
    auto gcfg = grm->GetVertex(vertex.second);
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto robot : group->GetRobots()) {
      auto cfg = gcfg.GetRobotCfg(robot);
      auto start = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,cfg));
      auto goal = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,cfg));
      MPTask t(robot);
      t.SetLabel("StartConstraint");
      t.SetStartConstraint(std::move(start));
      t.AddGoalConstraint(std::move(goal));
      task->AddTask(t);
    }
    const std::string label = "vid" + std::to_string(vid) + ":InitialPath"; 
    auto st = std::shared_ptr<SemanticTask>(new SemanticTask(label,top.get(),decomp,
        SemanticTask::SubtaskRelation::AND,false,true,task));
    decomp->AddTask(st);
    initialTasks[group] = std::make_pair(false,st.get());
  }
  path.erase(path.begin()); // remove the virtual source

  // Save last set of tasks in each hyperarc
  std::unordered_map<size_t, SemanticTask*> hyperarcTaskMap;

  for(auto hid : path) {
    if(this->m_debug)
      std::cout << "Creating semantic tasks for hyperarc: " << hid << std::endl;

    // Get grounded hypergraph hyperarc
    auto hyperarc = gh->GetTransition(hid);
    auto tail = gh->GetHyperarc(hid).tail;

    for(auto& groupTasks : hyperarc.taskSet) { // There should only be one
      if(groupTasks.size() > 1)
        throw RunTimeException(WHERE) << "More than 1 task in task set :(";

      auto& groupTask = *groupTasks.begin();
      const std::string label = std::to_string(hid) + ":"
                                + groupTask->GetRobotGroup()->GetLabel() + ":"
                                + groupTask->GetLabel();
      auto task = std::shared_ptr<SemanticTask>(new SemanticTask(label,top.get(),decomp,
                SemanticTask::SubtaskRelation::AND,false,true,groupTask));
      decomp->AddTask(task);
      hyperarcTaskMap[hid] = task.get();

      for(auto f : hyperarc.taskFormations[groupTask.get()])
        task->AddFormation(f);
      
      // Add dependencies for the hyperarcs that came before
      for(auto t : tail) {
        for(auto inhid : gh->GetIncomingHyperarcs(t)) {
          if(!_path.count(inhid))
            continue;

          if(hyperarcTaskMap.count(inhid)) {
            auto previous = hyperarcTaskMap[inhid];
            task->AddDependency(previous,SemanticTask::DependencyType::Completion);

            if(this->m_debug) {
              std::cout << "Added dependency on previous hid " << inhid << std::endl;
            }
          }
        }
      }
      // Check if group has been given initial dependency
      auto& init = initialTasks[groupTask->GetRobotGroup()];
      if(init.first or !init.second)
        continue;

      // If not, assign the dependency
      task->AddDependency(init.second,SemanticTask::DependencyType::Completion);
      init.first = true;

      if(this->m_debug)
        std::cout << "Added dependency on initial task " << init.second << std::endl;
    }
  }

  plan->SetDecomposition(decomp);
}

std::vector<WoDaSH::HID>
WoDaSH::
OrderPath(std::unordered_set<HID> _path) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::OrderPath");

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_groundedHypergraphLabel).get());

  std::unordered_set<HID> used;
  std::vector<HID> ordered;

  while(ordered.size() != _path.size()) {
    for(auto hid : _path) {
      if(used.count(hid))
        continue;

      // Check if all incoming to tail set are in the ordered path
      auto hyperarc = gh->GetHyperarc(hid);
      bool ready = true;
      for(auto vid : hyperarc.tail) {
        for(auto inhid : gh->GetIncomingHyperarcs(vid)) {
          if(!_path.count(inhid))
            continue;
          
          if(!used.count(inhid)) {
            ready = false;
            break;
          }
        }
        if(!ready)
          break;
      }

      // Skip if the entire tail set is not in the ordered path
      if(!ready)
        continue;

      ordered.push_back(hid);
      used.insert(hid);
    }
  }

  return ordered; 
}

/*------------------------------------------------------------*/
