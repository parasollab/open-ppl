#ifndef GOAL_GRAPH_H
#define GOAL_GRAPH_H

#include <vector>

#include <containers/sequential/graph/graph.h>
#include "Vector.h"

#include "ConfigurationSpace/Path.h"
#include "Geometry/Boundaries/Boundary.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/Robot.h"

#include "Utilities/PMPLExceptions.h"


////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GoalMap : public stapl::sequential::graph<stapl::DIRECTED,
                          stapl::NONMULTIEDGES, std::shared_ptr<Constraint>,
                          PathType<MPTraits>>{

  public:

    ///@name Local Types
    ///@{

    typedef stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
        std::shared_ptr<Constraint>, PathType<MPTraits>>    GoalGraph;

    typedef typename GoalGraph::vertex_iterator                 iterator;
    typedef typename GoalGraph::const_vertex_iterator           const_iterator;

    typedef typename GoalGraph::edge_iterator                   edge_iterator;
    typedef typename GoalGraph::const_edge_iterator             const_edge_iterator;

    typedef typename GoalGraph::adj_edge_iterator               adj_edge_iterator;
    typedef typename GoalGraph::const_adj_edge_iterator         const_adj_edge_iterator;

    typedef typename GoalGraph::vertex_descriptor               vertex_descriptor;
    typedef typename GoalGraph::edge_descriptor                 edge_descriptor;

    typedef typename MPTraits::RoadmapType                      RoadmapType;
    typedef typename RoadmapType::VID                           VID;
    typedef typename MPTraits::Path                             Path;
    
    typedef typename MPTraits::MPLibrary                        MPLibrary;
    
    ///@}
    ///@name Construction
    ///@{

    GoalMap();

    GoalMap(Robot* _robot, std::string _queryMethod, MPLibrary* _library);

    virtual ~GoalMap();

    ///@}
    ///@name Vertex Accessors
    ///@{

    /// Get the number of vertices contained in the goal map.
    /// @return Number of vertices.
    const size_t GetNumVertices() const noexcept;

    //const Boundary& GetVertex(const size_t _i) const noexcept;

    //bool IsDepot(const size_t _i);

    /// Get the set of descriptors for the goal vertices.
    /// @return Set of descriptors for the goal vertices.
    std::vector<size_t>* GetGoalDescriptors();
    
    /// Get the set of descriptors for the depot vertices (used in TRP).
    /// @return Set of descriptors for the depot vertices.
    std::vector<size_t>* GetDepotDescriptors();
    
    //const vertex_descriptor GetDescriptor(const VID _vid) const
    //    noexcept;

    ///@}
    ///@name Path Accessors
    ///@{

    /// Get a particular path.
    /// @param _i1 Origin vertex descriptor.
    /// @param _i2 Destination vertex descriptor.
    /// @return Path with given origin and destination.
    const Path& GetPath(const size_t _i1, const size_t _i2) const
        noexcept;

    ///@}
    ///@name Problem Customization
    ///@{
    
    /// Add the depots needed for TRP->ATSP
    /// @param _workers Set of robots to use for depot locations.
    void AddDepots(std::vector<Robot*> _workers);
    
    ///@}
    ///@name Modifiers

    /// Write the goal graph to an output stream.
    void Print(std::ostream& _os) const;

    ///@}

  private:
  
    ///@name Helpers
    ///@{

    /// Find the shortest paths between the goals 
    void UpdateGoalMap();

    ///@}
    ///@name Internal State
    ///@{
   
    bool m_debug{true};

    std::string m_queryMethodLabel; /// denotes the query method utilized to find
                                    /// the shortest path between goals
    
    MPLibrary* m_library{nullptr};  /// used to calculate paths between goals
    Robot* m_robot{nullptr};        /// should be coordinator asigned all the goals
    //vector<VID> m_goals;            /// stores the goals for the roadmap
    vector<Path> m_intergoalPaths;  /// stores shortest paths between goals
    vector<edge_descriptor> m_pathDescriptors;

    vector<std::shared_ptr<Constraint>> m_goalConstraints;
    vector<std::shared_ptr<Constraint>> m_depotConstraints;

    vector<vertex_descriptor> m_goalDescriptors;
    vector<vertex_descriptor> m_depotDescriptors;
    
    vector<edge_descriptor> m_depotGoalPathDescriptors;
    vector<Path> m_depotGoalPaths;
    
    vector<edge_descriptor> m_goalDepotPathDescriptors;
    vector<Path> m_goalDepotPaths;
   
    vector<edge_descriptor> m_depotPathDescriptors;
    vector<Path> m_depotPaths;
    /// starting locations of all the worker robots
    //std::vector<CSpaceConstraint> m_startingPositions; 

    ///@}
};


/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GoalMap<MPTraits>::
GoalMap() = default;

template <typename MPTraits>
GoalMap<MPTraits>::
GoalMap(Robot* _robot, std::string _queryMethod, MPLibrary* _library){
  //m_roadmap = _roadmap;
  //m_goals = _goals;
  m_robot = _robot;
  m_queryMethodLabel = _queryMethod;
  m_library = _library;
  UpdateGoalMap();
}

template <typename MPTraits>
GoalMap<MPTraits>::
~GoalMap() = default;

/*----------------------------- Accessors ------------------------------------*/

template <typename MPTraits>
const size_t 
GoalMap<MPTraits>::
GetNumVertices() const noexcept {
  return m_depotDescriptors.size() + m_goalDescriptors.size();
}

template <typename MPTraits>
std::vector<size_t>* 
GoalMap<MPTraits>::
GetGoalDescriptors() {
  return &m_goalDescriptors;
}

template <typename MPTraits>
std::vector<size_t>* 
GoalMap<MPTraits>::
GetDepotDescriptors() {
  return &m_depotDescriptors;
}
/*template <typename MPTraits>
const typename Boundary& 
GoalMap<MPTraits>::
GetGoal(const size_t _i) const noexcept {
  return Boundary();
}*/

/*template <typename MPTraits>
const typename GoalMap<MPTraits>::vertex_descriptor 
GoalMap<MPTraits>::
GetDescriptor(const VID _vid) const noexcept {
  return vertex_descriptor();
}*/

template <typename MPTraits>
const typename GoalMap<MPTraits>::Path& 
GoalMap<MPTraits>::
GetPath(const size_t _i1, const size_t _i2) const noexcept {
  const_adj_edge_iterator edge;
  const_iterator vert;
  edge_descriptor ed(_i1, _i2);
  std::cout << "Calling find edge for " << _i1 << " " << _i2 <<std::endl;
  this->find_edge(ed, vert, edge);
  std::cout << "Found edge" << std::endl;
  return edge->property();
  
}

/*----------------------- Problem Customization ------------------------------*/

template <typename MPTraits>
void
GoalMap<MPTraits>::
AddDepots(std::vector<Robot*> _workers){
  std::cout << "adding depots for " << _workers.size() << " workers" << endl; 
  for(auto robot : _workers){
    std::unique_ptr<CSpaceConstraint> constraint(
          new CSpaceConstraint(robot,robot->GetDynamicsModel()->GetSimulatedState()));
    constraint->SetRobot(nullptr);
    m_depotConstraints.push_back(constraint->Clone());
  }
  for(auto& depot : m_depotConstraints){
    m_depotDescriptors.push_back(this->add_vertex(depot->Clone()));
  }
  //debug purposes
  for(auto depot : m_depotDescriptors){
    std::cout << "Depot Descriptor: " << depot << endl; 
  }
/*
  for(auto& depot : m_depotConstraints){
    for(auto& goal : m_goalConstraints){
      MPTask* task(new MPTask(m_robot));
      task->AddStartConstraint(std::move(depot->Clone()));
      task->AddGoalConstraint(std::move(goal->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
            m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_intergoalPaths.push_back(*m_library->GetPath());
    }
  }
*/

  for(size_t i = 0; i < m_depotConstraints.size(); i++){
    for(size_t j = 0 ; j < m_depotConstraints.size(); j++){
      if(i == j) continue;
      MPTask* task(new MPTask(m_robot));
      task->AddStartConstraint(std::move(m_depotConstraints[i]->Clone()));
      task->AddGoalConstraint(std::move(m_depotConstraints[j]->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_depotPaths.push_back(*m_library->GetPath());
      m_depotPathDescriptors.push_back(this->add_edge(
            m_depotDescriptors[i], m_depotDescriptors[j],*m_library->GetPath())); 
      if(m_debug){
        std::cout << "Path for: " << m_depotDescriptors[i] << ", "<<  m_depotDescriptors[j] 
                  << " has length: " << m_depotPaths.back().Length() << std::endl;
      }
    }
  }

/*
  for(auto& depot : m_depotConstraints){
    for(auto& goal : m_goalConstraints){
      MPTask* task(new MPTask(m_robot));
      task->AddStartConstraint(std::move(goal->Clone()));
      task->AddGoalConstraint(std::move(depot->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
            m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_intergoalPaths.push_back(*m_library->GetPath());
    }
  }

  for(auto path : m_intergoalPaths){
    m_goalDepotPathDescriptors.push_back(this->add_edge(path));
  }
*/
  
  for(size_t i = 0; i < m_depotConstraints.size(); i++){
    for(size_t j = 0 ; j < m_goalConstraints.size(); j++){
      MPTask* task(new MPTask(m_robot));
      task->AddStartConstraint(std::move(m_depotConstraints[i]->Clone()));
      task->AddGoalConstraint(std::move(m_goalConstraints[j]->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_depotGoalPaths.push_back(*m_library->GetPath());
      m_depotGoalPathDescriptors.push_back(this->add_edge(
            m_depotDescriptors[i], m_goalDescriptors[j],*m_library->GetPath())); 
      //debug purposes
      std::cout << "Depot Descriptor: " << m_depotDescriptors[i] << std::endl;
      std::cout << "Goal Descriptor: " << m_goalDescriptors[j] << std::endl;
      std::cout << "Distance: " << m_depotGoalPaths.back().Length() << std::endl;
      
      
      
      
      
      task = new MPTask(m_robot);
      task->AddStartConstraint(std::move(m_goalConstraints[j]->Clone()));
      task->AddGoalConstraint(std::move(m_depotConstraints[i]->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_goalDepotPaths.push_back(*m_library->GetPath());
      m_goalDepotPathDescriptors.push_back(this->add_edge(
            m_goalDescriptors[j], m_depotDescriptors[i],*m_library->GetPath())); 
      //debug purposes
      std::cout << "Goal Descriptor: " << m_goalDescriptors[j] << std::endl;
      std::cout << "Depot Descriptor: " << m_depotDescriptors[i] << std::endl;
      std::cout << "Distance: " << m_goalDepotPaths.back().Length() << std::endl;
    }
  }




  /*for(auto robot : _workers){
    std::unique_ptr<CSpaceConstraint> constraint(
            new CSpaceConstraint(robot,robot->GetDynamcsModel()->GetSimulatedState()));
    CSpaceConstraint copy = contraint->Clone();
    copy->SetRobot(nullptr);
    m_depots.push_back(this->add(*copy));

    for(auto goal : m_goals){
      MPTask* task(new MPTask(m_robot));
      task->AddGoalConstraint(goal->property());
      task->AddStartConstraint(copy);
      //find the path between the two goals
      m_library->Solve(m_library->GetMPProblem(),task,
        m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_intergoalPaths.push_back(this->add_edge(*m_library->GetPath()));

      task = new MPTask(m_robot);
      task->AddGoalConstraint(copy);
      task->AddStartConstraint(goal->property())
    }
  }
*/

}



/*template <typename MPTraits>
void
AddDepots(std::vector<Robot*> _workers) {
  std::vector<CSpaceConstraint> startingPositions; //starting locations of all the workers
  for(auto robot : _workers){
    CSpaceConstraint constraint(robot,robot->GetDynamcsModel()->GetSimulatedState());
    startingPositions.push_back(constraint);
  }
  //TODO Add boundaries of constraints (depots) to graph
  for(auto constraint : startingPositions){
    depots.push_back(this->add(*constraint->Boundary()));
    
    for(auto goal : m_goals){
      MPTask* task(new MPTask(m_robot));
      task->AddGoalConstraint(goal->property());
    }
  }

  //TODO Add terminals (fake depots)
  

  //TODO Add edges from goals to depots
  for(auto depot : m_depots){
    for(auto goal : m_goals){
      // set the start and end point for the new path
      MPTask* task(new MPTask(m_robot));
      task->AddGoalConstraint((depot->property()));
      task->AddStartConstraint();
      // find the path between the two goals
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_intergoalPaths.push_back(*m_library->GetPath());
    }
  }
  }

  //TODO Copy those edges to the terminals


  //TODO Add zero-weight edges from terminals to depots
  // Maybe leave these unconnected and leave out the terminals altogether here
  // and add them in the LKH Search tool and pass in the number of workers to
  // the tool and have it copy the last n---if graph is unordered it won't
  // work---unlesss I search for the nodes of the worker locations instead
  //
  // other option is have a path with zero weight instead
  // paths default have a length of zero
  // perhaps generate/copy any random path object then call clear to set length
  // to zero

}
*/

/*------------------------------ Helpers -------------------------------------*/


template <typename MPTraits>
void 
GoalMap<MPTraits>::
UpdateGoalMap() {
   
  std::cout << "Updating goal map" << std::endl;  
  for(auto task : m_library->GetMPProblem()->GetTasks(m_robot)){
    m_goalConstraints.push_back(task->GetGoalConstraints().front()->Clone());
  }
  for(auto& goal : m_goalConstraints){
    m_goalDescriptors.push_back(this->add_vertex(goal->Clone()));
  }
  /*
  for(auto& start : m_goalConstraints){
    for(auto& end : m_goalConstraints){
      if(start == end) continue;
      MPTask* task(new MPTask(m_robot));
      task->AddStartConstraint(std::move(start->Clone()));
      task->AddGoalConstraint(std::move(end->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_intergoalPaths.push_back(*m_library->GetPath());
    }
  }
  for(auto path : m_intergoalPaths){
    m_pathDescriptors.push_back(this->add_edge(path));
  }*/
  for(size_t i = 0; i < m_goalConstraints.size(); i++){
    for(size_t j = 0 ; j < m_goalConstraints.size(); j++){
      if(i == j) continue;
      MPTask* task(new MPTask(m_robot));
      task->AddStartConstraint(std::move(m_goalConstraints[i]->Clone()));
      task->AddGoalConstraint(std::move(m_goalConstraints[j]->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_intergoalPaths.push_back(*m_library->GetPath());
      m_pathDescriptors.push_back(this->add_edge(
            m_goalDescriptors[i], m_goalDescriptors[j],*m_library->GetPath())); 
    }
  }


}


/*----------------------------------------------------------------------------*/



#endif
