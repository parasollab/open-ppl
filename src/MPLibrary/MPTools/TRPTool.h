#ifndef TRP_TOOL_H_
#define TRP_TOOL_H_

#include "ConfigurationSpace/Path.h"
#include "MPLibrary/MPLibrary.h"
#include "MPProblem/GoalMap.h"
#include "MPProblem/Robot/Robot.h"
#include "MPLibrary/MPBaseObject.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluate a roadmap to find the optimal assignment of goals to a team of
/// robots. This solves the Multiple Depot, Multiple Traveling Salesman Problem
/// hereby refered to as the Traveling Robots Problem.
///
/// This implements the transformation of the TRP into a Atypical Traveling
/// Salesman Problem as described in the paper : 
//TODO insert paper name and authors
///
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TRPTool {

  public:

    typedef typename MPTraits::MPLibrary              MPLibrary;
    typedef typename MPTraits::GoalMapType            GoalMap;
    typedef typename GoalMap::vertex_descriptor       vertex_descriptor;
    typedef typename MPTraits::Path                   Path;
    typedef std::vector<std::vector<Path>>            PathSets;
    ///@}
    ///@name Construction
    ///@{
    

    TRPTool();

    TRPTool(XMLNode& _node); 
    
    virtual ~TRPTool()=default;
    
    ///@}
    ///@name Problem Interface
    ///@{


    /// Provide problem input information
    /// @param _robot Main robot that holds the set of tasks to be performed by
    ///               the group.
    /// @param _workers The group of robots that will be performing the set of
    ///                 tasks.
    void Initialize(Robot* _robot, std::vector<Robot*> _workers);
    
    /// Perform TRP search on the problem
    /// @return Set of paths with each index corresponding to index of robot in
    /// _worker input in Initialize.
    PathSets Search();
    
    ///@}
    ///@name Accessors
    ///@{

    const std::string& GetLabel();
    
    ///@}
    ///@name Setters
    ///@{

    void SetMPLibrary(MPLibrary* _library);

    ///@}
  private:
    
    ///@name helpers
    ///@{
    
    /// Adds the depots (robot locations) to the goal map.
    void AddDepots();

    /// Extracts the path set from the output of the LKH library.
    /// @params _pathVertices The indices of the paths taken by each robot in
    /// the solution.
    PathSets Extract(std::vector<std::vector<size_t>> _pathVertices);
    
    ///@}
    ///@name internal state 
    ///@{
   
    MPLibrary* m_library{nullptr};
    std::string m_queryMethod;
    Robot* m_robot{nullptr};
    std::vector<Robot*> m_workers;
    GoalMap m_goalMap;
    vector<vertex_descriptor> m_ATSPPath;
    std::string m_label;
    ///@}


};


/*------------------------------- Construction -------------------------------*/
template <typename MPTraits>
TRPTool<MPTraits>::
TRPTool() = default;



template <typename MPTraits>
TRPTool<MPTraits>::
TRPTool(XMLNode& _node){
  m_queryMethod = _node.Read("queryLabel", true, "", "TRP Query Method for Goal Map");
  m_label = _node.Read("label", true, "", "TRPTool label");
}


/*------------------------------ Problem Interface -----------------------------*/

template <typename MPTraits>
void
TRPTool<MPTraits>::
Initialize(Robot* _robot, std::vector<Robot*> _workers){
  //std::cout << "Initializing TRPTool" << std::endl;
  //std::cout << "Number of workers: " << _workers.size() << std::endl;
  m_robot = _robot;
  m_workers = _workers;
  //goal mapp constructor should initialize the map to include all of the goals
  //std::cout << "creating the goal map" << std::endl;
  m_goalMap = GoalMap(_robot, m_queryMethod, m_library);
  AddDepots();
}


template <typename MPTraits>
typename TRPTool<MPTraits>::PathSets
TRPTool<MPTraits>::
Search(){
  auto lkh = m_library->GetMPTools()->GetLKHSearch("lkhSearch");
  std::vector<std::vector<size_t>> pathVertices = lkh->SearchTRP(&m_goalMap);
  return Extract(pathVertices);
}

/*-------------------------------- Accessors ----------------------------------*/
template <typename MPTraits>
const std::string&
TRPTool<MPTraits>::
GetLabel(){
  return m_label;
}


/*--------------------------------- Setters -----------------------------------*/

template <typename MPTraits>
void
TRPTool<MPTraits>::
SetMPLibrary(MPLibrary* _library){
  m_library = _library;
}


/*--------------------------------- Helpers ----------------------------------*/
template <typename MPTraits>
void
TRPTool<MPTraits>::
AddDepots(){
  m_goalMap.AddDepots(m_workers);
}


template <typename MPTraits>
typename TRPTool<MPTraits>::PathSets
TRPTool<MPTraits>::
Extract(std::vector<std::vector<size_t>> _pathVertices){ 
  //each index is the set of paths for the corresponding worker
  TRPTool<MPTraits>::PathSets pathSets;
  //vertex set is the goal map vertex descriptor representation of the path 
  //for each worker as found by the LKH Library and transformation to/from ATSP
  for(auto vertexSet : _pathVertices){
    std::vector<typename MPTraits::Path> workerSet;
    for(size_t i = 0; i < vertexSet.size()-1; i++){
      workerSet.push_back(m_goalMap.GetPath(vertexSet[i], vertexSet[i+1]));
    }
    pathSets.push_back(workerSet);
  }
  return pathSets;
}

#endif



















