#ifndef MP_HANDOFF_TEMPLATE_H_
#define MP_HANDOFF_TEMPLATE_H_

#include <memory>
#include <string>
#include <vector>

#include "MPTask.h"
#include "MPProblem.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPLibrary/MPBaseObject.h"
#include "Utilities/XMLNode.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"

////////////////////////////////////////////////////////////////////////////////
/// This represents a Handoff Template, which stores the tasks required for
/// robots to perform a handoff.
////////////////////////////////////////////////////////////////////////////////
//template <typename MPTraits>
class MPHandoffTemplate {

  public:

    //typedef typename MPTraits::RoadmapType        Roadmap;

    ///@name Construction
    ///@{

    MPHandoffTemplate(MPProblem* _problem, XMLNode& _node); 

    ///@}
    
    ///@name Accessors
    ///@{
    
    std::vector<std::shared_ptr<MPTask>> GetTasks() const;
  
    std::vector<RoadmapGraph<Cfg,DefaultWeight<Cfg>>*> GetRoadmaps() const;

    std::string GetLabel() const;
    
    GMSPolyhedron GetHandoffPolyhedron() const;

    size_t GetMaxAttempts() const;

    RoadmapGraph<Cfg, DefaultWeight<Cfg>>* GetConnectedRoadmap() const; 

    std::vector<std::vector<size_t>>& GetDistinctRoadmaps();
    
    ///@}    
    ///@name Member Management
    ///@{
    
    void AddRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _roadmap);
    
    void AddHandoffCfg(Cfg _cfg, MPProblem* _problem);

    void SetHandoffPolyhedron(const Boundary* _boundary);

    void ConnectRoadmaps(Robot* _robot, MPProblem* _problem);

    ///@} 


  protected:

    MPProblem* m_problem{nullptr}; ///< The handoff template problem. 

    /// The boundary for the template environment.
    GMSPolyhedron m_handoffPolyhedron;

    ///The set of tasks that must be performed to handoff.
    std::vector<std::shared_ptr<MPTask>> m_tasks;

    ///The set of end configurations for each task.
    std::vector<Cfg> m_handoffCfgs;

    ///The set of paths created from solving each task.
    std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> m_roadmaps; 

    ///The combination of all roadmaps in m_roadmaps.
    RoadmapGraph<Cfg, DefaultWeight<Cfg>>* m_connectedRoadmap{nullptr};

    ///The handoff label
    std::string m_label;

    ///The number of attempts to try and place the template in the environment.
    size_t m_maxAttempts;

    ///The set of VIDs from each distinct roadmap in the connected roadmap.
    std::vector<std::vector<size_t>> m_distinctRoadmaps;
};

#endif
