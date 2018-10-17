#ifndef MP_HANDOFF_TEMPLATE_H_
#define MP_HANDOFF_TEMPLATE_H_

#include <memory>
#include <string>
#include <vector>

#include "MPTask.h"
#include "MPProblem.h"
#include "ConfigurationSpace/Cfg.h"
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

    std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> GetRoadmaps();

    RoadmapGraph<Cfg, DefaultWeight<Cfg>>* GetConnectedRoadmap() const; 

    std::vector<std::vector<size_t>>& GetDistinctRoadmaps();

    //Center Point of Handoff Locations
    std::vector<Cfg> GetLocations();

    //Positions of Robots for Handoff Locations
    std::vector<Cfg> GetPositions();

    bool SavedPaths();
    
    ///@}    
    ///@name Member Management
    ///@{
    
    void AddRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _roadmap);

    void AddPath(std::vector<Cfg> _path, double _cost);
    
    void AddHandoffCfg(Cfg _cfg, MPProblem* _problem);

    void SetHandoffPolyhedron(const Boundary* _boundary);

    void ConnectRoadmaps(Robot* _robot, MPProblem* _problem);

    void FindLocations();

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

    ///The weight of the edge between interaction cfgs
    size_t m_interactionWeight{0};

    ///The set of VIDs from each distinct roadmap in the connected roadmap.
    std::vector<std::vector<size_t>> m_distinctRoadmaps;

    ///The locations for manually placed handoffs
    std::vector<Cfg> m_handoffLocations;

    bool m_savePaths;

    ///The paths for each of the agents involved in the interaction.
    std::vector<std::vector<Cfg>> m_distinctPaths;

    ///Costs of the corresponding paths
    std::vector<double> m_distinctPathCosts;

    ///The set of roadmaps without the corresponding paths
    std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> m_pathlessRoadmaps;

    std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> m_pathOnlyRoadmaps;
};

#endif
