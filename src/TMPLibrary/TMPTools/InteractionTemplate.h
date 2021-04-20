#ifndef INTERACTION_TEMPLATE_H_
#define INTERACTION_TEMPLATE_H_

#include <memory>
#include <string>
#include <vector>

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "ConfigurationSpace/GenericStateGraph.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "MPProblem/InteractionInformation.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/MPProblem.h"
#include "Utilities/XMLNode.h"


////////////////////////////////////////////////////////////////////////////////
/// This represents an Interaction Template, which stores the roadmaps for the
/// robots to perform an interaction (handoff).
///
/// @todo This should probably move to the configuration space directory since
///       it represents a structure which exists purely between two c-spaces.
////////////////////////////////////////////////////////////////////////////////
class InteractionTemplate {

  public:

    //typedef typename MPTraits::RoadmapType        Roadmap;

    ///@name Construction
    ///@{

    InteractionTemplate(InteractionInformation*);

    ///@}

    ///@name Accessors
    ///@{

    std::vector<GenericStateGraph<Cfg,DefaultWeight<Cfg>>*> GetRoadmaps() const;

    std::vector<GenericStateGraph<Cfg, DefaultWeight<Cfg>>*> GetRoadmaps();

    InteractionInformation* GetInformation();

    GenericStateGraph<Cfg, DefaultWeight<Cfg>>* GetConnectedRoadmap() const;

    std::vector<std::vector<size_t>>& GetDistinctRoadmaps();

    /// Positions of Robots for Handoff Locations in base template
    std::vector<Cfg> GetPositions();

    /// Paths of Robots for Handoff Locations in base template
    std::vector<std::vector<Cfg>> GetPaths();

    /// Positions of Robots for Handoff Locations at each handoff location
    std::vector<Cfg>& GetTranslatedPositions();

    /// Gets the transformed position of robot pairs at each of the IT locations. 
    /// First is the robot handing off. Second is robot receving.
    std::vector<std::pair<Cfg,Cfg>> GetTransformedPositionPairs();

    /// Paths of Robots for Handoff Locations at each handoff location
    std::vector<std::vector<Cfg>>& GetTranslatedPaths();

    ///@}
    ///@name Member Management
    ///@{

    void AddRoadmap(GenericStateGraph<Cfg, DefaultWeight<Cfg>>* _roadmap);

    void AddPath(std::vector<Cfg> _path/*, double _cost*/, MPProblem* _problem);

    void AddHandoffCfg(Cfg _cfg, MPProblem* _problem);

    void SetHandoffPolyhedron(const Boundary* _boundary);

    void ConnectRoadmaps(Robot* _robot, MPProblem* _problem);

    std::pair<size_t,size_t> GetConnectingEdge();
    ///@}


  protected:

    InteractionInformation* m_information;

    ///The set of end configurations for each task.
    std::vector<Cfg> m_handoffCfgs;

    ///The paths for each of the agents involved in the interaction.
    std::vector<std::vector<Cfg>> m_interactionPaths;

    ///The set of paths created from solving each task.
    std::vector<GenericStateGraph<Cfg, DefaultWeight<Cfg>>*> m_roadmaps;

    ///The combination of all roadmaps in m_roadmaps.
    GenericStateGraph<Cfg, DefaultWeight<Cfg>>* m_connectedRoadmap{nullptr};

    ///The set of VIDs from each distinct roadmap in the connected roadmap.
    std::vector<std::vector<size_t>> m_distinctRoadmaps;

    ///Costs of the corresponding paths
    std::vector<double> m_distinctPathCosts;

    ///The set of roadmaps without the corresponding paths
    std::vector<GenericStateGraph<Cfg, DefaultWeight<Cfg>>*> m_pathlessRoadmaps;

    std::vector<GenericStateGraph<Cfg, DefaultWeight<Cfg>>*> m_pathOnlyRoadmaps;

    bool m_debug{false};

    std::vector<Cfg> m_translatedInteractionPositions;

    std::vector<std::pair<Cfg,Cfg>> m_transformedInteractionPositionPairs;

    std::vector<std::vector<Cfg>> m_translatedInteractionPaths;

    std::pair<size_t,size_t> m_connectingEdge;
};

#endif
