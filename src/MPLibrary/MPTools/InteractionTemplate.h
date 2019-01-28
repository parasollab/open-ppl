#ifndef INTERACTION_TEMPLATE_H_
#define INTERACTION_TEMPLATE_H_

#include <memory>
#include <string>
#include <vector>

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "MPLibrary/MPBaseObject.h"
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

    std::vector<RoadmapGraph<Cfg,DefaultWeight<Cfg>>*> GetRoadmaps() const;

    std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> GetRoadmaps();

    InteractionInformation* GetInformation();

    RoadmapGraph<Cfg, DefaultWeight<Cfg>>* GetConnectedRoadmap() const;

    std::vector<std::vector<size_t>>& GetDistinctRoadmaps();

    /// Positions of Robots for Handoff Locations in base template
    std::vector<Cfg> GetPositions();

    /// Positions of Robots for Handoff Locations at each handoff location
    std::vector<Cfg>& GetTranslatedPositions();

    ///@}
    ///@name Member Management
    ///@{

    void AddRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _roadmap);

    void AddPath(std::vector<Cfg> _path, double _cost);

    void AddHandoffCfg(Cfg _cfg, MPProblem* _problem);

    void SetHandoffPolyhedron(const Boundary* _boundary);

    void ConnectRoadmaps(Robot* _robot, MPProblem* _problem);

    std::pair<size_t,size_t> GetConnectingEdge();
    ///@}


  protected:

    InteractionInformation* m_information;

    ///The set of end configurations for each task.
    std::vector<Cfg> m_handoffCfgs;

    ///The set of paths created from solving each task.
    std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> m_roadmaps;

    ///The combination of all roadmaps in m_roadmaps.
    RoadmapGraph<Cfg, DefaultWeight<Cfg>>* m_connectedRoadmap{nullptr};

    ///The set of VIDs from each distinct roadmap in the connected roadmap.
    std::vector<std::vector<size_t>> m_distinctRoadmaps;

    ///The paths for each of the agents involved in the interaction.
    std::vector<std::vector<Cfg>> m_distinctPaths;

    ///Costs of the corresponding paths
    std::vector<double> m_distinctPathCosts;

    ///The set of roadmaps without the corresponding paths
    std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> m_pathlessRoadmaps;

    std::vector<RoadmapGraph<Cfg, DefaultWeight<Cfg>>*> m_pathOnlyRoadmaps;

    bool m_debug{false};

    std::vector<Cfg> m_translatedInteractionPositions;

    std::pair<size_t,size_t> m_connectingEdge;
};

#endif
