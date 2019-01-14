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
class InteractionInformation {

  public:

    //typedef typename MPTraits::RoadmapType        Roadmap;

    ///@name Construction
    ///@{

    InteractionInformation(MPProblem* _problem, XMLNode& _node);

    ///@}

    ///@name Accessors
    ///@{

    std::vector<std::shared_ptr<MPTask>> GetTasks() const;

    std::string GetLabel() const;

    size_t GetMaxAttempts() const;

    MPProblem* GetMPProblem() const;

    std::vector<std::shared_ptr<MPTask>>& GetInteractionTasks();

    double GetInteractionWeight() const;

    /// Adds an addition location to place an IT
    void AddTemplateLocation(Cfg _location);

    /// Gets the set of locations to place ITs
    std::vector<Cfg>& GetTemplateLocations();

    /// Gets the final position of robots at each of the IT locations
    std::vector<Cfg> GetInteractionPositions();

    bool SavedPaths();

    ///@}

  protected:

    MPProblem* m_problem{nullptr}; ///< The handoff template problem.

    ///The set of tasks that must be performed to handoff.
    std::vector<std::shared_ptr<MPTask>> m_tasks;

    ///The handoff label
    std::string m_label;

    ///The number of attempts to try and place the template in the environment.
    size_t m_maxAttempts;

    ///The weight of the edge between interaction cfgs
    size_t m_interactionWeight{0};

    ///The locations for manually placed handoffs
    std::vector<Cfg> m_handoffLocations;

    ///Indicates if the interaction template should save the entire paths of the
    ///interaction or just the final configurations.
    bool m_savePaths;
};

#endif
