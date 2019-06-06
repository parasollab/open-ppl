#ifndef IT_CONNECTOR_H_
#define IT_CONNECTOR_H_

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "ConfigurationSpace/Weight.h"

#include "Traits/CfgTraits.h"


class InteractionTemplate;
class WorkspaceSkeleton;

class ITConnector {
  public:
    ///@name Typenames
    ///@{

    typedef MPLibraryType<MPTraits<Cfg,DefaultWeight<Cfg>>>  MPLibrary;
    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;

    ///@}
    ///@name Construction
    ///@{

    struct AdjustedInfo{
      Cfg*   m_connection;
      double m_summedDistance;
      bool   m_directConnection;
      bool   m_changed;
    };

    ITConnector(double _threshold, MPLibrary* _library);

		ITConnector(XMLNode& _node);

    ~ITConnector() = default;

    ///@}
    ///@name
    ///@{

    /// Assumes that all ITs provided contain the given capability.
    RoadmapGraph<Cfg, DefaultWeight<Cfg>>* ConnectInteractionTemplates(
                 std::vector<std::shared_ptr<InteractionTemplate>>& _ITs,
                 const std::string& _capability,
                 std::vector<Cfg>& _startAndGoal,
                 RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _megaRoadmap);

    ///@}

  private:
    ///@name Helper Functions
    ///@{

    std::vector<Cfg*> CalculateBaseDistances(std::vector<std::shared_ptr<InteractionTemplate>>& _ITs,
                                             const std::string& _capability,
                                             std::vector<Cfg>& _startAndGoal);

    void FindAlternativeConnections(std::vector<Cfg*>& _cfgs);

    void UpdateAdjustedDistances(Cfg* _cfg1, Cfg* _cfg2, std::vector<Cfg*> _cfgs);

    void CopyInTemplates(RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _graph,
                         std::vector<std::shared_ptr<InteractionTemplate>>& _ITs,
                         const std::string& _capability,
                         std::vector<Cfg>& _startAndGoal);

    void TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg);

    void ConnectTemplates(RoadmapGraph<Cfg,DefaultWeight<Cfg>>* _graph);

    void BuildSkeletons();


    double SkeletonPathWeight(typename WorkspaceSkeleton::adj_edge_iterator& _ei) const;

    /// Checks the capability skeleton of the corresponding type to see if the
    /// two cfgs are in connected free space.
    bool InConnectedWorkspace(Cfg _cfg1, Cfg _cfg2);
    ///@}
    ///@name Member Variables
    ///@{

    std::unordered_map<Cfg*,std::unordered_map<Cfg*,double>> m_baseDistances;

    std::list<std::pair<Cfg*,Cfg*>> m_connections;

    std::unordered_map<Cfg*,std::unordered_map<Cfg*,AdjustedInfo>> m_adjDist;

    MPLibrary* m_library;

    double m_threshold;

    bool m_debug{false};

    std::unordered_map<std::string,std::shared_ptr<WorkspaceSkeleton>> m_capabilitySkeletons;


    ///@}
};
#endif
