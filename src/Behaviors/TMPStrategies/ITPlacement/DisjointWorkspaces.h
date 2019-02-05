#ifndef DISJOINT_WORKSPACES_H_
#define DISJOINT_WORKSPACES_H_

#include "PlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class DisjointWorkspaces : public PlacementMethod {

  public:

    ///@name Construction
    ///@{

    DisjointWorkspaces(MPProblem* _problem);

    DisjointWorkspaces(MPProblem* _problem, XMLNode& _node);

    ~DisjointWorkspaces() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution,
                         MPLibrary* _library, Coordinator* _coordinator) override;

    void TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg);

    bool CheckLocation(Cfg _cfg, MPLibrary* _library, InteractionTemplate* _it);//,
                       //std::unordered_map<std::string, HandoffAgent*> _capabilityAgents);

    ///@}


  private:
    double m_precision;
    double GetRandomDouble(double _min, double _max);
    int m_maxAttempts;

};

#endif

