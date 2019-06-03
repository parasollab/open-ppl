#ifndef DISJOINT_WORKSPACES_H_
#define DISJOINT_WORKSPACES_H_

#include "ITPlacementMethod.h"

#include "TMPLibrary/TMPTools/InteractionTemplate.h"

#include "Utilities/XMLNode.h"

class DisjointWorkspaces : public ITPlacementMethod {

  public:

    ///@name Construction
    ///@{
    
		DisjointWorkspaces() = default;

    DisjointWorkspaces(MPProblem* _problem);

    DisjointWorkspaces(XMLNode& _node);

    ~DisjointWorkspaces() = default;

		std::unique_ptr<ITPlacementMethod> Clone() override;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution,
                         MPLibrary* _library, TMPStrategyMethod* _tmpMethod) override;

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

