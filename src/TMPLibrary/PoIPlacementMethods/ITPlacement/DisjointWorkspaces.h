#ifndef DISJOINT_WORKSPACES_H_
#define DISJOINT_WORKSPACES_H_

#include "TMPLibrary/PoIPlacementMethods/ITPlacement/ITPlacementMethod.h"

#include "Utilities/XMLNode.h"

class DisjointWorkspaces : public ITPlacementMethod {

  public:

    ///@name Construction
    ///@{
    
		DisjointWorkspaces();

    DisjointWorkspaces(XMLNode& _node);

    ~DisjointWorkspaces() = default;

		std::unique_ptr<ITPlacementMethod> Clone() override;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution) override;

    void TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg);

    bool CheckLocation(Cfg _cfg, InteractionTemplate* _it);//,
                       //std::unordered_map<std::string, HandoffAgent*> _capabilityAgents);

    ///@}


  private:
    double m_precision;
    double GetRandomDouble(double _min, double _max);
    int m_maxAttempts;

};

#endif

