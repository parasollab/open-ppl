#ifndef PPL_TEMPLATE_INTERACTIONS_H_
#define PPL_TEMPLATE_INTERACTIONS_H_

#include "InteractionStrategyMethod.h"

class Interaction;

class TemplateInteractions : public InteractionStrategyMethod {

  public:

    ///@name Local Types
    ///@{

    typedef InteractionStrategyMethod::State State;

    struct InteractionTemplate {
      State start;
      std::vector<std::string> stages;
      std::unordered_map<std::string,std::unique_ptr<MPSolution>> toStageSolutions;
    };

    ///@}
    ///@name Construction
    ///@{

    TemplateInteractions();

    TemplateInteractions(XMLNode& _node);

    virtual ~TemplateInteractions();

    ///@}
    ///@name Interface
    ///@{

    virtual void Initialize() override;

    virtual bool operator()(Interaction* _interaction, State& _start) override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    bool CreateTemplate(Interaction* _interaction);

    void TransformSolution(MPSolution* _old, MPSolution* _new);

    GroupCfg TransformGroupCfg(GroupCfg& _gcfg);

    Cfg TransformCfg(Cfg& _cfg);
    
    ///@}
    ///@name Internal State
    ///@{
    
    bool m_lazy{true};

    bool m_refine{true};

    std::string m_templateEnvironment;

    std::unordered_map<Interaction*,InteractionTemplate> m_templateMap; 
    ///@}
};

#endif
