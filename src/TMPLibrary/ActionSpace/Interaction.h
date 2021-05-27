#ifndef PPL_INTERACTION_H_
#define PPL_INTERACTION_H_

#include "Action.h"

#include "MPLibrary/MPSolution.h"
#include "Traits/CfgTraits.h"

class Interaction : public Action {
  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;
		typedef MPSolutionType<MPTraits<Cfg,
            DefaultWeight<Cfg>>> MPSolution;

    ///@}
    ///@name Construction
    ///@{

    Interaction();

    Interaction(XMLNode& _node);

    ~Interaction();

    ///@}
    ///@name Interface
    ///@{

    virtual void Initialize() override;

    virtual bool Valid(const State& _state) override;

    ///@}
    ///@name Accessors
    ///@{

    const std::vector<std::string>& GetInterimConditions() const;

    MPSolution* GetToInterimSolution() const;

    MPSolution* GetToPostSolution() const;

    const std::string GetInteractionStrategyLabel() const;

    ///@}

  protected:
    ///@name Helper Functions
    ///@{

    virtual void SetClassName() override;

    virtual void ParseXMLNode(XMLNode& _node) override;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_isLabel; ///< Interaction Strategy Label

    std::unique_ptr<MPSolution> m_toInterimSolution;

    std::unique_ptr<MPSolution> m_toPostSolution;

    /// The set of conditions representing the intermediate stage
    /// of the interaction needed by the interaction strategy.
    std::vector<std::string> m_interimConditions;
    ///@}
};

#endif
