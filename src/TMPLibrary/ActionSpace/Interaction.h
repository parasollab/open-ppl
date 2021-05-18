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

    virtual bool Valid(const State& _state) override;

    ///@}
    ///@name Accessors
    ///@{

    const std::vector<std::unique_ptr<Condition>>& 
                      GetInterimConditions() const;

    MPSolution* GetMPSolution() const;
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

    std::unique_ptr<MPSolution> m_mpSolution;

    /// The set of conditions representing the intermediate stage
    /// of the interaction needed by the interaction strategy.
    std::vector<std::unique_ptr<Condition>> m_interimConditions;
    ///@}
};

#endif
