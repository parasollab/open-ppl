#ifndef PPL_INTERACTION_H_
#define PPL_INTERACTION_H_

#include "Action.h"

#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "MPLibrary/MPSolution.h"
#include "Traits/CfgTraits.h"

class Interaction : public Action {
  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;
		typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>> GroupPathType;


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

    const std::string GetInteractionStrategyLabel() const;

    MPSolution* GetToInterimSolution() const;

    MPSolution* GetToPostSolution() const;

    std::unique_ptr<MPSolution>&& ExtractToInterimSolution();

    std::unique_ptr<MPSolution>&& ExtractToPostSolution();

    void SetToInterimPath(GroupPathType* _path);

    GroupPathType* GetToInterimPath();

    void SetToPostPath(GroupPathType* _path);

    GroupPathType* GetToPostPath();

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

    /// The set of conditions representing the intermediate stage
    /// of the interaction needed by the interaction strategy.
    std::vector<std::string> m_interimConditions;

    std::unique_ptr<MPSolution> m_toInterimSolution;

    std::unique_ptr<MPSolution> m_toPostSolution;

    GroupPathType* m_toInterimPath;

    GroupPathType* m_toPostPath;

    ///@}
};

#endif
