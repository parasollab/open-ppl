#ifndef PPL_SIMULTANEOUS_MULTI_ARM_EVALUATOR_H_
#define PPL_SIMULTANEOUS_MULTI_ARM_EVALUATOR_H_

#include "TaskEvaluatorMethod.h"

#include "TMPLibrary/ActionSpace/Condition.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/StateGraphs/ObjectCentricModeGraph.h"

class SimultaneousMultiArmEvaluator : public TaskEvaluatorMethod {
  public:

    ///@name LocalTypes
    ///@{

    typedef Condition::State                          State;

    typedef ObjectCentricModeGraph::ObjectMode        ObjectMode;
    typedef ObjectCentricModeGraph::ObjectModeSwitch  ObjectModeSwitch;
    typedef ObjectCentricModeGraph::GraphType         GraphType;
    typedef GraphType::VID                            VID;

    ///@}
    ///@name Construction
    ///@{

    SimultaneousMultiArmEvaluator();

    SimultaneousMultiArmEvaluator(XMLNode& _node);

    virtual ~SimultaneousMultiArmEvaluator();

    ///@}
    ///@name Task Evaluator Interface

    virtual void Initialize() override;

    ///@}
  protected:

    ///@name Helper Functions
    ///@{

    ///Exectute
    ///@param _plan pointer
    ///@return True if exectuion is successful
    virtual bool Run(Plan* _plan = nullptr) override;

    VID SelectMode();

    std::set<VID> GetModeNeighbors(VID _vid);

    bool SampleTransition(VID _source, VID _target);

    void ConnectToExistingRoadmap(Interaction* _interaction, State& _state, bool _reverse);
    
    void AddToRoadmap(Cfg _cfg);

    ///@}
    ///@name Internal State
    ///@{

    size_t m_maxAttempts; ///< Number of attempts to sample a transition.

    std::string m_connectorLabel; ///< Label for connector method to use in connecting transitions.
    ///@}
};

#endif
