#ifndef PPL_GRASP_STRATEGY_H_
#define PPL_GRASP_STRATEGY_H_

#include "InteractionStrategyMethod.h"

#include "MPProblem/MPTask.h"

class GraspStrategy : public InteractionStrategyMethod {

  public: 

    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Construction
    ///@{

    GraspStrategy();

    GraspStrategy(XMLNode& _node);

    ~GraspStrategy();

    ///@}
    ///@name Interface 
    ///@{

    virtual bool operator()(Interaction* _interaction, State& _start) override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    virtual void AssignRoles(const State& _state, const std::vector<std::string>& _conditions) override;

    Cfg SampleObjectPose(Robot* _object, Interaction* _interaction);

    // Compute the EE Frame with respect to the world
    Transformation ComputeEEWorldFrame(const Cfg& _objectPose, const Transformation& _transform);

    Cfg ComputeManipulatorCfg(Robot* _robot, Transformation& _transform);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_objectSMLabel; ///< Sampler method for object poses

    std::string m_objectVCLabel; ///< Validity checker for object poses

    size_t m_maxAttempts; ///< Max attempts to generate an object pose

    std::vector<std::unique_ptr<MPTask>> m_objectPoseTasks;

    bool m_doctorBaseOrientation{true};

    ///@}

};

#endif
