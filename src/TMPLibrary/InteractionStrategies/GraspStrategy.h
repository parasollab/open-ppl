#ifndef PPL_GRASP_STRATEGY_H_
#define PPL_GRASP_STRATEGY_H_

#include "IndependentPaths.h"

#include "MPProblem/MPTask.h"

class GraspStrategy : public IndependentPaths {

  public: 

    ///@name Local Types
    ///@{

    typedef TMPBaseObject::GroupCfgType GroupCfgType;
    typedef Condition::State            State;

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

  protected:

    ///@name Helper Functions
    ///@{

    virtual void AssignRoles(const State& _state, const std::vector<std::string>& _conditions) override;

    Cfg SampleObjectPose(Robot* _object, Interaction* _interaction);

    // Compute the EE Frame with respect to the world
    Transformation ComputeEEWorldFrame(const Cfg& _objectPose, const Transformation& _transform);

    std::unordered_map<Robot*,Transformation> ComputeEEFrames(Interaction* _interaction, 
                                                              std::map<Robot*,Cfg>& objectPoses,
                                                              size_t _stage);

    Cfg ComputeManipulatorCfg(Robot* _robot, Transformation& _transform);

    void SetEEDOF(Interaction* _interaction, Cfg& _cfg, const std::string& _stage);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_objectSMLabel; ///< Sampler method for object poses

    std::string m_objectVCLabel; ///< Validity checker for object poses

    size_t m_maxAttempts; ///< Max attempts to generate an object pose

    std::vector<std::unique_ptr<MPTask>> m_objectPoseTasks;

    bool m_doctorBaseOrientation{true};

    bool m_physicalDemo{false};

    ///@}

};

#endif
