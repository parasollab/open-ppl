#ifndef PPL_CONDITION_H_
#define PPL_CONDITION_H_

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "TMPLibrary/TMPBaseObject.h"

class RobotGroup;

class Condition : public TMPBaseObject {

  public:
    ///@name Local Types
    ///@{

    typedef GroupLocalPlan<Cfg>                        GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>  GroupRoadmapType;
    typedef size_t                                     VID;
    typedef std::unordered_map<RobotGroup*,
                     std::pair<GroupRoadmapType*,VID>> State;

    ///@}
    ///@name Construction
    ///@{

    Condition();

    Condition(XMLNode& _node);

    ~Condition();

    static std::unique_ptr<Condition> Factory(XMLNode& _node);

    ///@}
    ///@name Interface
    ///@{

    virtual RobotGroup* Satisfied(const State& _state) const;

    ///@}
    ///@name Accessors
    ///@{

    virtual bool IsUnique() const;

    ///@}

  protected:
    ///@name Internal State
    ///@{

    /// Flag that indicates that the robot group satisfying this condition
    /// cannot satisfy any other condition.
    bool m_unique{false};

    ///@}

};

#endif
