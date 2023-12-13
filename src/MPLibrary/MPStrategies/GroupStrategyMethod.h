#ifndef PMPL_GROUP_STRATEGY_METHOD_H_
#define PMPL_GROUP_STRATEGY_METHOD_H_

#include "MPStrategyMethod.h"

#include "ConfigurationSpace/GenericStateGraph.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/MPTask.h"
#include "MPLibrary/Samplers/SamplerMethod.h"
#include "Utilities/XMLNode.h"

////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for MPStrategies that plan for robot groups.
///
/// @todo Incorporate path constraints when generating the start and goal.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class GroupStrategyMethod : public MPStrategyMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType      GroupCfgType;
    typedef typename MPBaseObject::GroupRoadmapType  GroupRoadmapType;
    typedef typename GroupRoadmapType::VID           VID;
    typedef typename SamplerMethod::BoundaryMap      BoundaryMap;

    ///@}
    ///@name Construction
    ///@{

    GroupStrategyMethod() = default;

    GroupStrategyMethod(XMLNode& _node);

    virtual ~GroupStrategyMethod() = default;

    ///@}

  protected:

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Finalize() override;

    virtual size_t GenerateStart(const std::string& _samplerLabel) override;

    virtual std::vector<size_t> GenerateGoals(const std::string& _samplerLabel)
        override;

    ///@}
    ///@name Helpers
    ///@{

    /// Get a map from robot -> boundary for the current group task's start
    /// constraints.
    BoundaryMap GetStartBoundaryMap() const noexcept;

    /// Get a map from robot -> boundary for each of the current group task's
    /// goal constraints.
    std::vector<BoundaryMap> GetGoalBoundaryMaps() const noexcept;

    ///@}

};

#endif
