#ifndef PMPL_GROUP_DECOUPLED_STRATEGY_H_
#define PMPL_GROUP_DECOUPLED_STRATEGY_H_

#include "GroupStrategyMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Runs a single-robot planner for each robot in the group, and manages
/// inter-robot conflicts at query time.
////////////////////////////////////////////////////////////////////////////////
class GroupDecoupledStrategy : public GroupStrategyMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType     GroupCfgType;
    typedef typename MPBaseObject::WeightType       WeightType;
    typedef typename MPBaseObject::RoadmapType      RoadmapType;
    typedef typename MPBaseObject::GroupWeightType  GroupWeightType;
    typedef typename RoadmapType::VID               VID;

    ///@}
    ///@name Construction
    ///@{

    GroupDecoupledStrategy();

    GroupDecoupledStrategy(XMLNode& _node);

    virtual ~GroupDecoupledStrategy() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    virtual void Finalize() override;

    ///@}

  private:

    ///@name
    ///@{

    std::string m_strategyLabel; ///< The individual strategy label.

    ///@}

};

#endif
