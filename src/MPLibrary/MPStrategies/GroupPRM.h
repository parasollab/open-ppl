#ifndef PMPL_GROUP_PRM_H_
#define PMPL_GROUP_PRM_H_

#include "GroupStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Utilities/XMLNode.h"


////////////////////////////////////////////////////////////////////////////////
/// Basic PRM algorithm for multirobot teams using composite c-space.
///
/// @todo Create option for decoupled planning.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class GroupPRM : public GroupStrategyMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType      GroupCfgType;
    typedef typename MPBaseObject::GroupRoadmapType  GroupRoadmapType;
    typedef typename GroupRoadmapType::VID           VID;

    ///@}
    ///@name Local Types
    ///@{

    /// Settings for a specific sampler.
    struct SamplerSetting {
      std::string label;   ///< The sampler label.
      size_t count;        ///< The number of samples per call.
      size_t attempts;     ///< The number of attempts per sample.
    };

    ///@}
    ///@name Construction
    ///@{

    GroupPRM();

    GroupPRM(XMLNode& _node);

    virtual ~GroupPRM() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}

  protected:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    ///@}
    ///@name Helpers
    ///@{

    /// Sample and add configurations to the roadmap.
    /// @return The generated VIDs for the successful samples.
    std::vector<VID> Sample();

    /// Connect nodes in the roadmap.
    /// @param _vids A set of node VIDs to connect to the rest of the roadmap.
    void Connect(const std::vector<VID>& _vids);

    ///@}
    ///@name Internal State
    ///@{

    /// Sampler labels with number and attempts of sampler.
    std::vector<SamplerSetting> m_samplers;
    /// Connector labels for node-to-node.
    std::vector<std::string> m_connectorLabels;

    ///@}

};

#endif
