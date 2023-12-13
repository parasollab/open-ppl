#ifndef PMPL_BASIC_PRM_H_
#define PMPL_BASIC_PRM_H_

#include "MPStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Utilities/XMLNode.h"


////////////////////////////////////////////////////////////////////////////////
/// Basic PRM algorithm.
///
/// Reference:
///   Lydia E. Kavraki and Petr Svestka and Jean-Claude Latombe and Mark H.
///   Overmars. "Probabilistic Roadmaps for Path Planning in High-Dimensional
///   Configuration Spaces". TRO 1996.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class BasicPRM : public MPStrategyMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID           VID;

    ///@}
    ///@name Local Types
    ///@{

    /// Settings for a specific sampler.
    struct SamplerSetting {
      std::string label;
      size_t count;
      size_t attempts;
    };

    ///@}
    ///@name Construction
    ///@{

    BasicPRM();

    BasicPRM(XMLNode& _node);

    virtual ~BasicPRM() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
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

    std::string m_inputMapFilename; ///< Input roadmap to initialize map

    ///@}
    ///@name Fix-base hacks
    ///@{
    /// To be removed when we properly implement path constraints or perhaps a
    /// sampler-method option.

    bool m_fixBase{false};  ///< Keep the base fixed to the start cfg?
    /// An optional sampling boundary for fixing the base.
    std::unique_ptr<Boundary> m_samplingBoundary;

    ///@}

};

#endif
