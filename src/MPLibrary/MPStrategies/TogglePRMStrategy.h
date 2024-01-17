#ifndef PMPL_TOGGLE_PRM_STRATEGY_H_
#define PMPL_TOGGLE_PRM_STRATEGY_H_

#include "MPLibrary/MPStrategies/BasicPRM.h"


////////////////////////////////////////////////////////////////////////////////
/// Toggle PRM builds two roadmaps, one free and one obstacle. The information
/// discovered about each space is used to assist the mapping of the other
/// space.
///
/// Reference:
///   Jory Denny, Nancy Amato. "Toggle PRM: A Coordinated Mapping of C-free and
///   C-obstacle in Arbitrary Dimension." WAFR 2012.
///
/// @todo This can probably derive from BasicPRM to reduce duplication of common
///       code.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class TogglePRMStrategy : public BasicPRM {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType   RoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename BasicPRM::SamplerSetting    SamplerSetting;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::deque<std::pair<bool, Cfg>> ToggleQueue;

    ///@}
    ///@name Construction
    ///@{

    TogglePRMStrategy();

    TogglePRMStrategy(XMLNode& _node);

    virtual ~TogglePRMStrategy() = default;

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

    /// Sample new nodes and put them into the toggle queue.
    void GenerateNodes();

    /// Try to connect a new node to the correct roadmap.
    /// @param _valid Is the new node valid?
    /// @param _vid The new node's VID.
    void ConnectHelper(const bool _valid, const VID _vid);

    /// Add a configuration to the toggle queue.
    /// @param _cfg The configuration to enqueue. Must already be validated by a
    ///             collision-detection VC.
    void Enqueue(const Cfg& _cfg);

    ///@}
    ///@name Internal State
    ///@{


    std::vector<std::string> m_colConnectorLabels;
    std::string m_vcLabel;     ///< Validity checker for lazy samplers.

    bool m_priority{false};    ///< Give priority to valid nodes in the queue?

    ToggleQueue m_queue;   ///< Queue for sharing information between maps.

    ///@}

};

#endif
