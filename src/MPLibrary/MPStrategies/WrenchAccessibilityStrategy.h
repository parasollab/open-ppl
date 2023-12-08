#ifndef PMPL_WRENCH_ACCESSIBILITY_STRATEGY_H_
#define PMPL_WRENCH_ACCESSIBILITY_STRATEGY_H_

#include "MPStrategyMethod.h"
#include "MPLibrary/MPTools/WrenchAccessibilityTool.h"
#include "Utilities/XMLNode.h"



////////////////////////////////////////////////////////////////////////////////
/// Calls the WrenchAccessibilityTool in order to get an accessibility dcore
/// for a system/
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class WrenchAccessibilityStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    ///@}
    ///@name Local Types
    ///@{


    ///@}
    ///@name Construction
    ///@{

    WrenchAccessibilityStrategy();

    WrenchAccessibilityStrategy(XMLNode& _node);

    virtual ~WrenchAccessibilityStrategy() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    ///@}

  protected:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    ///@}
    ///@name Helpers
    ///@{

    ///@}
    ///@name Internal State
    ///@{

    std::string m_watLabel; ///< label of WrenchAccessibilityTool

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
WrenchAccessibilityStrategy<MPTraits>::
WrenchAccessibilityStrategy() {
  this->SetName("WrenchAccessibilityStrategy");
}


template <typename MPTraits>
WrenchAccessibilityStrategy<MPTraits>::
WrenchAccessibilityStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("WrenchAccessibilityStrategy");

  m_watLabel = _node.Read("watLabel", true, "", "label of WrenchAccessibilityTool");

}

/*-------------------------- MPBaseObject Overrides --------------------------*/



/*------------------------ MPStrategyMethod Overrides ------------------------*/


template <typename MPTraits>
void
WrenchAccessibilityStrategy<MPTraits>::
Iterate() {
  WrenchAccessibilityTool<MPTraits>* wat = this->GetMPTools()->GetWrenchAccessibilityTool(m_watLabel);

  wat->ComputeWrenchAccessibility();
}

/*--------------------------------- Helpers ----------------------------------*/

/*----------------------------------------------------------------------------*/

#endif
