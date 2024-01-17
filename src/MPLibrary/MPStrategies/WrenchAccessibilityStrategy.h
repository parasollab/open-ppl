#ifndef PMPL_WRENCH_ACCESSIBILITY_STRATEGY_H_
#define PMPL_WRENCH_ACCESSIBILITY_STRATEGY_H_

#include "MPStrategyMethod.h"
#include "Utilities/XMLNode.h"


////////////////////////////////////////////////////////////////////////////////
/// Calls the WrenchAccessibilityTool in order to get an accessibility dcore
/// for a system/
////////////////////////////////////////////////////////////////////////////////
class WrenchAccessibilityStrategy : public MPStrategyMethod {

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

#endif
