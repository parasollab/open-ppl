#include "WrenchAccessibilityStrategy.h"

#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPTools/WrenchAccessibilityTool.h"

/*------------------------------- Construction -------------------------------*/

WrenchAccessibilityStrategy::
WrenchAccessibilityStrategy() {
  this->SetName("WrenchAccessibilityStrategy");
}


WrenchAccessibilityStrategy::
WrenchAccessibilityStrategy(XMLNode& _node) : MPStrategyMethod(_node) {
  this->SetName("WrenchAccessibilityStrategy");

  m_watLabel = _node.Read("watLabel", true, "", "label of WrenchAccessibilityTool");

}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

void
WrenchAccessibilityStrategy::
Iterate() {
  WrenchAccessibilityTool* wat = this->GetMPLibrary()->GetMPTools()->GetWrenchAccessibilityTool(m_watLabel);

  wat->ComputeWrenchAccessibility();
}

/*----------------------------------------------------------------------------*/
