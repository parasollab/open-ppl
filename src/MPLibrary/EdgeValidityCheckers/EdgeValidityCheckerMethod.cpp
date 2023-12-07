#include "EdgeValidityCheckerMethod.h"

/*------------------------------ Construction --------------------------------*/

EdgeValidityCheckerMethod::
EdgeValidityCheckerMethod(XMLNode& _node) : MPBaseObject(_node) {

  this->SetName("EdgeValidityCheckerMethod");

  m_reportCollisions = _node.Read("reportCols", false, m_reportCollisions,
    "Report obstacles in collision with edge if true");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/


/*--------------------------- EdgeValidityChecker Interface -------------------------*/



/*----------------------------------------------------------------------------*/
