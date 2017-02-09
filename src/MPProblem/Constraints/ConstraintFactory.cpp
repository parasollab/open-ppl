#include "ConstraintFactory.h"

#include "Constraint.h"
#include "CSpaceConstraint.h"
#include "WorkspaceConstraint.h"

#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


Constraint*
ConstraintFactory::
operator()(ActiveMultiBody* const _m, XMLNode& _node) const {
  if(_node.Name() == "CSpaceConstraint")
    return new CSpaceConstraint(_m, _node);

  else if(_node.Name() == "WorkspaceConstraint")
    return new WorkspaceConstraint(_m, _node);

  else
    throw RunTimeException(WHERE, "Unrecognized constraint type '" +
        _node.Name() + "'.");
  return nullptr;
}
