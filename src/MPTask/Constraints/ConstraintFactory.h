#ifndef CONSTRAINT_FACTORY_H_
#define CONSTRAINT_FACTORY_H_

#include "Constraint.h"
#include "CSpaceConstraint.h"
#include "WorkspaceConstraint.h"

#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"

class ActiveMultiBody;


////////////////////////////////////////////////////////////////////////////////
/// Helps build the appropriate constraint type from an XML node.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
struct ConstraintFactory {

  /// Construct a constraint of the appropriate type from an XML node describing
  /// any constraint.
  /// @param[in] _m The ActiveMultiBody to which the constraint applies.
  /// @param[in] _node The XML node to parse.
  /// @return A constraint for _m of the type/parameters described by _node.
  ConstraintType<MPTraits>* operator()(ActiveMultiBody* const _m, XMLNode& _node)
      const;

};


template <typename MPTraits>
ConstraintType<MPTraits>*
ConstraintFactory<MPTraits>::
operator()(ActiveMultiBody* const _m, XMLNode& _node) const {
  if(_node.Name() == "CSpaceConstraint")
    return new CSpaceConstraint<MPTraits>(_m, _node);

  else if(_node.Name() == "WorkspaceConstraint")
    return new WorkspaceConstraint<MPTraits>(_m, _node);

  else
    throw RunTimeException(WHERE, "Unrecognized constraint type '" +
        _node.Name() + "'.");
  return nullptr;
}

#endif
