#ifndef CONSTRAINT_FACTORY_H_
#define CONSTRAINT_FACTORY_H_

class ActiveMultiBody;
class Constraint;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Helps build the appropriate constraint type from an XML node.
////////////////////////////////////////////////////////////////////////////////
struct ConstraintFactory {

  /// Construct a constraint of the appropriate type from an XML node describing
  /// any constraint.
  /// @param[in] _m The ActiveMultiBody to which the constraint applies.
  /// @param[in] _node The XML node to parse.
  /// @return A constraint for _m of the type/parameters described by _node.
  Constraint* operator()(ActiveMultiBody* const _m, XMLNode& _node) const;

};

#endif
