#ifndef CONSTRAINT_FACTORY_H_
#define CONSTRAINT_FACTORY_H_

class Constraint;
class Robot;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Helps build the appropriate constraint type from an XML node.
////////////////////////////////////////////////////////////////////////////////
struct ConstraintFactory {

  /// Construct a constraint of the appropriate type from an XML node describing
  /// any constraint.
  /// @param[in] _r The robot to which the constraint applies.
  /// @param[in] _node The XML node to parse.
  /// @return A constraint for _m of the type/parameters described by _node.
  Constraint* operator()(Robot* const _r, XMLNode& _node) const;

};

#endif
