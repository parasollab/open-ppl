#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

#include <memory>
#include <vector>

class Boundary;
class Cfg;
class Robot;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// An abstract base class representing the required interface for a constraint
/// on the state of a robot. The Constraint interface requires plain jane Cfg's
/// to ensure that they are usable by reactive agents.
////////////////////////////////////////////////////////////////////////////////
class Constraint {

  public:

    ///@name Construction
    ///@{

    /// Create a constraint for a robot.
    /// @param _r The robot to constrain.
    explicit Constraint(Robot* const _r);

    virtual ~Constraint();

    /// Construct a constraint of the appropriate type from an XML node.
    /// @param _r The robot to which the constraint applies.
    /// @param _node The XML node to parse.
    /// @return A constraint for _m of the type/parameters described by _node.
    static std::unique_ptr<Constraint> Factory(Robot* const _r, XMLNode& _node);

    /// Copy this constraint.
    /// @return A copy of this constraint.
    virtual std::unique_ptr<Constraint> Clone() const = 0;

    ///@}
    ///@name Constraint Interface
    ///@{

    /// Change the subject of this constraint.
    /// @param _r The new robot to constrain.
    virtual void SetRobot(Robot* const _r);

    /// Get a sampling boundary that describes the subset of CSpace allowed by
    /// this constraint.
    virtual const Boundary* GetBoundary() const = 0;

    /// Determine whether a given configuration of the robot satisfies this
    /// constraint.
    /// @param _c The configuration to check.
    /// @return   True if _c satisfies this constraint.
    virtual bool Satisfied(const Cfg& _c) const = 0;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr}; ///< The subject of this constraint.

    ///@}

};

#endif
