#ifndef NON_HOLONOMIC_BODY_H_
#define NON_HOLONOMIC_BODY_H_

#include "ActiveMultiBody.h"

class Control;

////////////////////////////////////////////////////////////////////////////////
/// An active multibody with controls and dynamic state.
////////////////////////////////////////////////////////////////////////////////
class NonHolonomicMultiBody : public ActiveMultiBody {

  public:

    ///@name Construction
    ///@{

    NonHolonomicMultiBody();

    NonHolonomicMultiBody(const NonHolonomicMultiBody&) = delete;
    NonHolonomicMultiBody& operator=(const NonHolonomicMultiBody&) = delete;

    ///@}
    ///@name MultiBody Info
    ///@{

    using MultiBody::MultiBodyType;

    /// Get the type for this MultiBody.
    virtual MultiBodyType GetType() const noexcept override {
      return MultiBodyType::NonHolonomic;
    }

    ///@}
    ///@name Controls
    ///@{

    const vector<double>& GetRandomControl() const;
    const vector<shared_ptr<Control>>& AvailableControls() const;

    ///@}
    ///@name State Info
    ///@{

    /// Sample random velocity in bounds.
    vector<double> GetRandomVelocity() const;

    double GetMaxLinearVelocity() const {return m_maxLinearVel;}
    double GetMaxAngularVelocity() const {return m_maxAngularVel;}

    /// @param _pos Configuration dofs
    /// @param _vel Configuration vels
    /// @param _b Workspace bounds
    /// @return True if configuration values are inside physical robot
    ///         constraints
    bool InSSpace(const vector<double>& _pos, const vector<double>& _vel,
        const Boundary* const _b);

    ///@}
    ///@name I/O
    ///@{

    virtual void Read(istream& _is, CountingStreamBuffer& _cbs);
    virtual void Write(ostream& _os);

    ///@}

  private:

    ///@name Internal State
    ///@{

    vector<shared_ptr<Control>> m_controls; ///< Available controls
    double m_maxLinearVel{numeric_limits<double>::max()}; ///< Max linear velocity
    double m_maxAngularVel{numeric_limits<double>::max()}; ///< Max angular velocity

    ///@}
};

#endif
