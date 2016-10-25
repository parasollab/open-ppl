#ifndef STATE_H_
#define STATE_H_

#include "Cfg.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Cfgs
/// @brief Default state space definition.
///
/// State defines a configuration, velocity, and accelleration for each of the
/// degrees of freedom of a robot. This abstraction represents State space.
////////////////////////////////////////////////////////////////////////////////
class State : public Cfg {
  public:
    explicit State(size_t _index = 0);
    explicit State(const Vector3d& _v, size_t _index = 0);
    State(const Cfg& _other);
    State(const State& _other);
    virtual ~State() = default;

    State operator=(const State& _other);
    bool operator==(const State& _other) const;
    State operator+(const State& _s) const;
    State& operator+=(const State& _s);
    State operator-(const State& _s) const;
    State& operator-=(const State& _s);
    State operator-() const;
    State operator*(double _d) const;
    State& operator*=(double _d);
    State operator/(double _d) const;
    State& operator/=(double _d);

    const vector<double>& GetVelocity() const {return m_vel;}

    static double GetTimeRes() { return m_timeRes; }
    static void SetTimeRes(double _timeRes) { m_timeRes = _timeRes; }

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Apply control to state using 4th order Runge-Kutta (RK4)
    /// @param _u Control
    /// @param _dt Time step to integrate over
    /// @return Resulting state
    State Apply(const vector<double>& _u, double _dt);

    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;

  protected:
    virtual void GetRandomCfgImpl(Environment* _env, shared_ptr<Boundary> bb);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Apply a specific force and torque to a state to yield a new
    ///        velocity of the state
    /// @param _s State
    /// @param _force Force on center of mass
    /// @param _torque Torque on body
    /// @return Resulting velocity
    static State F(const State& _s, const Vector3d& _force,
        const Vector3d& _torque);

    vector<double> m_vel;

    static double m_timeRes;
};

#endif
