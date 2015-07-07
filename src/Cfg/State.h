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
    State(size_t _index = 0);
    State(const Cfg& _other);
    State(const State& _other);
    virtual ~State() {};

    State operator=(const State& _other);
    bool operator==(const State& _other) const;
    State operator+(const State& _s) const;
    State& operator+=(const State& _s);
    State operator*(double _d) const;
    State& operator*=(double _d);
    State operator/(double _d) const;
    State& operator/=(double _d);

    const vector<double>& GetVelocity() const {return m_vel;}

    State Apply(Environment* _env, const vector<double>& _u, double _dt);

    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;

  private:
    static State F(Environment* _env, const State& _s,
        const Vector3d& _force, const Vector3d& _torque);

    vector<double> m_vel;
};

#endif
