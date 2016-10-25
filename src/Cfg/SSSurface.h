#ifndef SS_SURFACE_H_
#define SS_SURFACE_H_

#include "CfgSurface.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup SurfaceCfgs
/// @brief State based configuration restricted to be on a surface
///
/// A derived class from CfgSurface that uses state space and dynamics
/// properties. For now, these will approximate a Reeds-Shepp style car.
////////////////////////////////////////////////////////////////////////////////
class SSSurface : public CfgSurface {
  public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Constructors and Destructor
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    //===================================================================
    /**@name  Constructors and Destructor*/
    //===================================================================
    SSSurface();
    SSSurface(const SSSurface& _rhs);
    SSSurface(const CfgSurface& _rhs);

    ///Do nothing destructor
    virtual ~SSSurface(){};

#ifdef _PARALLEL
    void define_type(stapl::typer &t)
    {
      Cfg::define_type(t);
    }
#endif

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Access Methods : Retrive and set related information of this class
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;

    virtual void GetRandomCfgImpl(Environment* _env,shared_ptr<Boundary> _bb);

    template<class DistanceMetricPointer>
      void GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm=true);

    SSSurface& operator=(const SSSurface& _rhs);
   // virtual void WeightedSum(const Cfg&, const Cfg&, double _weight = 0.5);
    ///////////////////////////////////////////////////////////////////////////////////////////


    ///Increase every value in this instance in each dimention by the value in _increment
    virtual void IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks,
        double _positionRes, double _orientationRes);
    virtual void FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks);

    virtual const string GetName() const;

    //get rotations about axes (in radians, between -PI and PI)
    virtual Vector3d GetRotations() const{return m_orientation;}
    virtual Vector2d GetVelocity() const{return m_velocity;}
    virtual Vector2d GetAcceleration() const {return m_acceleration;}

    double GetRotX() const{return m_orientation[0];}
    double GetRotY() const{return m_orientation[1];}
    double GetRotZ() const{return m_orientation[2];}

    double GetVelX() const{return m_velocity[0];}
    double GetVelY() const{return m_velocity[1];}

    double GetAccelX() const{return m_acceleration[0];}
    double GetAccelY() const{return m_acceleration[1];}

    void SetOrientation(const Vector3d& _newOrientation);
    void SetVelocity(const Vector2d& _newVelocity);
    void SetAcceleration(const Vector2d& _newAcceleration);

    void SetRotX(double _x){m_orientation[0] = _x;}
    void SetRotY(double _y){m_orientation[1] = _y;}
    void SetRotZ(double _z){m_orientation[2] = _z;}

    void SetVelX(double _x){m_velocity[0] = _x;}
    void SetVelY(double _y){m_velocity[1] = _y;}

    void SetAccelX(double _x){m_acceleration[0] = _x;}
    void SetAccelY(double _y){m_acceleration[1] = _y;}

    double GetSteeringAngle(){return m_steeringAngle;}
    void SetSteeringAngle(double _nsa);
    double GetWheelbase(){return m_wheelbase;}
    void SetWheelbase(double _nwbl){ m_wheelbase = fabs(_nwbl);}
    double GetMinTurningRadius(){return m_wheelbase / sin(m_maxSteeringAngle);}
    double GetTurningRadius(){return m_wheelbase / sin(m_steeringAngle);}
    bool IsReversing(){return m_reversing;}
    void SetReverse(bool _isReversing){m_reversing = _isReversing;}

    void SetMaxSteeringAngle(double _msa){ m_maxSteeringAngle = NormalizeTheta(m_maxSteeringAngle);}
    double GetMaxSteeringAngle(){return m_maxSteeringAngle;}

    void SetMaxSpeed(double _ms){ m_maxSpeed = fabs(_ms);}
    double GetMaxSpeed(){ return m_maxSpeed;}

    void SetMaxAccel(double _ma){ m_maxAccel = fabs(_ma);}
    double GetMaxAccel(){ return m_maxAccel;}

    ///determines equality of this and other configuration
    bool operator==(const SSSurface& _cfg) const;

    ///determines non-equality of this and other configuration
    bool operator!=(const SSSurface& _cfg) const;

    //addition
    SSSurface operator+(const SSSurface& _cfg) const;
    SSSurface& operator+=(const SSSurface& _cfg);

    //subtraction
    SSSurface operator-(const SSSurface& _cfg) const;
    SSSurface& operator-=(const SSSurface& _cfg);

    //negate
    SSSurface operator-() const;

    //scalar multiply
    SSSurface operator*(double _d) const;
    SSSurface& operator*=(double _d);

    //scalar divide
    SSSurface operator/(double _d) const;
    SSSurface& operator/=(double _d);

    //access dof values
    double& operator[](size_t _dof);
    const double& operator[](size_t _dof) const;

    //Update
    //takes in a delta, which is the size of a timestep.
    //Based on steering angle, velocity, acceleration, position, and orientation,
    //the system will increment itself to the next state, returning the next state.
    //The current state will remain intact
    SSSurface Update(double _dt);

    ////////////////////////////////////////////////////////////
    //this should have been implemented in the first plac3
    bool ConfigureRobot(Environment* _env) const;

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    private Data member and member methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
  private:
    //positional, height components from CfgSurface, as well as surfaceID
    Vector2d m_velocity;
    Vector2d m_acceleration;
    Vector3d m_orientation; // rotation in radians about x y and z

    //limites on the norms of velocity and acceleration vectors
    double m_maxAccel;
    double m_maxSpeed;

    //for a car
    double m_steeringAngle; //a value between -PI and Pi that defines the current steering angle
    double m_maxSteeringAngle; //limits on the steering angle, between 0 and PI
    double m_wheelbase; //distance between front and rear axles
    bool m_reversing;

    //helper functions
    void InitDefaults();
    void CapValues();
};

//I/O operators for SSSurface
ostream& operator<< (ostream& _os, const SSSurface& _cfg);
istream& operator>> (istream& _is, SSSurface& _cfg);

template<class DistanceMetricPointer>
void
SSSurface::GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm) {
  CfgSurface::GetRandomRay(_incr, _env, _dm, _norm);

  //randomly sample velocity, rotations, accelerations
}
#endif
