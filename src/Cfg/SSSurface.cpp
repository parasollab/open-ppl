
////////////////////////////////////////////////////////////////////////////////////////////
#include "SSSurface.h"
#include "CfgSurface.h"

#include "Environment/ActiveMultiBody.h"
#include "Environment/Environment.h"
#include "Environment/FreeBody.h"

void
SSSurface::InitDefaults(){
  m_maxAccel = 1.0;
  m_maxSpeed = 5.0;
  m_maxSteeringAngle = 0.22;//PI/2.0 was not working and previously default was this
  m_wheelbase = 1.0;
  m_steeringAngle = 0.0;
  m_velocity[0] = 0.0; m_velocity[1] = 1.0;
  m_acceleration[0] = m_acceleration[1] = 0.0;
  m_orientation[0] = m_orientation[1] = m_orientation[2] = 0.0;
  m_reversing = false;
}

SSSurface::SSSurface() : CfgSurface(){
  InitDefaults();
}

SSSurface::SSSurface(const SSSurface& _rhs){
  *this = _rhs;
}

SSSurface::SSSurface(const CfgSurface& _rhs) : CfgSurface(_rhs){
  InitDefaults();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//    Access Methods : Retrive and set related information of this class
//
//
//////////////////////////////////////////////////////////////////////////////////////////
void SSSurface::SetOrientation(const Vector3d& _newOrientation){
  SetRotX(_newOrientation[0]);
  SetRotY(_newOrientation[1]);
  SetRotZ(_newOrientation[2]);
}

void SSSurface::SetVelocity(const Vector2d& _newVelocity){
  SetVelX(_newVelocity[0]);
  SetVelY(_newVelocity[1]);
}

void SSSurface::SetAcceleration(const Vector2d& _newAcceleration){
  SetAccelX(_newAcceleration[0]);
  SetAccelY(_newAcceleration[1]);
}

void SSSurface::Read(istream& _is){
  CfgSurface::Read(_is);
  //read in orientation, velocity, acceleration, steering angle, max speed, max acceleration (in that order)
  _is >> m_orientation
    >> m_velocity
    >> m_acceleration
    >> m_steeringAngle
    >> m_maxSteeringAngle
    >> m_maxSpeed
    >> m_maxAccel
    >> m_wheelbase;

  //cap velocity, acceleration, normalize orientation and steering angle
  CapValues();

}

void SSSurface::Write(ostream& _os) const{
  CfgSurface::Write(_os);

  //write out orientation, velocity, acceleration, steering angle, max speed, max acceleration (in that order)
  _os << setw(4) << m_orientation[0] << " "
    << setw(4) << m_orientation[1] << " "
    << setw(4) << m_orientation[2] << " "
    << setw(4) << m_velocity[0] << " "
    << setw(4) << m_velocity[1] << " "
    << setw(4) << m_acceleration[0] << " "
    << setw(4) << m_acceleration[1] << " "
    << setw(4) << m_steeringAngle << " "
    << setw(4) << m_maxSteeringAngle << " "
    << setw(4) << m_maxSpeed << " "
    << setw(4) << m_maxAccel << " "
    << setw(4) << m_wheelbase << " ";
  if (_os.fail()){
    cerr << "SSSurface::Write error - failed to write to file" << endl;
    exit(1);
  }
}

ostream&
operator<< (ostream& _os, const SSSurface& _cfg){
  _cfg.Write(_os);
  return _os;
}

istream&
operator>> (istream& _is, SSSurface& _cfg){
  _cfg.Read(_is);
  return _is;
}

void SSSurface::GetRandomCfgImpl(Environment* _env,shared_ptr<Boundary> _bb){
  cerr << "SSSurface::GetRandomCfgImpl not yet implemented" << endl;
  exit(1);
}


SSSurface& SSSurface::operator=(const SSSurface& _c){

  //call base class' operator
  CfgSurface* temp = this;
  const CfgSurface* temp2 = &_c;
  *temp = *temp2;

  m_velocity = _c.m_velocity;
  m_acceleration = _c.m_acceleration;
  m_orientation = _c.m_orientation;
  m_steeringAngle = _c.m_steeringAngle;
  m_maxSteeringAngle = _c.m_maxSteeringAngle;
  m_wheelbase = _c.m_wheelbase;
  m_reversing = _c.m_reversing;
  m_maxSpeed = _c.m_maxSpeed;
  m_maxAccel = _c.m_maxAccel;

  return *this;
}

void SSSurface::IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment){
  CfgSurface::IncrementTowardsGoal(_goal, _increment);
  //TODO: the base class iterates over m_dof...which may not be right for us in this case, we want to also iterate over
  //acceleration etc, unless we set m_dof correctly at the start
}

void SSSurface::FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks,
    double _positionRes, double _orientationRes){
  CfgSurface::FindIncrement(_start, _goal, _nTicks, _positionRes, _orientationRes);
}

void SSSurface::FindIncrement(const Cfg& _start, const Cfg& _goal, int _nTicks){
  CfgSurface::FindIncrement(_start, _goal, _nTicks);
}

const string SSSurface::GetName() const{
  return "SSSurface";
}

void SSSurface::SetSteeringAngle(double _nsa){
  _nsa = NormalizeTheta(_nsa);
  m_steeringAngle = _nsa;
  if (fabs(m_steeringAngle) > m_maxSteeringAngle){
    m_steeringAngle = m_maxSteeringAngle;
    if (_nsa < 0)
      m_steeringAngle *= -1.0;
  }
}

double& SSSurface::operator[](size_t _dof){
  if (_dof < 3){
    return CfgSurface::operator[](_dof);
  }
  else switch(_dof){
    case 3: return m_orientation[0];
    case 4: return m_orientation[1];
    case 5: return m_orientation[2];
    case 6: return m_velocity[0];
    case 7: return m_velocity[1];
    case 8: return m_acceleration[0];
    case 9: return m_acceleration[1];
    case 10: return m_steeringAngle;
    default:
       cerr << "SSSurface Invalid access to index " << _dof << ". Exiting." << endl;
       exit(1);
  }
}

const double& SSSurface::operator[](size_t _dof) const{
  if (_dof < 3){
    return CfgSurface::operator[](_dof);
  }
  else switch(_dof){
    case 3: return m_orientation[0];
    case 4: return m_orientation[1];
    case 5: return m_orientation[2];
    case 6: return m_velocity[0];
    case 7: return m_velocity[1];
    case 8: return m_acceleration[0];
    case 9: return m_acceleration[1];
    case 10: return m_steeringAngle;
    default:
       cerr << "Cfg Surface Invalid access to index " << _dof << ". Exiting." << endl;
       exit(1);
  }
}
bool
SSSurface::operator==(const SSSurface& _cfg) const {
  return ((m_pt == _cfg.m_pt) && (fabs(m_h-_cfg.m_h) < numeric_limits<double>::epsilon()) &&
      (m_velocity == _cfg.m_velocity) && (m_acceleration == _cfg.m_acceleration) &&
      (m_orientation == _cfg.m_orientation) &&
      (fabs(m_steeringAngle-_cfg.m_steeringAngle) < numeric_limits<double>::epsilon()) &&
      (m_robotIndex == _cfg.m_robotIndex) &&
      (fabs(m_wheelbase-_cfg.m_wheelbase) < numeric_limits<double>::epsilon()) &&
      (m_reversing == _cfg.m_reversing));
}

bool
SSSurface::operator!=(const SSSurface& _cfg) const {
  return !(*this == _cfg);
}

SSSurface
SSSurface::operator+(const SSSurface& _cfg) const {
  SSSurface result = *this;
  result += _cfg;
  return result;
}

SSSurface&
SSSurface::operator+=(const SSSurface& _cfg) {

  CfgSurface* temp = this;
  const CfgSurface* temp2 = &_cfg;
  *temp += *temp2;

  m_velocity += _cfg.m_velocity;
  m_acceleration += _cfg.m_acceleration;
  m_orientation += _cfg.m_orientation;
  m_steeringAngle += _cfg.m_steeringAngle;

  //cap velocity, acceleration, normalize orientation and steering angle
  CapValues();
  return *this;
}

SSSurface
SSSurface::operator-(const SSSurface& _cfg) const {
  SSSurface result = *this;
  result -= _cfg;
  return result;
}

SSSurface&
SSSurface::operator-=(const SSSurface& _cfg) {
  CfgSurface* temp = this;
  const CfgSurface* temp2 = &_cfg;
  *temp -= *temp2;

  m_velocity -= _cfg.m_velocity;
  m_acceleration -= _cfg.m_acceleration;
  m_orientation -= _cfg.m_orientation;
  m_steeringAngle -= _cfg.m_steeringAngle;

  //cap velocity, acceleration, normalize orientation and steering angle
  CapValues();
  return *this;
}

SSSurface
SSSurface::operator-() const {
  CfgSurface temp = *this;
  SSSurface result(-temp);

  result.m_velocity = -m_velocity;
  result.m_acceleration = -m_acceleration;
  result.m_orientation = -m_orientation;
  result.m_steeringAngle = -m_steeringAngle;
  return result;
}

SSSurface
SSSurface::operator*(double _d) const {
  SSSurface result = *this;
  result *= _d;
  return result;
}

SSSurface&
SSSurface::operator*=(double _d) {
  CfgSurface* temp = this;
  *temp *= _d;

  m_velocity *= _d;
  m_acceleration *= _d;
  m_orientation *= _d;
  m_steeringAngle *= _d;

  //cap velocity, acceleration, normalize orientation and steering angle
  CapValues();
  return *this;
}

SSSurface
SSSurface::operator/(double _d) const {
  SSSurface result = *this;
  result /= _d;
  return result;
}

SSSurface&
SSSurface::operator/=(double _d) {
  CfgSurface* temp = this;
  *temp /= _d;

  m_velocity /= _d;
  m_acceleration /= _d;
  m_orientation /= _d;
  m_steeringAngle /= _d;

  //cap velocity, acceleration, normalize orientation and steering angle
  CapValues();
  return *this;
}

void SSSurface::CapValues(){
  if (m_velocity.norm() > m_maxSpeed){
    m_velocity = m_velocity.normalize() * m_maxSpeed;
  }

  if (m_acceleration.norm() > m_maxAccel){
    m_acceleration = m_acceleration.normalize() * m_maxAccel;
  }

  m_steeringAngle =  NormalizeTheta(m_steeringAngle);
  m_orientation[0] = NormalizeTheta(m_orientation[0]);
  m_orientation[1] = NormalizeTheta(m_orientation[1]);
  m_orientation[2] = NormalizeTheta(m_orientation[2]);
}

SSSurface SSSurface::Update(double _dt){
  //for now, we're going to model the dynamics of the Reed-Shepp car.
  //That is, unit velocity and no acceleration.
  //a true car that uses acceleration must also monitor tire friction with the ground, and since we don't do that,
  //acceleration is useless to us at this point.
  //Velocity will be rounded to the nearest integer, with a minimum of fabs(velocity) = 1.
  //Because Reeds-Shepp assumes unit velocity, a velocity higher than 1 implies that we need to integrate the state more
  //than once in this timestep


  SSSurface toReturn = *this;
  //update position based on current inputs
  //then update orientation

  int velocity = m_velocity.norm();
  double theta = m_orientation[1]; //rotation about vertical axis
  velocity = m_maxSpeed;
  bool straightLine = true;
  double turningRadius = 0.0;

  if (fabs(m_steeringAngle) > 1e-5){
    turningRadius = GetTurningRadius();
    straightLine = false;
  }
  if (m_reversing)
    velocity *= -1;

  double vel=1.0;
  if (velocity < 0)
    vel = -1.0;

  //assume cfgs loaded in to be facing down 0,-1 instead of 0, 0
  theta = NormalizeTheta(theta+PI/2.0);
  //theta = NormalizeTheta(theta+3*PI/2.0);
  Point2d pos = m_pt;
  for (int i=0; i<abs(velocity); ++i){
  //for (int i=0; i<1; ++i){ //<--this step was only done once before!!!!!F
    //pos.set(pos[0] + vel*_dt*cos(theta), pos[1] + vel*_dt*sin(theta)); -- way it should be

    //way it actually is with the weird coordinates
    pos(pos[0] - vel*_dt*cos(theta), pos[1] + vel*_dt*sin(theta));

    if (!straightLine){
      theta = theta + (vel*_dt)/turningRadius;
    }
  }//end Euler integration

  //undo rotation from before
  toReturn.SetRotY(NormalizeTheta(theta-PI/2.0));
  //toReturn.SetRotY(NormalizeTheta(theta-3*PI/2.0));
  toReturn.SetPos(pos);

  return toReturn;
}

/////////////////////////////////////////////////////////////
bool SSSurface::ConfigureRobot(Environment* _env) const {
  shared_ptr<ActiveMultiBody> mb = _env->GetRobot(m_robotIndex);

  // configure the robot according to current Cfg: joint parameters
  // (and base locations/orientations for free flying robots.)
  Transformation T1 = Transformation(
      Vector3d(m_pt[0], m_h, m_pt[1]),
      Orientation(EulerAngle(0, m_orientation[1], 0)));
  // update link i
  mb->GetFreeBody(0)->Configure(T1);

  return true;
}
