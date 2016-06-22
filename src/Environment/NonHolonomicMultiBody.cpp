#include "NonHolonomicMultiBody.h"

#include "Control.h"

NonHolonomicMultiBody::
NonHolonomicMultiBody() : ActiveMultiBody(),
  m_maxLinearVel(numeric_limits<double>::max()),
  m_maxAngularVel(numeric_limits<double>::max()) {
    m_multiBodyType = MultiBodyType::NonHolonomic;
  }

const vector<double>&
NonHolonomicMultiBody::
GetRandomControl() const {
  size_t index = rand() % m_controls.size();
  return m_controls[index]->GetControl();
}

const vector<shared_ptr<Control>>&
NonHolonomicMultiBody::
AvailableControls() const {
  return m_controls;
}

vector<double>
NonHolonomicMultiBody::
GetRandomVelocity() const {
  Vector3d l(DRand);
  l = l.normalize() * DRand() * m_maxLinearVel;
  Vector3d a(DRand);
  a = a.normalize() * DRand() * m_maxAngularVel;
  vector<double> ret;
  ret.reserve(6);
  for(size_t i = 0; i < 3; ++i)
    ret.emplace_back(l[i]);
  for(size_t i = 0; i < 3; ++i)
    ret.emplace_back(a[i]);
  return ret;
}

bool
NonHolonomicMultiBody::
InSSpace(const vector<double>& _pos, const vector<double>& _vel,
    shared_ptr<Boundary>& _b) {
  if(InCSpace(_pos, _b)) {
    Vector3d linear(_vel[0], _vel[1], _vel[2]);
    Vector3d angular(_vel[3], _vel[4], _vel[5]);
    if(linear.norm() < m_maxLinearVel && angular.norm() < m_maxAngularVel)
      return true;
  }
  return false;
}

void
NonHolonomicMultiBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {

  ActiveMultiBody::Read(_is, _cbs);

  //first read velocity bounds
  string velocityBoundsTag = ReadFieldString(_is, _cbs,
      "Failed reading velocity bounds tag.");
  if(velocityBoundsTag != "VELOCITYBOUNDS")
    throw ParseException(_cbs.Where(),
        "Unknwon controls tag '" + velocityBoundsTag + "'."
        " Should read 'VelocityBounds'.");

  m_maxLinearVel = ReadField<double>(_is, _cbs,
      "Failed reading maximum linear velocity");
  m_maxAngularVel = ReadField<double>(_is, _cbs,
      "Failed reading maximum angular velocity");

  //read available controls
  string controlsTag = ReadFieldString(_is, _cbs,
      "Failed reading controls tag.");
  if(controlsTag != "CONTROLS")
    throw ParseException(_cbs.Where(),
        "Unknwon controls tag '" + controlsTag + "'."
        " Should read 'Controls'.");
  size_t controlsCount = ReadField<size_t>(_is, _cbs,
      "Failed reading number of controls.");

  //finish empty line of count
  string tmp;
  getline(_is, tmp);

  for(size_t i = 0; i < controlsCount; ++i) {
    shared_ptr<Control> c(new Control(this));
    m_controls.push_back(c);
    m_controls.back()->Read(_is, _cbs);
  }
}

void
NonHolonomicMultiBody::
Write(ostream & _os) {
  ActiveMultiBody::Write(_os);
  _os << "VelocityBounds" << endl
    << m_maxLinearVel << " " << m_maxAngularVel << endl;
  _os << "Controls" << endl;
  for(auto& control : m_controls)
    _os << *control << endl;
}

