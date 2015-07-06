#include "NonHolonomicMultiBody.h"

#include "Control.h"

NonHolonomicMultiBody::
NonHolonomicMultiBody() : ActiveMultiBody(),
  m_maxLinearVel(numeric_limits<double>::max()),
  m_maxAngularVel(numeric_limits<double>::max()) {
    m_multiBodyType = MultiBodyType::NonHolonomic;
  }

vector<double>
NonHolonomicMultiBody::
GetRandomControl() const {
  size_t index = rand() % m_controls.size();
  return m_controls[index]->GetControl();
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

  cout << "Velocity bounds" << endl
    << m_maxLinearVel << " " << m_maxAngularVel << endl;

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

  m_controls.push_back(shared_ptr<Control>(new Control(this)));
  for(size_t i = 0; i < controlsCount; ++i) {
    shared_ptr<Control> c(new Control(this));
    m_controls.push_back(c);
    m_controls.back()->Read(_is, _cbs);
    cout << "Read control: " << *m_controls.back() << endl;
  }

  cout << "Moment: " << endl;
  cout << GetFreeBody(0)->GetMoment() << endl;
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

