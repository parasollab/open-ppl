#include "FixedBody.h"

FixedBody::
FixedBody(MultiBody* _owner, const string& _filename) :
  Body(_owner) {
    m_filename = _filename;
  }

FixedBody::
FixedBody(MultiBody* _owner, GMSPolyhedron& _polyhedron) :
  Body(_owner, _polyhedron) {
  }

void
FixedBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  m_filename = ReadFieldString(_is, _cbs,
      "Failed reading geometry filename.", false);
  Read();

  m_worldTransformation = ReadField<Transformation>(_is, _cbs,
      "Failed reading fixed body transformation.");
}

Transformation&
FixedBody::GetWorldTransformation() {
  return m_worldTransformation;
}

ostream&
operator<<(ostream& _os, const FixedBody& _fb){
  return _os << _fb.m_filename << " " << _fb.m_worldTransformation;
}

