#include "FixedBody.h"

FixedBody::
FixedBody(MultiBody* _owner, const string& _filename) :
  Body(_owner) {
    m_filename = _filename;
  }

void
FixedBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  m_filename = ReadFieldString(_is, _cbs,
      "Failed reading geometry filename.", false);

  ReadOptions(_is, _cbs);

  Read(m_comAdjust);

  m_worldTransformation = ReadField<Transformation>(_is, _cbs,
      "Failed reading fixed body transformation.");
  m_worldPolyhedronAvailable = false;
}

Transformation&
FixedBody::GetWorldTransformation() {
  return m_worldTransformation;
}

void
FixedBody::
PutWorldTransformation(Transformation& _worldTransformation) {
  m_worldTransformation = _worldTransformation;
  m_worldPolyhedronAvailable = false;
}


ostream&
operator<<(ostream& _os, const FixedBody& _fb){
  return _os << _fb.m_filename << " " << _fb.m_worldTransformation;
}
