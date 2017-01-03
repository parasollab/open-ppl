#include "FixedBody.h"


FixedBody::
FixedBody(MultiBody* _owner, const string& _filename) : Body(_owner) {
  m_filename = _filename;
}


void
FixedBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  m_filename = ReadFieldString(_is, _cbs,
      "Failed reading geometry filename.", false);

  ReadOptions(_is, _cbs);

  Read(m_comAdjust);

  m_transform = ReadField<Transformation>(_is, _cbs,
      "Failed reading fixed body transformation.");

  MarkDirty();
}


const Transformation&
FixedBody::
GetWorldTransformation() const {
  return m_transform;
}


void
FixedBody::
PutWorldTransformation(const Transformation& _worldTransformation) {
  m_transform = _worldTransformation;
  MarkDirty();
}


ostream&
operator<<(ostream& _os, const FixedBody& _fb){
  return _os << _fb.m_filename << " " << _fb.m_transform;
}
