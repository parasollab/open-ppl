#include "FixedBody.h"


FixedBody::
FixedBody(MultiBody* _owner, const string& _filename) : Body(_owner) {
  m_filename = _filename;
}


FixedBody::
FixedBody(MultiBody* _owner, XMLNode& _node) : Body(_owner, _node) {
  ReadXML(_node);
}


void
FixedBody::
ReadXML(XMLNode& _node) {
  m_filename = _node.Read("filename", true, "", "The file containing the"
      " geometry information of this body.");

  // Call base class (Body.h) read. This is safe since the body was setup in the
  // constructor.
  Read(m_comAdjust);

  // Read transform values.
  const std::string transform = _node.Read("transform", false, "0 0 0 0 0 0",
      "The transformation values (x, y, z, roll, pitch, yaw).");

  // Use string stream to read values from string to a transform object.
  istringstream buffer(transform);
  buffer >> m_transform;

  MarkDirty();
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
