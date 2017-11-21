#include "StaticMultiBody.h"
#include "FixedBody.h"
#include "Utilities/XMLNode.h"

StaticMultiBody::
StaticMultiBody(MultiBodyType _m) : MultiBody() {
  m_multiBodyType = _m;
}

void
StaticMultiBody::
Initialize(const string& _modelFileName, const Transformation& _where) {
  shared_ptr<FixedBody> fix(new FixedBody(this, _modelFileName));
  fix->Read(GMSPolyhedron::COMAdjust::COM);

  Transformation worldTransform(_where);
  fix->PutWorldTransformation(worldTransform);

  AddBody(fix);

  FindMultiBodyInfo();
}

bool
StaticMultiBody::
IsInternal() const {
  return m_multiBodyType == MultiBodyType::Internal;
}

const FixedBody*
StaticMultiBody::
GetFixedBody(size_t _index) const {
  if(_index >= m_fixedBody.size())
    throw RunTimeException(WHERE,
        "Cannot access FixedBody(" + ::to_string(_index) + ").");
  return m_fixedBody[_index].get();
}

void
StaticMultiBody::
AddBody(const shared_ptr<FixedBody>& _body) {
  m_fixedBody.push_back(_body);
  MultiBody::AddBody(_body);
}


void
StaticMultiBody::
ReadXML(XMLNode& _node) {
  shared_ptr<FixedBody> fix(new FixedBody(this, _node));

  AddBody(fix);

  FindMultiBodyInfo();
}


void
StaticMultiBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  shared_ptr<FixedBody> fix(new FixedBody(this));
  fix->Read(_is, _cbs);

  AddBody(fix);

  FindMultiBodyInfo();
}

void
StaticMultiBody::
Write(ostream & _os) {
  _os << GetTagFromMultiBodyType(m_multiBodyType) << endl;
  for(auto& body : m_fixedBody)
    _os << *body << endl;
}
