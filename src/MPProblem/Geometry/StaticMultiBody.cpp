#include "StaticMultiBody.h"

#include "FixedBody.h"

StaticMultiBody::
StaticMultiBody() : MultiBody() {
}

void
StaticMultiBody::
Initialize(const string& _modelFileName, const Transformation& _where) {
  shared_ptr<FixedBody> fix(new FixedBody(this, _modelFileName));
  fix->Read();

  Transformation worldTransform(_where);
  fix->PutWorldTransformation(worldTransform);

  AddBody(fix);

  FindBoundingBox();
  ComputeCenterOfMass();
}

shared_ptr<FixedBody>
StaticMultiBody::
GetFixedBody(size_t _index) const {
  if(_index < m_fixedBody.size())
    return m_fixedBody[_index];
  else
    return shared_ptr<FixedBody>();
}

void
StaticMultiBody::
AddBody(const shared_ptr<FixedBody>& _body) {
  m_fixedBody.push_back(_body);
  MultiBody::AddBody(_body);
}

void
StaticMultiBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {

  //all are same type, namely fixed body
  shared_ptr<FixedBody> fix(new FixedBody(this));
  fix->Read(_is, _cbs);

  //add fixed body to multibody
  AddBody(fix);

  FindBoundingBox();
  ComputeCenterOfMass();
}

void
StaticMultiBody::
Write(ostream & _os) {
  _os << GetTagFromBodyType(GetBodyType()) << endl;
  for(auto& body : m_fixedBody)
    _os << *body << endl;
}
