#include "StaticMultiBody.h"

#include "FixedBody.h"

StaticMultiBody::
StaticMultiBody() : MultiBody() {
}

shared_ptr<FixedBody>
StaticMultiBody::
GetFixedBody(size_t _index) const {
  if(_index < fixedBody.size())
    return fixedBody[_index];
  else
    return shared_ptr<FixedBody>();
}

void
StaticMultiBody::
AddBody(const shared_ptr<FixedBody>& _body) {
  fixedBody.push_back(_body);
  MultiBody::AddBody(_body);
}

void
StaticMultiBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  if(IsSurface())
    m_surfaceLabel = ReadFieldString(_is, _cbs, "Failed reading surface tag.");

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
  if(IsSurface())
    _os << m_surfaceLabel << endl;
  for(auto& body : fixedBody)
    _os << *body << endl;
}
