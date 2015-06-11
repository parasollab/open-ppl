#include "SurfaceMultiBody.h"

#include "FixedBody.h"

SurfaceMultiBody::
SurfaceMultiBody() : StaticMultiBody() {
}

void
SurfaceMultiBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
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
SurfaceMultiBody::
Write(ostream & _os) {
  _os << GetTagFromBodyType(GetBodyType()) << endl;
  _os << m_surfaceLabel << endl;
  for(auto& body : fixedBody)
    _os << *body << endl;
}
