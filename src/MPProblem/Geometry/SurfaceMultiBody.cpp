#include "SurfaceMultiBody.h"

#include "FixedBody.h"

SurfaceMultiBody::
SurfaceMultiBody() : StaticMultiBody(MultiBodyType::Surface) {
}

void
SurfaceMultiBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  m_surfaceLabel = ReadFieldString(_is, _cbs, "Failed reading surface tag.");

  shared_ptr<FixedBody> fix(new FixedBody(this));
  fix->Read(_is, _cbs);

  AddBody(fix);

  FindMultiBodyInfo();
}

void
SurfaceMultiBody::
Write(ostream & _os) {
  _os << GetTagFromMultiBodyType(m_multiBodyType) << endl;
  _os << m_surfaceLabel << endl;
  for(auto& body : m_fixedBody)
    _os << *body << endl;
}
