#include "Connection.h"

#include "ActiveMultiBody.h"
#include "FreeBody.h"

size_t Connection::m_globalCounter = 0;

Connection::
Connection(MultiBody* _owner) : m_multibody(_owner) {
  m_globalIndex = m_globalCounter++;
}

void
Connection::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  //body indices
  m_bodyIndices.first = ReadField<int>(_is, _cbs,
      "Failed reading previous body index.");
  m_bodyIndices.second = ReadField<int>(_is, _cbs,
      "Failed reading next body index.");

  //grab the shared_ptr to bodies
  m_bodies[0] = ((ActiveMultiBody*)m_multibody)->GetFreeBody(m_bodyIndices.first);
  m_bodies[1] = ((ActiveMultiBody*)m_multibody)->GetFreeBody(m_bodyIndices.second);

  //transformation to DHFrame
  m_transformationToDHFrame = ReadField<Transformation>(_is, _cbs,
      "Failed reading transformation to DH frame.");

  //DH parameters
  m_dhParameters = ReadField<DHParameters>(_is, _cbs,
      "Failed reading DH parameters.");

  //transformation to next body
  m_transformationToBody2 = ReadField<Transformation>(_is, _cbs,
      "Failed reading transformation to next body.");

  //make the connection
  m_bodies[0]->Link(*this);
}

ostream&
operator<<(ostream& _os, const Connection& _c) {
  return _os << _c.m_bodyIndices.first << " " << _c.m_bodyIndices.second << " "
    << _c.m_transformationToDHFrame << " " << _c.m_dhParameters << " "
    << _c.m_transformationToBody2;
}
