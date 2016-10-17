#include "MPProblemBase.h"

using namespace std;

string MPProblemBase::m_filePath = "";

string
MPProblemBase::
GetPath(const string& _filename) {
  if(_filename[0] != '/')
    return m_filePath + _filename;
  else
    return _filename;
}

void
MPProblemBase::
SetPath(const string& _filename) {
  m_filePath = _filename;
}
