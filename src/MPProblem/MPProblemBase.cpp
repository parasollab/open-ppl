#include "MPProblemBase.h"

using namespace std;

string MPProblemBase::m_filePath = "";

bool
MPProblemBase::
IsAbsolutePathBaseFile(const string& _baseFilename) {
  return _baseFilename[0] == '/';
}

string
MPProblemBase::
GetPath(const string& _filename) {
  if(!IsAbsolutePathBaseFile(_filename))
    return m_filePath + _filename;
  else
    return _filename;
}

