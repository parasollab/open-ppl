#include "ValidityChecker.h"

ValidityChecker::
~ValidityChecker() {}

shared_ptr<ValidityCheckerMethod> ValidityChecker::GetMethod(string _label) {
  return ElementSet<ValidityCheckerMethod>::GetElement(_label);
}

void 
ValidityChecker::AddMethod(string const& _label, ValidityCheckerPointer _dmm){
  ElementSet<ValidityCheckerMethod>::AddElement(_label, _dmm);
}

void 
ValidityChecker::SetMPProblem(MPProblem* _mp){
  MPBaseObject::SetMPProblem(_mp);
  ElementSet<ValidityCheckerMethod>::SetMPProblem(_mp);
}

void ValidityChecker::PrintOptions(ostream& _os) const { 
  _os << "  Validity checker methods available:" << endl;
  for(map<string, shared_ptr<ValidityCheckerMethod> >::const_iterator M = ElementsBegin(); M != ElementsEnd(); ++M)
    _os <<"\t\"" << M->first << "\" (" << M->second->GetName() << ")" << endl;
}

vector<cd_predefined>
ValidityChecker::
GetSelectedCDTypes() const {
  vector<cd_predefined> cd_types;
  for(map<string, shared_ptr<ValidityCheckerMethod> >::const_iterator M = ElementsBegin(); M != ElementsEnd(); ++M) 
    if(CollisionDetectionValidity* method = dynamic_cast<CollisionDetectionValidity*>(M->second.get()))   
      cd_types.push_back(method->GetCDType());
  return cd_types;
}
