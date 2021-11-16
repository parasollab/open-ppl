void ValidityCheckersUseCase() {

  // The validity checker (VC) label is a string that corresponds to the XML node
  // label for the validity checker you would like to use.
  std::string vc_label = "CollisionDetectionValidity"

  // Given a VC label the MPLibrary function GetValidityChecker() returns a
  // ValidityCheckerPointer object that points to an appropriate VC.
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);

  // The C-space configuration cfg will be tested for validity.
  CfgType cfg = CfgType(#NEED A ROBOT HERE);

  // The collision detection info object will be used by the VC to output
  // additional information such as the clearance (distance from obstacles) of
  // the configuration.
  CDInfo cdInfo;

  // The caller function name is given to the VC for debugging purposes.
  std::string caller = "ValidityCheckersUseCaseFunction"

  // The IsValid function will classify the C-space configuration cfg as
  // belonging to free-space or obstacle space by returning true or false
  // respectively. Additional information will be stored in the CDInfo 
  // object cdInfo, and if it is run in debugging mode or an exception is
  // thrown, the caller string will be used to indicate the calling function.
  bool valid = vc->IsValid(cfg, cdInfo, caller);
}






