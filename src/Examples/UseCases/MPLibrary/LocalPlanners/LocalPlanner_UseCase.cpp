void LocalPlannerUseCase() {
  // lp is a pointer to a LocalPlanner instance generated with the 
  // given m_lpLabel. This is the planner that will attempt to find paths
  // from one provided configuration to another within a given environment.
  LocalPlannerPointer lp = this->GetLocalPlanner(m_lpLabel);

  // The environment defines properties of the planning problem workspace.
  // This is nessecary within the context of the LocalPlanner as to
  // provide the positional and orientational resolutions on a
  // configuration's degrees of freedom.
  Environment* env = this->GetEnvironment();

  // c1 and c2 are the configurations that will be queried for a
  // connecting path in both main functions. col is the witness 
  // configuration if a path cannot be determined by the function
  // IsConnected (failure).
  CfgType c1, c2, col;

  // lpOut is an abstraction that will be populated with various 
  // information regarding the attempted path upon being called 
  // by IsConnected. Functions as a secondary output.
  LPOutput<MPTraits> lpOut;

  // IsConnected is the main function of LocalPlanner, and takes in all
  // of the above information in attempting to solve for a path
  // between the two provided configurations. Returns true if a path is
  // found, false otherwise; additional information is stored within the
  // provided LPOutput. 
  // IsConnected may take two optional booleans: _checkCollision
  // (default true) determines whether validity checking is used, and
  // _savePath (default false) determines whether intermediate 
  // configurations calculated along the path are stored or discarded.
  lp->IsConnected(c1, c2, col, &lpOut, env->GetPositionRes(), 
	   env->GetOrientationRes());

  // BlindPath is a secondary main function of LocalPlanner, taking in
  // the above information to build and return a path from one configuration
  // to another. It does not use collision checking, and as such is best
  // utilized for recomputing and working with pre-validated paths 
  // and path waypoints. Returns a vector of intermediate 
  // configurations connecting the given endpoints (excluding endpoints).
  lp->BlindPath({c1, c2}, env->GetPositionRes(), env->GetOrientationRes());
}
