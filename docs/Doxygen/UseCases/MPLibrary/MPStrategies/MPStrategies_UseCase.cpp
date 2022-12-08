void MPStrategiesUseCase() {
  // Get the motion planning strategy by the XML node label.
  std::string m_mpsLabel = "BasicPRM";
  MPStrategyMethod* mpStrategy = this->GetMPStrategy(m_mpsLabel);

  // Calling operator on the MPStrategy object with pointer notation. This will
  // execute all the associated functionalities of the MP strategy algorithm
  // including preprocessing (Initialize), processing (Run), and postprocessing
  // (Finalize).
  mpStrategy->operator()();
}
