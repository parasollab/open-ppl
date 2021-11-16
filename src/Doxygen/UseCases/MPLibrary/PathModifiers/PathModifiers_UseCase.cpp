void PathModifiersUseCase() {
  // The inputPath holds the initial path of the robot. It is typically
  // generated using a MapEvaluator. Each path is a vector of configurations
  // that the robot will follow from its start to its goal.
  Path* inputPath = this->GetPath(<< Specified Robot* >>);

  // The ouputPath will store the new path that is created by the path modifier.
  Path outputPath;

  // The path modifier label is a string that corresponds to the XML node label
  // for the path modifier algorithm you would like to use.
  std::string pmLabel = "ShortcuttingPathModifier";

  // The path modifier object corresponding to a specific path modifier
  // algorithm is used to run the path modifier algorithm.
  auto pm = this->GetPathModifier(pmLabel);

  // The Modify method of a path modifier object runs the path modifier
  // algorithm on the inputPath to create a new path that is stored in ouputPath.
  pm->Modify(*inputPath, outputPath);
}
