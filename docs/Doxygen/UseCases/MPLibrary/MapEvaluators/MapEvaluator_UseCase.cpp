void MapEvaluatorUseCase() {
  // Get a map evaluator by the XML node label.
  std::string evaluatorLabel = "QueryMethod";
  MapEvaluatorMethod* evaluator = this->GetMapEvaluator(evaluatorLabel);

  // The map evaluator's operator() evaluates the current free-space roadmap and
  // returns true if and only if it satisfies a specific set of criteria.
  // Roadmaps are generated using a sampler.
  bool passed = evaluator->operator();
}
