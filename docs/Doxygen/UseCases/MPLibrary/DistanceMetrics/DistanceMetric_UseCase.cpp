void DistanceMetricUseCase() {
  // Get the distance metric by the XML node label
  std::string dmLabel = "Euclidean";
  DistanceMetricMethod* distanceMetric = this->GetDistanceMetric(dmLabel);

  // The two configurations to calculate the distance from.
  CfgType c1, c2;

  // The distance function takes as input two configurations, c1 and c2, and
  // returns the computed transition distance between them.
  double dist = distanceMetric->Distance(c1, c2);


  // Desired magnitude by which the directional configuration is scaled.
  double length = 0.3;

  // Configuration to be scaled.
  CfgType c3;

  // The ScaleCfg function rescales a configuration vector based on the
  // determined distance metric. It scales a directional configuration to a
  // certain magnitude using the default origin.
  distanceMetric->ScaleCfg(length, c3);
}
