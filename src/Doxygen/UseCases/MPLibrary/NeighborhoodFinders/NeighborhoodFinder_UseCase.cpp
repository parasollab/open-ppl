void NeighborhoodFinderUseCase() {
  // Get the neighborhood finder by the XML node label.
  std::string nfLabel = "BruteForceNF";
  NeighborhoodFinder* nf = this->GetNeighborhoodFinder(nfLabel);

  // Get the current free-space roadmap. You can choose to pass in a Robot* to
  // specify which robot's roadmap, but if it is omitted, it will return the
  // roadmap of the task's assigned robot.
  RoadmapType* roadmap = this->GetRoadmap();

  // Get a specific cfg from the roadmap. We will be finding neighbors for this
  // configuration.
  CfgType cfg = roadmap->GetVertex(<VID>);

  // Specify the set of candidate vertices from which we will be finding the
  // neighbors.
  VertexSet candidates = <unordered_set<VID>>;

  // Create the output object in which we will be storing the neighbors we find.
  std::vector<Neighbor> neighbors;

  // The FindNeighbors function takes an input configuration (cfg) and
  // optionally a set of candidate neighbors (candidates). It returns the
  // computed set of neighbors from a specified roadmap and their distance from
  // the input configuration through the 'Neighbor' structure.
  nf->FindNeighbors(roadmap, cfg, candidates, std::back_inserter(neighbors));
}
