void ConnectorsUseCase() {
  // Get the connector method by the XML node label
  std::string connectorLabel = "CCsConnector";
  ConnectorMethod* connectorMethod = this->GetConnector(connectorLabel);

  // Get the current free-space roadmap. You can choose to pass in a Robot* to
  // specify which robot's roadmap, but if it is omitted, it will return the
  // roadmap of the task's assigned robot.
  RoadmapType* roadmap = this->GetRoadmap();

  // You can specify which set of vertices you attempt to connect from. You can
  // choose either a single VID or a set of VIDs. In this case, we will be
  // choosing a set of VIDs.
  VertexSet sourceSet = roadmap->GetAllVIDs();

  // You can specify which set of vertices you attempt to connect to. If set to
  // a nullptr, the connector method will attempt to connect the full roadmap.
  VertexSet* targetSet = nullptr;

  // Connectors attempt to connect the vertices from a 'source' set in a roadmap
  // to each of the vertices in a 'target' set. Successful connections will give
  // a new edge in the roadmap. This particular connect call generates edges
  // using a range of VIDs as the source. When a target set of vertices is not
  // given, it will attempt to connect the full roadmap.
  connectorMethod->Connect(roadmap, sourceSet.begin(), sourceSet.end(), targetSet);
}
