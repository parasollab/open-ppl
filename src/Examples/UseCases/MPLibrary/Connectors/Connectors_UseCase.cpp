void ConnectorsUseCase() {
    // Connectors attempt to connect the vertices from a 'source' set
    // in a roadmap to each of the vertices in a 'target' set. 
    // Successful connections will give a new edge in the roadmap.
    ConnectorMethod* connectorMethod = this->GetConnector(connectorLabel);

    // Get roadmap for the assigned task's robot
    // (Alternatively specify which robot to retrieve roadmap for)
    RoadmapType* roadmap = this->getRoadmap();

    // Specify iterators for start and end points of source vertices
    // Can choose to use whole roadmap or only single vertex as source
    it_rStart = roadmap->begin();
    it_rEnd = roadmap->end();

    // Generate edge connections using range of VIDs as sources
    // Targets are not specified here and thus the whole roadmap is used
    // But they can be specified as a vertex set
    // An outputiterator of RoadmapTpe* can be specified to store collisions
    connectorMethod->Connect(roadmap, it_rStart, it_rEnd);   
}