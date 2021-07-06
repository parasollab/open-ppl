#ifndef PPL_CONNECTOR_METHOD_TEST_H_
#define PPL_CONNECTOR_METHOD_TEST_H_

#include "MPLibrary/Connectors/RewireConnector.h"
#include "Testing/MPLibrary/Connectors/ConnectorMethodTest.h"

// @TODO: Default Function Calls? See ConnectorMethodTest.h, line ~45

template <class MPTraits>
class RewireConnectorTest : virtual public RewireConnector<MPTraits>,
                                 public ConnectorMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;
    typedef typename MPTraits::RoadmapType RoadmapType;

    ///@}
    ///@name Construction
    ///@{

    RewireConnectorTest();

    RewireConnectorTest(XMLNode& _node);

    ~RewireConnectorTest();

    ///@}

  private: 

    ///@name Interface Test Functions
    ///@{

    virtual TestResult IndividualRobotConnectTest() override;

    virtual TestResult RobotGroupConnectTest() override;

    ///@}


    ///@name Test Helper Functions
    ///@{
    bool
    IndividualRobotConnectRunTest(
            std::vector<std::vector<double>> vertex_poses,
            std::vector<std::pair<size_t, size_t>> start_edges,
            std::vector<std::pair<size_t, size_t>> end_edges);

    double
    ComputeDist(std::vector<double> a, std::vector<double> b);
    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
RewireConnectorTest<MPTraits>::
RewireConnectorTest() : RewireConnector<MPTraits>() {}

template <typename MPTraits>
RewireConnectorTest<MPTraits>::
RewireConnectorTest(XMLNode& _node) : ConnectorMethod<MPTraits>(_node),
                                           RewireConnector<MPTraits>(_node) {}

template <typename MPTraits>
RewireConnectorTest<MPTraits>::
~RewireConnectorTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename RewireConnectorTest<MPTraits>::TestResult
RewireConnectorTest<MPTraits>::
IndividualRobotConnectTest() {

  bool passed = true;
  std::string message = "";

  // Create a test Configuration and ensure robot has enough DOFs for the test.
  typename MPTraits::CfgType testCfg = this->GetIndividualCfg();
  if (testCfg.PosDOF() < 2) {
    passed = false;
    message = "\tGiven robot does not have enough PosDOFs to run the test.\n";
  }


  // Begin running Tests.
  //
  // IMPORTANT NOTE: The last vertex given in vertex_poses is assumed to be
  //  the vertex which connect is called on! I.e. the most recently added
  //  vertex to the tree.

  // Test1: Rewire triangle.
  //  Before:      After:
  //     1             1
  //    /|            /
  //   / |           /
  //  R  2          R--2

  // Vertices
  std::vector<double> p_root = {0.0, 0.0};
  std::vector<double> p_1    = {1.0, 1.0};
  std::vector<double> p_2    = {1.0, 0.0};

  std::vector<std::vector<double>> verts;
  verts.push_back(p_root);
  verts.push_back(p_1);
  verts.push_back(p_2);

  // Start edges.
  std::vector<std::pair<size_t, size_t>> start_edges;
  start_edges.push_back(std::pair<size_t, size_t>(0, 1));
  start_edges.push_back(std::pair<size_t, size_t>(1, 2));

  // Expected end edges.
  std::vector<std::pair<size_t, size_t>> end_edges;
  end_edges.push_back(std::pair<size_t, size_t>(0, 1));
  end_edges.push_back(std::pair<size_t, size_t>(0, 2));

  // Call the helper function.
  if (! IndividualRobotConnectRunTest(verts, start_edges, end_edges)) {
    // Test failed!
    passed = false;
    message += "\tFailed Test1: Rewire triangle.\n";
  }


  // @TODO: Finish writing individual Test.
  passed = false;
  message = "\n\tIndividualRobotConnectTest not implemented yet!\n";

  if(passed) {
    message = "IndividualRobotConnectTest::PASSED!\n";
  }
  else {
    message = "IndividualRobotConnectTest::FAILED :(\n" + message + "\n";
  }

  return std::make_pair(passed,message);
}

template <typename MPTraits>
typename RewireConnectorTest<MPTraits>::TestResult
RewireConnectorTest<MPTraits>::
RobotGroupConnectTest() {

  bool passed = true;
  std::string message = "";

  // @TODO: write Group Test.
  passed = false;
  message = "\n\tRobotGroupConnectTest not implemented yet!\n";

  if(passed) {
    message = "RobotGroupConnectTest::PASSED!\n";
  }
  else {
    message = "RobotGroupConnectTest::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

/*----------------------- Test Helper Functions ----------------------*/

template <typename MPTraits>
bool
RewireConnectorTest<MPTraits>::
IndividualRobotConnectRunTest(
        std::vector<std::vector<double>> vertex_poses,
        std::vector<std::pair<size_t, size_t>> start_edges,
        std::vector<std::pair<size_t, size_t>> end_edges) {

  // Create the roadmap.
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  RoadmapType roadmap(robot);


  // Add Cfgs to roadmap. (Collect Vertex IDs to refer to them later)

  // Vector to create cfgs.
  std::vector<typename MPTraits::CfgType> cfgs;
  // Vector to capture Vertex IDs for use later.
  std::vector<size_t> v_ids;

  std::cout << "Part1" << std::endl;

  for (size_t i = 0; i < vertex_poses.size(); i++) {
    // Create a new Cfg and ensure that it's start configuration is all 0.
    cfgs.push_back(this->GetIndividualCfg());
    for (size_t j = 0; j < cfgs[i].DOF(); j++) {
      (cfgs[i])[j] = 0.0;
    }

    // Dump the requested values into the cfg.
    for (size_t j = 0; j < vertex_poses[i].size(); j++) {
      // @TODO: ensure that we don't go past # of DOFs while dumping.
      (cfgs[i])[j] = (vertex_poses[i])[j];
    }

    // Add the Cfg to the map and capture the Vertex ID. Maintain ordering.
    v_ids.push_back(roadmap.AddVertex(cfgs[i]));
  }
  std::cout << "Part2" << std::endl;


  // Add all the requested edges.

  // Add start edges, don't care in what order we add them.
  // Use our distance metric to set the weight.

  std::cout << start_edges.size() << std::endl;
  for (auto it = start_edges.begin(); it != start_edges.end(); ++it) {
    std::cout << "h" << std::endl;
    DefaultWeight<typename MPTraits::CfgType> weight;

    weight.SetWeight(this->ComputeDist(vertex_poses[it->first],
                                       vertex_poses[it->second]));

    roadmap.AddEdge(v_ids[it->first], v_ids[it->second], weight);
  }
  std::cout << "Part3" << std::endl;


  // Run the Rewire Connector.
  this->Connect(&roadmap, v_ids.back());

  std::cout << "Part4" << std::endl;

  // Check the resulting edges.

  // Create an edge iterator just to call the function, no other purpose.
  typename RoadmapType::EI edge_itr;

  // Loop over the edges that should be in the map and verify if they exist.
  for (auto it = end_edges.begin(); it != end_edges.end(); ++it) {
    if (! roadmap.GetEdge(v_ids[it->first], v_ids[it->second], edge_itr)) {
      // Did not have expected edge, fail test.
      return false;
    }
  }
  std::cout << "Part5" << std::endl;

  return true;
}


template <typename MPTraits>
double
RewireConnectorTest<MPTraits>::
ComputeDist(std::vector<double> a, std::vector<double> b) {
  double dist = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    dist += (a[i] - b[i]) * (a[i] - b[i]);
  }

  return dist;
}

#endif
