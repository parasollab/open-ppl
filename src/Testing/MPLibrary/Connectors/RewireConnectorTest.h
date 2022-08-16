#ifndef PPL_CONNECTOR_METHOD_TEST_H_
#define PPL_CONNECTOR_METHOD_TEST_H_

#include "MPLibrary/Connectors/RewireConnector.h"
#include "Testing/MPLibrary/Connectors/ConnectorMethodTest.h"


template <class MPTraits>
class RewireConnectorTest : virtual public RewireConnector<MPTraits>,
                                 public ConnectorMethodTest<MPTraits> {
  public:

    /// @name Local Types
    /// @{

    typedef TestBaseObject::TestResult TestResult;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    /// @}
    /// @name Construction
    /// @{

    RewireConnectorTest();

    RewireConnectorTest(XMLNode& _node);

    ~RewireConnectorTest();

    /// @}

  private: 

    /// @name Interface Test Functions
    /// @{

    virtual TestResult IndividualRobotConnectTest() override;

    virtual TestResult RobotGroupConnectTest() override;

    /// @}


    /// @name Test Helper Functions
    /// @brief These functions automate test running.
    /// @{
    /// @param _vertexPoses Vector containing the Cfgs for the tree. Rewire is
    ///        called on the last vertex in the vector.
    /// @param _startEdges Vector containing the starting edges for the tree in
    ///        the form pair <A, B> gives an edge (A -> B).
    /// @param _endEdges Vector containing the expected edges after the rewire.
    bool IndividualRobotConnectRunTest(
            std::vector<std::vector<double>> _vertexPoses,
            std::vector<std::pair<size_t, size_t>> _startEdges,
            std::vector<std::pair<size_t, size_t>> _endEdges,
            bool _debug);

    /// @param _vertexPoses Vector containing the Cfgs for the tree. Rewire is
    ///        called on the last vertex in the vector.
    /// @param _startEdges Vector containing the starting edges for the tree in
    ///        the form pair <A, B> gives an edge (A -> B).
    /// @param _endEdges Vector containing the expected edges after the rewire.
    bool RobotGroupConnectRunTest(
            std::vector<std::vector<double>> _vertexPoses,
            std::vector<std::pair<size_t, size_t>> _startEdges,
            std::vector<std::pair<size_t, size_t>> _endEdges,
            bool _debug);

    /// @brief Hacky way to compute Euclidean distance since getting a distance
    ///        metric here is hard.
    /// @param _a first configuration as a vector.
    /// @param _b second configuration as a vector.
    double
    ComputeDist(std::vector<double> _a, std::vector<double> _b);
    /// @}
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

  // Indicates if debug information should be printed.
  bool debug = false;

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
  // IMPORTANT NOTE: The last vertex given in _vertexPoses is assumed to be
  //  the vertex which connect is called on! I.e. the most recently added
  //  vertex to the tree.


  // ///////////////////////////////////////////////////////////////////////////
  // Test1: Rewire triangle.
  //  Before:      After:
  //  1             1
  //  |\            |
  //  | \           |
  //  R  2          R--2

  // Print the name of the test if debug is request.
  std::string testName = "Rewire triangle";
  if (debug) {
    std::cout << "\n\tRunning test '" << testName << "'" << std::endl;
  }

  // Vertices
  std::vector<double> p_root = {0.0, 0.0};
  std::vector<double> p_1    = {0.0, 1.0};
  std::vector<double> p_2    = {1.0, 0.0};

  std::vector<std::vector<double>> verts;
  verts.push_back(p_root);
  verts.push_back(p_1);
  verts.push_back(p_2);

  // Start edges.
  std::vector<std::pair<size_t, size_t>> startEdges;
  startEdges.push_back(std::pair<size_t, size_t>(0, 1));
  startEdges.push_back(std::pair<size_t, size_t>(1, 2));

  // Expected end edges.
  std::vector<std::pair<size_t, size_t>> endEdges;
  endEdges.push_back(std::pair<size_t, size_t>(0, 1));
  endEdges.push_back(std::pair<size_t, size_t>(0, 2));

  // Call the helper function, last argument indicates debug messages or not.
  if (! IndividualRobotConnectRunTest(verts, startEdges, endEdges, debug)) {
    // Test failed!
    passed = false;
    message += "\tFailed Test1: '" + testName + "'.\n";
  }


  // Test 2: Rewire Large Tree.
  //
  // Print the name of the test if debug is request.
  testName = "Rewire Large Tree";
  if (debug) {
    std::cout << "\n\tRunning test '" << testName << "'" << std::endl;
  }

  // Vertices.
  p_root = {0.0, 0.0};
  p_1 = {0.0, 4.0};
  p_2 = {0.0, 3.0};
  std::vector<double> p_3 = {3.0, 3.0};
  std::vector<double> p_4 = {0.0, 2.0};
  std::vector<double> p_5 = {1.0, 2.0};
  std::vector<double> p_6 = {4.0, 2.0};
  std::vector<double> p_7 = {2.0, 1.0};
  std::vector<double> p_8 = {4.0, 1.0};
  std::vector<double> p_9 = {2.0, 0.0};
  std::vector<double> p_10 = {4.0, 0.0};
  std::vector<double> p_11 = {2.0, 3.0};
  
  verts.clear();
  verts.push_back(p_root);
  verts.push_back(p_1);
  verts.push_back(p_2);
  verts.push_back(p_3);
  verts.push_back(p_4);
  verts.push_back(p_5);
  verts.push_back(p_6);
  verts.push_back(p_7);
  verts.push_back(p_8);
  verts.push_back(p_9);
  verts.push_back(p_10);
  verts.push_back(p_11);

  // Start edges.
  startEdges.clear();
  startEdges.push_back(std::pair<size_t, size_t>(0, 4));
  startEdges.push_back(std::pair<size_t, size_t>(0, 5));
  startEdges.push_back(std::pair<size_t, size_t>(0, 7));
  startEdges.push_back(std::pair<size_t, size_t>(0, 9));
  startEdges.push_back(std::pair<size_t, size_t>(4, 2));
  startEdges.push_back(std::pair<size_t, size_t>(2, 1));
  startEdges.push_back(std::pair<size_t, size_t>(7, 3));
  startEdges.push_back(std::pair<size_t, size_t>(7, 6));
  startEdges.push_back(std::pair<size_t, size_t>(7, 8));
  startEdges.push_back(std::pair<size_t, size_t>(7, 10));
  startEdges.push_back(std::pair<size_t, size_t>(3, 11));

  // Expected end edges.
  endEdges.clear();
  endEdges.push_back(std::pair<size_t, size_t>(0, 4));
  endEdges.push_back(std::pair<size_t, size_t>(0, 5));
  endEdges.push_back(std::pair<size_t, size_t>(0, 7));
  endEdges.push_back(std::pair<size_t, size_t>(0, 9));
  endEdges.push_back(std::pair<size_t, size_t>(2, 1));
  endEdges.push_back(std::pair<size_t, size_t>(4, 2));
  endEdges.push_back(std::pair<size_t, size_t>(5, 11));
  endEdges.push_back(std::pair<size_t, size_t>(7, 6));
  endEdges.push_back(std::pair<size_t, size_t>(7, 8));
  endEdges.push_back(std::pair<size_t, size_t>(7, 10));
  endEdges.push_back(std::pair<size_t, size_t>(11, 3));

  // Call the helper function.
  if (! IndividualRobotConnectRunTest(verts, startEdges, endEdges, debug)) {
    // Test failed!
    passed = false;
    message += "\tFailed Test2: Rewire large tree.\n";
  }


  // Test 3: Rewire shortest path.
  //
  // Print the name of the test if debug is request.
  testName = "Rewire shortest path";
  if (debug) {
    std::cout << "\n\tRunning test '" << testName << "'" << std::endl;
  }

  // Vertices.
  p_root = {0.0, 0.0};
  p_1 = {1.0, 2.0};
  p_2 = {0.0, 1.0};
  p_3 = {0.5, 1.0};
  p_4 = {1.0, 0.0};
  p_5 = {2.0, 0.0};
  p_6 = {2.0, 1.0};

  verts.clear();
  verts.push_back(p_root);
  verts.push_back(p_1);
  verts.push_back(p_2);
  verts.push_back(p_3);
  verts.push_back(p_4);
  verts.push_back(p_5);
  verts.push_back(p_6);

  // Start edges.
  
  startEdges.clear();
  startEdges.push_back(std::pair<size_t, size_t>(0, 2));
  startEdges.push_back(std::pair<size_t, size_t>(0, 3));
  startEdges.push_back(std::pair<size_t, size_t>(0, 4));
  startEdges.push_back(std::pair<size_t, size_t>(2, 1));
  startEdges.push_back(std::pair<size_t, size_t>(4, 5));
  startEdges.push_back(std::pair<size_t, size_t>(5, 6));
  
  // Expected end edges.

  endEdges.clear();
  endEdges.push_back(std::pair<size_t, size_t>(0, 2));
  endEdges.push_back(std::pair<size_t, size_t>(0, 3));
  endEdges.push_back(std::pair<size_t, size_t>(0, 4));
  endEdges.push_back(std::pair<size_t, size_t>(2, 1));
  endEdges.push_back(std::pair<size_t, size_t>(4, 5));
  endEdges.push_back(std::pair<size_t, size_t>(4, 6));

  // Call the helper function.
  if (! IndividualRobotConnectRunTest(verts, startEdges, endEdges, debug)) {
    // Test failed!
    passed = false;
    message += "\tFailed Test3: Rewire Shortest Path.\n";
  }

  // Clear data structures for the next test.
  testName = "TODO: TEST NAME";
  verts.clear();
  startEdges.clear();
  endEdges.clear();
  // ///////////////////////////////////////////////////////////////////////////


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

  // Indicates if debug information should be printed.
  bool debug = false;

  bool passed = true;
  std::string message = "";


  // Begin running Tests.
  //
  // IMPORTANT NOTE: The last vertex given in _vertexPoses is assumed to be
  //  the vertex which connect is called on! I.e. the most recently added
  //  vertex to the tree.
  //
  // The tests here are identical to those for individual robot. The difference
  //  is that instead of 1 robot, 2 are used. The second robot has the exact
  //  same individual configurations, but mirrored and offset.


  // ///////////////////////////////////////////////////////////////////////////
  // Test1: Rewire triangle.
  //  Before:      After:
  //  1             1
  //  |\            |
  //  | \           |
  //  R  2          R--2

  // Print the name of the test if debug is request.
  std::string testName = "Rewire triangle";
  if (debug) {
    std::cout << "\n\tRunning test '" << testName << "'" << std::endl;
  }

  // Vertices
  std::vector<double> p_root = {0.0, 0.0};
  std::vector<double> p_1    = {0.0, 1.0};
  std::vector<double> p_2    = {1.0, 0.0};

  std::vector<std::vector<double>> verts;
  verts.push_back(p_root);
  verts.push_back(p_1);
  verts.push_back(p_2);

  // Start edges.
  std::vector<std::pair<size_t, size_t>> startEdges;
  startEdges.push_back(std::pair<size_t, size_t>(0, 1));
  startEdges.push_back(std::pair<size_t, size_t>(1, 2));

  // Expected end edges.
  std::vector<std::pair<size_t, size_t>> endEdges;
  endEdges.push_back(std::pair<size_t, size_t>(0, 1));
  endEdges.push_back(std::pair<size_t, size_t>(0, 2));

  // Call the helper function, last argument indicates debug messages or not.
  if (! RobotGroupConnectRunTest(verts, startEdges, endEdges, debug)) {
    // Test failed!
    passed = false;
    message += "\tFailed Test1: '" + testName + "'.\n";
  }


  // Test 2: Rewire Large Tree.
  //
  // Print the name of the test if debug is request.
  testName = "Rewire Large Tree";
  if (debug) {
    std::cout << "\n\tRunning test '" << testName << "'" << std::endl;
  }

  // Vertices.
  p_root = {0.0, 0.0};
  p_1 = {0.0, 4.0};
  p_2 = {0.0, 3.0};
  std::vector<double> p_3 = {3.0, 3.0};
  std::vector<double> p_4 = {0.0, 2.0};
  std::vector<double> p_5 = {1.0, 2.0};
  std::vector<double> p_6 = {4.0, 2.0};
  std::vector<double> p_7 = {2.0, 1.0};
  std::vector<double> p_8 = {4.0, 1.0};
  std::vector<double> p_9 = {2.0, 0.0};
  std::vector<double> p_10 = {4.0, 0.0};
  std::vector<double> p_11 = {2.0, 3.0};
  
  verts.clear();
  verts.push_back(p_root);
  verts.push_back(p_1);
  verts.push_back(p_2);
  verts.push_back(p_3);
  verts.push_back(p_4);
  verts.push_back(p_5);
  verts.push_back(p_6);
  verts.push_back(p_7);
  verts.push_back(p_8);
  verts.push_back(p_9);
  verts.push_back(p_10);
  verts.push_back(p_11);

  // Start edges.
  startEdges.clear();
  startEdges.push_back(std::pair<size_t, size_t>(0, 4));
  startEdges.push_back(std::pair<size_t, size_t>(0, 5));
  startEdges.push_back(std::pair<size_t, size_t>(0, 7));
  startEdges.push_back(std::pair<size_t, size_t>(0, 9));
  startEdges.push_back(std::pair<size_t, size_t>(4, 2));
  startEdges.push_back(std::pair<size_t, size_t>(2, 1));
  startEdges.push_back(std::pair<size_t, size_t>(7, 3));
  startEdges.push_back(std::pair<size_t, size_t>(7, 6));
  startEdges.push_back(std::pair<size_t, size_t>(7, 8));
  startEdges.push_back(std::pair<size_t, size_t>(7, 10));
  startEdges.push_back(std::pair<size_t, size_t>(3, 11));

  // Expected end edges.
  endEdges.clear();
  endEdges.push_back(std::pair<size_t, size_t>(0, 4));
  endEdges.push_back(std::pair<size_t, size_t>(0, 5));
  endEdges.push_back(std::pair<size_t, size_t>(0, 7));
  endEdges.push_back(std::pair<size_t, size_t>(0, 9));
  endEdges.push_back(std::pair<size_t, size_t>(2, 1));
  endEdges.push_back(std::pair<size_t, size_t>(4, 2));
  endEdges.push_back(std::pair<size_t, size_t>(5, 11));
  endEdges.push_back(std::pair<size_t, size_t>(7, 3));
  endEdges.push_back(std::pair<size_t, size_t>(7, 6));
  endEdges.push_back(std::pair<size_t, size_t>(7, 8));
  endEdges.push_back(std::pair<size_t, size_t>(7, 10));

  // Call the helper function.
  if (! RobotGroupConnectRunTest(verts, startEdges, endEdges, debug)) {
    // Test failed!
    passed = false;
    message += "\tFailed Test2: Rewire large tree.\n";
  }


  // Test 3: Rewire shortest path.
  //
  // Print the name of the test if debug is request.
  testName = "Rewire shortest path";
  if (debug) {
    std::cout << "\n\tRunning test '" << testName << "'" << std::endl;
  }

  // Vertices.
  p_root = {0.0, 0.0};
  p_1 = {1.0, 2.0};
  p_2 = {0.0, 1.0};
  p_3 = {0.5, 1.0};
  p_4 = {1.0, 0.0};
  p_5 = {2.0, 0.0};
  p_6 = {2.0, 1.0};

  verts.clear();
  verts.push_back(p_root);
  verts.push_back(p_1);
  verts.push_back(p_2);
  verts.push_back(p_3);
  verts.push_back(p_4);
  verts.push_back(p_5);
  verts.push_back(p_6);

  // Start edges.
  
  startEdges.clear();
  startEdges.push_back(std::pair<size_t, size_t>(0, 2));
  startEdges.push_back(std::pair<size_t, size_t>(0, 3));
  startEdges.push_back(std::pair<size_t, size_t>(0, 4));
  startEdges.push_back(std::pair<size_t, size_t>(2, 1));
  startEdges.push_back(std::pair<size_t, size_t>(4, 5));
  startEdges.push_back(std::pair<size_t, size_t>(5, 6));
  
  // Expected end edges.

  endEdges.clear();
  endEdges.push_back(std::pair<size_t, size_t>(0, 2));
  endEdges.push_back(std::pair<size_t, size_t>(0, 3));
  endEdges.push_back(std::pair<size_t, size_t>(0, 4));
  endEdges.push_back(std::pair<size_t, size_t>(2, 1));
  endEdges.push_back(std::pair<size_t, size_t>(4, 5));
  endEdges.push_back(std::pair<size_t, size_t>(4, 6));
  
  // Call the helper function.
  if (! RobotGroupConnectRunTest(verts, startEdges, endEdges, debug)) {
    // Test failed!
    passed = false;
    message += "\tFailed Test3: Rewire Shortest Path.\n";
  }

  // Clear data structures for the next test.
  testName = "TODO: TEST NAME";
  verts.clear();
  startEdges.clear();
  endEdges.clear();
  // ///////////////////////////////////////////////////////////////////////////


  if(passed) {
    message = "RobotGroupConnectTest::PASSED!\n";
  }
  else {
    message = "RobotGroupConnectTest::FAILED :(\n" + message + "\n";
  }

  return std::make_pair(passed,message);
}


/*----------------------- Test Helper Functions ----------------------*/

template <typename MPTraits>
bool
RewireConnectorTest<MPTraits>::
IndividualRobotConnectRunTest(
        std::vector<std::vector<double>> _vertexPoses,
        std::vector<std::pair<size_t, size_t>> _startEdges,
        std::vector<std::pair<size_t, size_t>> _endEdges,
        bool _debug) {

  // Create the roadmap.
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  RoadmapType roadmap(robot);


  // Add Cfgs to roadmap. (Collect Vertex IDs to refer to them later)

  // Vector to create cfgs.
  std::vector<typename MPTraits::CfgType> cfgs;
  // Vector to capture Vertex IDs for use later.
  std::vector<size_t> vIds;


  for (size_t i = 0; i < _vertexPoses.size(); i++) {
    // Create a new Cfg and ensure that it's start configuration is all 0.
    cfgs.push_back(this->GetIndividualCfg());
    for (size_t j = 0; j < cfgs[i].DOF(); j++) {
      (cfgs[i])[j] = 0.0;
    }

    // Dump the requested values into the cfg.
    for (size_t j = 0; j < _vertexPoses[i].size(); j++) {
      // @TODO: ensure that we don't go past # of DOFs while dumping.
      (cfgs[i])[j] = (_vertexPoses[i])[j];
    }

    // Add the Cfg to the map and capture the Vertex ID. Maintain ordering.
    vIds.push_back(roadmap.AddVertex(cfgs[i]));
  }


  // Add all the requested edges.

  // Add start edges, don't care in what order we add them.
  // Use our distance metric to set the weight.

  for (auto it = _startEdges.begin(); it != _startEdges.end(); ++it) {
    DefaultWeight<typename MPTraits::CfgType> weight;

    weight.SetWeight(this->ComputeDist(_vertexPoses[it->first],
                                       _vertexPoses[it->second]));

    roadmap.AddEdge(vIds[it->first], vIds[it->second], weight);
  }

  // Variable needed to call GetEdge, we don't use it otherwise.
  typename RoadmapType::EI edgeItr;

  if (_debug) {
    std::cout << "\n\t\tPrinting original edges:" << std::endl;
    for (size_t i = 0; i < _vertexPoses.size(); ++i) {
      for (size_t j = 0; j < _vertexPoses.size(); ++j) {
        auto edgeBool = roadmap.GetEdge(vIds[i], vIds[j], edgeItr);
        std::cout << "\t\t\tEdge " << i << ", " << j
                  << " ?   " << (edgeBool == 1 ? "True" : "  False")
                  << std::endl;
      }
    }
  }


  // Run the Rewire Connector.
  this->Connect(&roadmap, vIds.back());


  // Check the resulting edges.

  // First, if _debug, print resulting edges.

  if (_debug) {
    std::cout << "\n\t\tPrinting rewired edges:" << std::endl;
    for (size_t i = 0; i < _vertexPoses.size(); ++i) {
      for (size_t j = 0; j < _vertexPoses.size(); ++j) {
        auto edgeBool = roadmap.GetEdge(vIds[i], vIds[j], edgeItr);
        std::cout << "\t\t\tEdge " << i << ", " << j
                  << " ?   " << (edgeBool == 1 ? "True" : "  False")
                  << std::endl;
      }
    }
  }

  // Loop over the edges that should be in the map and verify if they exist.
  if (_debug) {
    std::cout << "\n\t\tPrinting test results:" << std::endl;
  }

  bool ret = true;
  for (auto it = _endEdges.begin(); it != _endEdges.end(); ++it) {
    auto edgeBool =
            roadmap.GetEdge(vIds[it->first], vIds[it->second], edgeItr);

    // If _debug, print the edge checks as we go through them.
    if (_debug) {
      std::cout << "\t\t\tEdge " << it->first << ", " << it->second
                << " ?   " << (edgeBool == 1 ? "True" : "  False")
                << std::endl;
    }

    if (!edgeBool) {
      // Did not have expected edge, fail test.
      ret = false;
    }
  }

  if (_debug) {
    std::cout << "\n\t\tTest Pass/Fail ?  " << (ret ? "PASSED" : "FAILED")
              << std::endl;
  }

  return ret;
}


template <typename MPTraits>
bool
RewireConnectorTest<MPTraits>::
RobotGroupConnectRunTest(
        std::vector<std::vector<double>> _vertexPoses,
        std::vector<std::pair<size_t, size_t>> _startEdges,
        std::vector<std::pair<size_t, size_t>> _endEdges,
        bool _debug) {

  // Create the roadmap.
  auto group = this->GetMPProblem()->GetRobotGroups()[0].get();

  // Create MPSolution.
  typename MPTraits::MPSolution mpSolution(this->GetMPProblem()->GetRobotGroups()[0].get());
  for (auto robot:group->GetRobots()) {
    mpSolution.AddRobot(robot);
  }

  mpSolution.AddRobotGroup(group);
  auto roadmap = mpSolution.GetGroupRoadmap(group);

  // Add Cfgs to roadmap. (Collect Vertex IDs to refer to them later)

  // Vector to create cfgs.
  std::vector<GroupCfgType> cfgs;
  // Vector to capture Vertex IDs for use later.
  std::vector<size_t> vIds;


  for (size_t i = 0; i < _vertexPoses.size(); i++) {
    // Create a new group Cfg and ensure that both robot's start config is 0.
    cfgs.push_back(GroupCfgType(roadmap));
    // for (size_t j = 0; j < cfgs[i].DOF(); j++) {
    //   (cfgs[i]).GetRobotCfg(size_t(0))[j] = 0.0;
    //   (cfgs[i]).GetRobotCfg(size_t(1))[j] = 0.0;
    // }

    // Dump the requested values into the cfg.
    for (size_t j = 0; j < _vertexPoses[i].size(); j++) {
      // @TODO: ensure that we don't go past # of DOFs while dumping.
      (cfgs[i]).GetRobotCfg(size_t(0))[j] = (_vertexPoses[i])[j] + 1;
      (cfgs[i]).GetRobotCfg(size_t(1))[j] = -((_vertexPoses[i])[j] + 1);
    }

    // Add the Cfg to the map and capture the Vertex ID. Maintain ordering.
    vIds.push_back(roadmap->AddVertex(cfgs[i]));
  }


  // Add all the requested edges.

  // Add start edges, don't care in what order we add them.
  // Use our distance metric to set the weight.

  for (auto it = _startEdges.begin(); it != _startEdges.end(); ++it) {
    auto dm = this->GetDistanceMetric("euclidean");
    auto distance = dm->Distance(cfgs[it->first], cfgs[it->second]);
    auto rmap1 = roadmap->GetIndividualGraph(0);
    auto rmap2 = roadmap->GetIndividualGraph(1);

    DefaultWeight<typename MPTraits::CfgType> w;
    w.SetWeight(distance);
    rmap1->AddEdge(vIds[it->first], vIds[it->second], w);
    rmap2->AddEdge(vIds[it->first], vIds[it->second], w);

    typename MPTraits::GroupWeightType weight(roadmap);

    weight.SetWeight(distance);

    roadmap->AddEdge(vIds[it->first], vIds[it->second], weight);
  }

  // Variable needed to call GetEdge, we don't use it otherwise.
  typename MPTraits::GroupRoadmapType::EI edgeItr;

  if (_debug) {
    std::cout << "\n\t\tPrinting original edges:" << std::endl;
    for (size_t i = 0; i < _vertexPoses.size(); ++i) {
      for (size_t j = 0; j < _vertexPoses.size(); ++j) {
        auto edgeBool = roadmap->GetEdge(vIds[i], vIds[j], edgeItr);
        std::cout << "\t\t\tEdge " << i << ", " << j
                  << " ?   " << (edgeBool == 1 ? "True" : "  False")
                  << std::endl;
      }
    }
  }


  // Run the Rewire Connector.
  this->Connect(roadmap, vIds.back());


  // Check the resulting edges.

  // First, if _debug, print resulting edges.

  if (_debug) {
    std::cout << "\n\t\tPrinting rewired edges:" << std::endl;
    for (size_t i = 0; i < _vertexPoses.size(); ++i) {
      for (size_t j = 0; j < _vertexPoses.size(); ++j) {
        auto edgeBool = roadmap->GetEdge(vIds[i], vIds[j], edgeItr);
        std::cout << "\t\t\tEdge " << i << ", " << j
                  << " ?   " << (edgeBool == 1 ? "True" : "  False")
                  << std::endl;
      }
    }
  }

  // Loop over the edges that should be in the map and verify if they exist.
  if (_debug) {
    std::cout << "\n\t\tPrinting test results:" << std::endl;
  }

  bool ret = true;
  for (auto it = _endEdges.begin(); it != _endEdges.end(); ++it) {
    auto edgeBool =
            roadmap->GetEdge(vIds[it->first], vIds[it->second], edgeItr);

    // If _debug, print the edge checks as we go through them.
    if (_debug) {
      std::cout << "\t\t\tEdge " << it->first << ", " << it->second
                << " ?   " << (edgeBool == 1 ? "True" : "  False")
                << std::endl;
    }

    if (!edgeBool) {
      // Did not have expected edge, fail test.
      ret = false;
    }
  }

  if (_debug) {
    std::cout << "\n\t\tTest Pass/Fail ?  " << (ret ? "PASSED" : "FAILED")
              << std::endl;
  }

  return ret;
}


template <typename MPTraits>
double
RewireConnectorTest<MPTraits>::
ComputeDist(std::vector<double> _a, std::vector<double> _b) {
  double dist = 0.0;
  for (size_t i = 0; i < _a.size(); ++i) {
    dist += (_a[i] - _b[i]) * (_a[i] - _b[i]);
  }

  return dist;
}

#endif
