#ifndef PPL_BRUTE_FORCE_NF_TEST_H_
#define PPL_BRUTE_FORCE_NF_TEST_H_

#include "MPLibrary/NeighborhoodFinders/BruteForceNF.h"
#include "Testing/MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethodTest.h"

template <typename MPTraits>
class BruteForceNFTest : virtual public BruteForceNF<MPTraits>,
                         public NeighborhoodFinderMethodTest<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;

    typedef typename NeighborhoodFinderMethod<MPTraits>::OutputIterator OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    BruteForceNFTest();

    BruteForceNFTest(XMLNode& _node);

    ~BruteForceNFTest();

    ///@}

  private:

    ///@name Interface Test Functions
    ///@{
  
    virtual TestResult IndividualRobotFindNeighborsTest() override;

    virtual TestResult RobotGroupFindNeighborsTest() override;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>    
BruteForceNFTest<MPTraits>::
BruteForceNFTest() : BruteForceNF<MPTraits>() {}

template <typename MPTraits>    
BruteForceNFTest<MPTraits>::
BruteForceNFTest(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node),
                                   BruteForceNF<MPTraits>(_node) {}

template <typename MPTraits>    
BruteForceNFTest<MPTraits>::
~BruteForceNFTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>    
typename BruteForceNFTest<MPTraits>::TestResult
BruteForceNFTest<MPTraits>::
IndividualRobotFindNeighborsTest() {

  bool passed = true;
  std::string message = "";

  auto robot = this->GetMPProblem()->GetRobots()[0].get();

  RoadmapType* roadmap = new RoadmapType(robot);

  // Create vertices and add them to the roadmap
  auto p1 = CfgType(robot);
  std::istringstream p1Stream("0 0 0 0 0 0 0");
  p1.Read(p1Stream);
  roadmap->AddVertex(p1);

  auto p2 = CfgType(robot);
  std::istringstream p2Stream("0 1 0 0 0 0 0");
  p2.Read(p2Stream);
  auto goal = roadmap->AddVertex(p2);

  auto v1P = CfgType(robot);
  std::istringstream v1Stream("0 2 0 0 0 0 0");
  v1P.Read(v1Stream);
  auto v1 = roadmap->AddVertex(v1P);

  auto v2P = CfgType(robot);
  std::istringstream v2Stream("0 3 0 0 0 0 0");
  v2P.Read(v2Stream);
  auto v2 = roadmap->AddVertex(v2P);

  auto v3P = CfgType(robot);
  std::istringstream v3Stream("0 4 0 0 0 0 0");
  v3P.Read(v3Stream);
  auto v3 = roadmap->AddVertex(v3P);

  // Keep a vector of the vertices sorted by distance from p1
  std::vector<size_t> vec = {goal, v1, v2, v3};

  // Find the k nearest neighbors of p1
  std::vector<Neighbor> neighbors;
  OutputIterator output(neighbors); 

  this->FindNeighbors(roadmap, p1, roadmap->GetAllVIDs(), output); 

  // Check if k >= the number of vertices in the roadmap. If so, check that
  // the NF algorithm returned all of the vertices in the roadmap (except for start).
  // k = 0 means get all neighbors
  if (this->m_k >= roadmap->Size() or this->m_k == 0) {
    if (neighbors.size() != (roadmap->Size() - 1)) {
      passed = false; 
      message = message + "\n\tThe neighborhood finder did not return the"
                          " correct number of neighbors.\n";
    }
  } else {
    // Check that k neighbors were returned
    if (neighbors.size() != this->m_k) {
      passed = false; 
      message = message + "\n\tThe neighborhood finder did not return the"
                          " correct number of neighbors.\n";
    } else {
      // Check that each of the returned neighbors are closer than the 
      // (k+1)th closest vertex to start
      size_t distance_max = vec[this->m_k];

      for (size_t i =0; i < neighbors.size(); i++) {
        if (neighbors[i].distance > distance_max) {
          passed = false; 
          message = message + "\n\tThe neighborhood finder did not return the"
                          " k nearest neighbors.\n";
          break;
        }
      }
    }
  }

  if (passed)
    message = "IndividualRobotFindNeighbors::PASSED!\n";
  else
    message = "IndividualRobotFindNeighbors::FAILED :(\n" + message;

  return std::make_pair(passed,message);
}

template <typename MPTraits>    
typename BruteForceNFTest<MPTraits>::TestResult
BruteForceNFTest<MPTraits>::
RobotGroupFindNeighborsTest() {

  bool passed = true;
  std::string message = "";

  // TODO Need support for Groups

  if (passed)
    message = "RobotGroupFindNeighbors::PASSED!\n";
  else
    message = "RobotGroupFindNeighbors::FAILED :(\n" + message;
  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/
#endif
