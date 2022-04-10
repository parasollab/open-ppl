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

  auto robot = NeighborhoodFinderMethodTest<MPTraits>::GetMPProblem()->GetRobots()[0].get();

  RoadmapType* roadmap = new RoadmapType(robot);

  auto p1 = CfgType(robot);
  std::istringstream p1Stream("0 0 0 0 0 0 0");
  p1.Read(p1Stream);
  auto start = roadmap->AddVertex(p1);

  auto p2 = CfgType(robot);
  std::istringstream p2Stream("0 10 -10 0 0.5 3 0.9");
  p2.Read(p2Stream);
  auto goal = roadmap->AddVertex(p2);

  auto v1P = CfgType(robot);
  std::istringstream v1Stream("0 9 -4 0 0.3 0.7 0.1");
  v1P.Read(v1Stream);
  auto v1 = roadmap->AddVertex(v1P);

  auto v2P = CfgType(robot);
  std::istringstream v2Stream("0 20 0 0 0 0 0");
  v2P.Read(v2Stream);
  auto v2 = roadmap->AddVertex(v2P);

  roadmap->AddEdge(start, goal, WeightType("a",1.0));
  roadmap->AddEdge(start, v1, WeightType("b",2.0));
  roadmap->AddEdge(start, v2, WeightType("c",3.0));

  OutputIterator output; 

  this->FindNeighbors(roadmap, start, nullptr, output); 

  std::vector<size_t> vec;
  vec.push_back(goal);
  vec.push_back(v1);
  vec.push_back(v2);

   if (this->m_k >= roadmap->Size()) {

    if (output.size() == (roadmap->Size() -1)) {

     passed = true; 
     message = "IndividualRobotFindNeighbors::PASSED :(\n" + message;
    }
   }
   else {

    size_t distance_max = vec[this->m_k];

    for (size_t i =0; i < output.size(); i++) {
      if (output[i].distance > distance_max) {
      passed = false; 
      message = "IndividualRobotFindNeighbors::FAILED!\n";
      }
    }

  }

   return std::make_pair(passed,message);
}

template <typename MPTraits>    
typename BruteForceNFTest<MPTraits>::TestResult
BruteForceNFTest<MPTraits>::
RobotGroupFindNeighborsTest() {

  bool passed = true;
  std::string message = "";

  if (passed)
    message = "RobotGroupFindNeighbors::PASSED!\n";
  else
    message = "RobotGroupFindNeighbors::FAILED :(\n" + message;
  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/
#endif
