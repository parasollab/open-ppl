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

  if (passed)
    message = "RobotGroupFindNeighbors::PASSED!\n";
  else
    message = "RobotGroupFindNeighbors::FAILED :(\n" + message;
  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/
#endif
