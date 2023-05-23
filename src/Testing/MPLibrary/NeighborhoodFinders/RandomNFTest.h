#ifndef PPL_RANDOM_NF_TEST_H_
#define PPL_RANDOM_NF_TEST_H_

#include "MPLibrary/NeighborhoodFinders/RandomNF.h"
#include "Testing/MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethodTest.h"
#include <boost/math/distributions/chi_squared.hpp>

template <typename MPTraits>
class RandomNFTest : virtual public RandomNF<MPTraits>,
                         public NeighborhoodFinderMethodTest<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    typedef typename NeighborhoodFinderMethod<MPTraits>::OutputIterator OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    RandomNFTest();

    RandomNFTest(XMLNode& _node);

    ~RandomNFTest();

    ///@}

  private:

    ///@name Internal State
    ///@{

    size_t m_numSamples{0};              ///< How many times to sample neighbors

    ///@}

    ///@name Interface Test Functions
    ///@{
  
    virtual TestResult IndividualRobotFindNeighborsTest() override;

    virtual TestResult RobotGroupFindNeighborsTest() override;
    
    bool ChiSquaredWithinCriticalVal(std::vector<std::vector<int>>& _observedSamples,
        int _numNodes, int _numSamples, double _pVal);

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>    
RandomNFTest<MPTraits>::
RandomNFTest() : RandomNF<MPTraits>() {}

template <typename MPTraits>    
RandomNFTest<MPTraits>::
RandomNFTest(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node, Type::K),
                               RandomNF<MPTraits>(_node) {
  m_numSamples = 100;

}

template <typename MPTraits>    
RandomNFTest<MPTraits>::
~RandomNFTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>    
typename RandomNFTest<MPTraits>::TestResult
RandomNFTest<MPTraits>::
IndividualRobotFindNeighborsTest() {
  bool passed = true;
  std::string message = "";
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  RoadmapType* roadmap = new RoadmapType(robot);
  int n = 5;

  std::vector<std::vector<int>> arr(n, vector<int>(n,0));

  std::vector<size_t> vertices;
  for (int c = 1; c <= n; ++c) {
    auto p1 = CfgType(robot);
    std::istringstream p1Stream("0 " + std::to_string(c) + " 0 0 0 0 0 0");
    p1.Read(p1Stream);
    auto v1 = roadmap->AddVertex(p1);
    vertices.push_back(v1); // Add cfg to the roadmap
  }

  for (size_t j = 0; j < vertices.size(); j++) { 
    // For each vertex, find neighbors numSamples times
    for(size_t i = 0; i < this->m_numSamples; i++) {
      std::vector<Neighbor> neighbors;
      OutputIterator output(neighbors); 
      auto cfg = roadmap->GetVertex(vertices[j]);
      this->RandomNF<MPTraits>::FindNeighbors(roadmap, cfg, roadmap->GetAllVIDs(), output);
      // Count up rhow many times connected to random neighbors
      for (size_t k = 0; k < neighbors.size(); k++){
        for (size_t m = 0; m < vertices.size(); m++){
          if(vertices[m] == neighbors[k].target)
            arr[j][m] += 1;
        }
      }
    }
  }

  passed = ChiSquaredWithinCriticalVal(arr, n, this->m_numSamples, 0.05);

  if (passed)
    message = "IndividualRobotFindNeighbors::PASSED!\n";
  else
    message = "IndividualRobotFindNeighbors::FAILED :(\n" + message;

  return std::make_pair(passed,message);
}

template <typename MPTraits>    
typename RandomNFTest<MPTraits>::TestResult
RandomNFTest<MPTraits>::
RobotGroupFindNeighborsTest() {

  bool passed = true;
  std::string message = "";

  if (passed)
    message = "Redundant test -> RobotGroupFindNeighbors::PASSED!\n";
  else
    message = "RobotGroupFindNeighbors::FAILED :(\n" + message;
  return std::make_pair(passed,message);

}

template <typename MPTraits> 
bool
RandomNFTest<MPTraits>::
ChiSquaredWithinCriticalVal(std::vector<std::vector<int>>& _observedSamples,
        int _numNodes, int _numSamples, double _pVal) {
  // create chi squared distribution with appropriate dof
  // 2 less instead of one because no connections to self
  boost::math::chi_squared chiDist(_numNodes-2);
  double expectedSamples = this->m_k * _numSamples/(_numNodes-1);
  // std::cout << expectedSamples << std::endl;

  for(int i=0; i<_numNodes; i++) {
    // copy observed probabilities for node i
    std::vector<int> nodeObservedProbs(_observedSamples[i]);
    // erase probability of connecting to self because should be 0
    nodeObservedProbs.erase(nodeObservedProbs.begin() + i);
    // compute observed chi square statistic
    double chiSquaredStat = 0;
    for(int j=0; j<_numNodes-1; j++) {
      double diff = nodeObservedProbs[j]-expectedSamples;
      chiSquaredStat += (diff * diff)/expectedSamples;
    }
    double criticalValue = boost::math::quantile(boost::math::complement(chiDist, _pVal));

    if(chiSquaredStat > criticalValue) {
      std::cout << "Fail: " << i  << " " << chiSquaredStat << " " << criticalValue << " " << nodeObservedProbs << std::endl;
      return false;
    }
  }

  return true;
}
/*--------------------------------------------------------------------*/
#endif
