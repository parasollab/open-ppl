#ifndef SVM_TEST_H_
#define SVM_TEST_H_

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "MPLibrary/LearningModels/SVMModel.h"

#include "nonstd/io.h"


////////////////////////////////////////////////////////////////////////////////
/// Testing ground for SVM boundary models, part 2.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class SVMTest : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    SVMTest();

    SVMTest(XMLNode& _node);

    virtual ~SVMTest() = default;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
SVMTest<MPTraits>::
SVMTest() : MPStrategyMethod<MPTraits>() {
  this->SetName("SVMTest");
}


template <typename MPTraits>
SVMTest<MPTraits>::
SVMTest(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("SVMTest");
}

/*------------------------------ Interface -----------------------------------*/

template <typename MPTraits>
void
SVMTest<MPTraits>::
Iterate() {
  MethodTimer mt(this->GetStatClass(), "Test Iteration");

  std::cout << "\nGenerating sample grid...\n" << std::endl;

  auto sampler = this->GetSampler("Grid");

  const Boundary* const envBoundary = this->GetEnvironment()->GetBoundary();
  const Boundary* const cspace = this->GetTask()->GetRobot()->GetCSpace();

  std::vector<CfgType> trainingGood, trainingBad;
  sampler->Sample(1, 10, cspace, std::back_inserter(trainingGood),
      std::back_inserter(trainingBad));

  for(const auto& cfg : trainingGood)
    this->GetRoadmap()->AddVertex(cfg);
  for(const auto& cfg : trainingBad)
    this->GetBlockRoadmap()->AddVertex(cfg);

  std::cout << "\nTraining SVM model...\n" << std::endl;

  SVMModel<MPTraits> svm(this->GetMPLibrary());
  svm.SetBoundary(cspace->Clone());
  svm.LoadData(trainingGood, trainingBad);
  svm.Train();

  std::cout << "\nGenerating test points...\n" << std::endl;

  const size_t numTestPoints = 10000;
  std::vector<CfgType> testingGood, testingBad;
  testingGood.reserve(numTestPoints);
  testingBad.reserve(numTestPoints);
  auto sampler2 = this->GetSampler("UniformRandomFree");
  sampler2->Sample(numTestPoints, 10, cspace, std::back_inserter(testingGood),
      std::back_inserter(testingBad));

  std::cout << "\nTesting...\n" << std::endl;

  auto vc = this->GetValidityChecker("pqp_solid");

  size_t goodCorrect   = 0,
         goodIncorrect = 0,
         badCorrect    = 0,
         badIncorrect  = 0;

  double goodSimilarityCorrect    = 0,
         goodSimilarityIncorrect  = 0,
         badSimilarityCorrect     = 0,
         badSimilarityIncorrect   = 0;

  size_t outOfBounds = 0,
         awayIsZero  = 0;

  auto testPoints = [&svm, &vc, &envBoundary, &outOfBounds, &awayIsZero](
      std::vector<CfgType>& _cfgs,
      size_t& _correct, size_t& _incorrect,
      double& _similarityCorrect, double& _similarityIncorrect)
  {
    for(auto& cfg : _cfgs) {
      // Get the full CD info.
      CDInfo cd(true);
      const bool valid = vc->IsValid(cfg, cd, "SVMTest::Iterate");

      // Check for in-bounds.
      const bool oob = !envBoundary->InBoundary(cfg);
      outOfBounds += oob;

      // Compute the actual direction away from the obstacle.
      const Vector3d awayFromObstacle = cd.m_robotPoint - cd.m_objectPoint;
      const bool zero = awayFromObstacle == Vector3d()
                    and cd.m_robotPoint == Vector3d()
                    and cd.m_objectPoint == Vector3d();
      awayIsZero += zero;

      // Compute the SVM predicted validity and gradient, which should be
      // roughly in the same direction as awayFromObstacle.
      const bool svmValid = svm.Classify(cfg);
      const CfgType gradient = svm.Gradient(cfg);

      // Compute metrics of interest.
      const bool validityCorrect = valid == svmValid;
      const double directionSimilarity = gradient.GetPoint().normalize()
                                       * awayFromObstacle.normalize();
      if(validityCorrect)
      {
        ++_correct;
        if(!zero)
          _similarityCorrect += directionSimilarity;
      }
      else
      {
        ++_incorrect;
        if(!zero)
          _similarityIncorrect += directionSimilarity;
      }

      std::cout << "\nChecking configuration " << cfg.PrettyPrint()
                << "\n\tValidity: " << (validityCorrect ? "" : "in") << "correct"
                << "\n\t\tPredicted: " << (svmValid ? "" : "in") << "valid"
                << "\n\t\tExpected:  " << (valid ? "" : "in") << "valid"
                << "\n\tDirectional similarity: " << directionSimilarity
                << "\n\t\tGradient: " << gradient.PrettyPrint()
                << "\n\t\trobot point: " << cd.m_robotPoint
                << "\n\t\tobject point: " << cd.m_objectPoint
                << "\n\t\tCD away:  " << awayFromObstacle
                << std::endl;
      if(zero)
        std::cout << "\t\tAway is zero."
                  << "\n\t\tObst index: " << cd.m_collidingObstIndex
                  << std::endl;
      if(oob)
        std::cout << "\t\tOut of bounds." << std::endl;
    }

    // Complete computation of average similarities.
    if(_correct)
      _similarityCorrect   /= _correct;
    if(_incorrect)
      _similarityIncorrect /= _incorrect;
  };

  // Test the points.
  testPoints(testingGood, goodCorrect, goodIncorrect, goodSimilarityCorrect,
      goodSimilarityIncorrect);
  testPoints(testingBad,  badCorrect,  badIncorrect,  badSimilarityCorrect,
      badSimilarityIncorrect);

  // Assert that the counts make sense.
  if(testingGood.size() != goodCorrect + goodIncorrect)
    throw RunTimeException(WHERE, "Number of good test points does not match "
        "metrics!");
  if(testingBad.size() != badCorrect + badIncorrect)
    throw RunTimeException(WHERE, "Number of bad test points does not match "
        "metrics!");

  // Report aggregate metrics.
  const double successRateGood = goodCorrect / double(testingGood.size()),
               successRateBad  = badCorrect  / double(testingBad.size()),
               successRateAll  = (goodCorrect + badCorrect) /
                                 double(testingGood.size() + testingBad.size());

  const double similarityGood = ( goodSimilarityCorrect   * goodCorrect
                                + goodSimilarityIncorrect * goodIncorrect)
                                / double(testingGood.size()),
               similarityBad  = ( badSimilarityCorrect   * badCorrect
                                + badSimilarityIncorrect * badIncorrect)
                                / double(testingBad.size()),
               similarityAll  = ( similarityGood * testingGood.size()
                                + similarityBad  * testingBad.size())
                                / double(testingGood.size() + testingBad.size());

  std::cout << "\nTest set info:"
            << "\nValid cfgs:   " << testingGood.size()
            << "\nInvalid cfgs: " << testingBad.size()
            << "\nOut of bounds: " << outOfBounds
            << std::endl;

  std::cout << "\nClassification performance:"
            << "\nSuccess rate for valid cfgs:   " << successRateGood
            << " (" << goodCorrect << "/" << testingGood.size() << ")"
            << "\nSuccess rate for invalid cfgs: " << successRateBad
            << " (" << badCorrect << "/" << testingBad.size() << ")"
            << "\nTotal success rate:            " << successRateAll
            << " (" << goodCorrect + badCorrect << "/"
            << testingGood.size() + testingBad.size() << ")"
            << std::endl;

  std::cout << "\nBoundary direction performance:"
            << "\nAverage similarity for correctly classified valid cfgs:     "
            << goodSimilarityCorrect
            << "\nAverage similarity for incorrectly classified valid cfgs:   "
            << goodSimilarityIncorrect
            << "\nAverage similarity for all valid cfgs:                      "
            << similarityGood
            << "\nAverage similarity for correctly classified invalid cfgs:   "
            << badSimilarityCorrect
            << "\nAverage similarity for incorrectly classified invalid cfgs: "
            << badSimilarityIncorrect
            << "\nAverage similarity for all invalid cfgs:                    "
            << similarityBad
            << "\nAverage similarity for all:                                 "
            << similarityAll
            << "\nAway is zero:                                               "
            << awayIsZero
            << std::endl;
}

/*----------------------------------------------------------------------------*/

#endif
