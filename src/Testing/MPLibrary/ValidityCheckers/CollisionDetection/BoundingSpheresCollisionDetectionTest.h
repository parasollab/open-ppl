#ifndef PPL_BOUNDING_SPHERES_COLLISION_DETECTION_TEST_H_
#define PPL_BOUNDING_SPHERES_COLLISION_DETECTION_TEST_H_

#include "../ValidityCheckerMethodTest.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

template <typename MPTraits>
class BoundingSpheresCollisionDetectionTest :  public BoundingSpheres,
                  virtual public CollisionDetectionValidity<MPTraits>,
                               public ValidityCheckerMethodTest<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    BoundingSpheresCollisionDetectionTest();

    BoundingSpheresCollisionDetectionTest(XMLNode& _node);

    ~BoundingSpheresCollisionDetectionTest();

    ///@}
    ///@name Interface
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    ///@}

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult IndividualCfgValidityTest() override;

    virtual TestResult GroupCfgValidityTest() override;

    ///@}

    //using AlwaysTrueValidity<MPTraits>::m_name;
    template<typename T, typename U> friend class MethodSet;

};

/*--------------------------- Construction ---------------------------*/

template<typename MPTraits>
BoundingSpheresCollisionDetectionTest<MPTraits>::
BoundingSpheresCollisionDetectionTest() : BoundingSpheres(), CollisionDetectionValidity<MPTraits>() {}

template<typename MPTraits>
BoundingSpheresCollisionDetectionTest<MPTraits>::
BoundingSpheresCollisionDetectionTest(XMLNode& _node) : ValidityCheckerMethod<MPTraits>(_node),
                                         BoundingSpheres(_node), CollisionDetectionValidity<MPTraits>(_node) {
}

template<typename MPTraits>
BoundingSpheresCollisionDetectionTest<MPTraits>::
~BoundingSpheresCollisionDetectionTest() {}

/*---------------------------- Interface -----------------------------*/

template <typename MPTraits>
bool
BoundingSpheresCollisionDetectionTest<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  return CollisionDetectionValidity<MPTraits>::IsValidImpl(_cfg,_cdInfo,_callName);
  //return true;
}

/*--------------------- Test Interface Functions ---------------------*/

template<typename MPTraits>
typename BoundingSpheresCollisionDetectionTest<MPTraits>::TestResult
BoundingSpheresCollisionDetectionTest<MPTraits>::
IndividualCfgValidityTest() {

  bool passed = true;
  std::string message = "";

  auto output = this->IndividualCfgValidity();

  // Make sure that output is of size 2:
  if (output.size()!= 2){

    passed = false;
    message = message + "\n\tRobot did not have a positional degree of"
              " freedom.\n";
  }

  // Make sure that first response is false.
  if (output[0].first){

    passed = false;
    message = message + "\n\tA cfg was incorrectly labeled "
              "valid.\n";
  }
 // Make sure that second response is false.
  if (output[1].first){

    passed = false;
    message = message + "\n\tA cfg was incorrectly labeled "
              "valid.\n";
  }

  if(passed) {
    message = "IndividualCfgValidity::PASSED!\n";
  }
  else {
    message = "IndividualCfgValidity::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

template<typename MPTraits>
typename BoundingSpheresCollisionDetectionTest<MPTraits>::TestResult
BoundingSpheresCollisionDetectionTest<MPTraits>::
GroupCfgValidityTest() {

  bool passed = true;
  std::string message = "";

  auto output = this->GroupCfgValidity();
  // Make sure that output is of size 2:
  if (output.size()!= 2){

    passed = false;
    message = message + "\n\tRobot did not have a positional degree of"
              " freedom.\n";
  }

  // Make sure that first response is true.
  if (!output[0].first){

    passed = false;
    message = message + "\n\tA group cfg was incorrectly labeled "
              "invalid.\n";
  }

  // Make sure that second response is false.
  if (output[1].first){

    passed = false;
    message = message + "\n\tA group cfg was incorrectly labeled "
              "valid.\n";
  }

  if(passed) {
    message = "GroupCfgValidity::PASSED!\n";
  }
  else {
    message = "GroupCfgValidity::FAILED :(\n" + message;
  }
  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/
#endif
