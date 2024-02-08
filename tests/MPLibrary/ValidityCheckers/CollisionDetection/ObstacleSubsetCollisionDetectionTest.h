#ifndef PPL_OBSTACLE_SUBSET_COLLISION_DETECTION_TEST_H_
#define PPL_OBSTACLE_SUBSET_COLLISION_DETECTION_TEST_H_

#include "CollisionDetectionMethodTest.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CollisionDetectionValidity.h"

class ObstacleSubsetCollisionDetectionTest :  public CollisionDetectionValidity,
                               public CollisionDetectionMethodTest {

  public:
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult      TestResult;

    ///@}

    ///@name Construction
    ///@{

    ObstacleSubsetCollisionDetectionTest();

    ObstacleSubsetCollisionDetectionTest(MPProblem* _problem);

    ~ObstacleSubsetCollisionDetectionTest();

    ///@}
  
  private:
    std::unique_ptr<Environment> m_test_env{}; //This method must be tested with a specific environment


};

/*--------------------------- Construction ---------------------------*/

ObstacleSubsetCollisionDetectionTest::
ObstacleSubsetCollisionDetectionTest() : CollisionDetectionValidity() {}

ObstacleSubsetCollisionDetectionTest::
ObstacleSubsetCollisionDetectionTest(MPProblem* _problem) : CollisionDetectionMethod(),
                                                             CollisionDetectionValidity(){
  m_MPProblem = _problem;
}

ObstacleSubsetCollisionDetectionTest::
~ObstacleSubsetCollisionDetectionTest() {}