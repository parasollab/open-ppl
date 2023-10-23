#ifndef PPL_NBOX_TEST_H
#define PPL_NBOX_TEST_H

#include "Geometry/Shapes/NBox.h"   //src

#include <cmath>
#include <string>
#include <utility>
#include <cstddef>
#include <iostream>
#include <vector>
#include <functional>


class NBoxTest: public NBox {
    public:
        ///@name LocalTypes
        ///@{
            
        typedef std::pair<bool,std::string> TestResult;

        ///@}
        ///@name Construction
        ///@{

        explicit NBoxTest();

        virtual ~NBoxTest();

        ///@}

    public:

        ///@name Internal State
        ///@{

        std::vector<double> m_center;        ///< The center point of the box.
        std::vector<Range<double>> m_range;  ///< The range in each dimension.

        ///@}

        friend std::istream& operator>>(std::istream& _is, NBox& _box);

        TestResult TestContains();

        TestResult TestClearance();

        TestResult TestClearancePoint();

        TestResult TestSample();

        /// Helper for constructing an instance of the NBox. The NBox will have range (-i - 1, i +1)
        /// where i is the dimension index
        /// @param dimension the dimension of the NBox
        /// @return An instance of NBox 
        NBox* Construct(int dimension);

        /// Helper for printing out the test result messages
        void RunTestMessage(TestResult (*inputTestFunction)());
};

/*--------------------------- Construction ---------------------------*/

NBoxTest::NBoxTest() : NBox(1) {}
NBoxTest::~NBoxTest() noexcept = default;

/*--------------------------- Tests ---------------------------*/

typename NBoxTest::TestResult NBoxTest::TestContains() {
    int dimension = 3;
    NBox* testBox = Construct(dimension);

    bool passed = true;
    std::string message = "";

    // this point should be contained
    std::vector<double> validP;
    for (int i = 0; i < dimension; i++) {
        validP.push_back((i + 1) / 2);
    }

    // this point should not be contained
    std::vector<double> invalidP;
    for (int i = 0; i < dimension; i++) {
        invalidP.push_back(2.0 * (i + 1));
    }

    if (!testBox->Contains(validP)) {
        passed = false;
        std::cout << "\n\tNBox does not contain valid point." << std::endl;
    } 

    if (testBox->Contains(invalidP)) {
        passed = false;
        std::cout << "\n\tNBox contains invalid point." << std::endl;
    }

    message = "\tFINISHED TestContains";
    return std::make_pair(passed, message);
} 

typename NBoxTest::TestResult NBoxTest::TestClearance() {

    int dimension = 3;
    NBox* testBox = Construct(dimension);

    bool passed = true;
    std::string message = "";

    // the origin point, whose clearance should be equal to 1
    std::vector<double> testP;
    for (int i = 0; i < dimension; i++) {
        testP.push_back(0);
    }

    double expectedClearance = 1;
    double actualClearance = testBox -> Clearance(testP);

    if (expectedClearance != actualClearance) {
        passed = false;
        std::cout << "\n\tReturned incorrect clearance." << std::endl;
    }

    message = "\tFINISHED TestClearance";
    return std::make_pair(passed, message);
} 

typename NBoxTest::TestResult NBoxTest::TestClearancePoint() {

    int dimension = 3;
    NBox* testBox = Construct(dimension);

    bool passed = true;
    std::string message = "";

    // the origin point, whose clearance point should be equal to (-1, 0, 0, ...)
    std::vector<double> testP;
    for (int i = 1; i <= dimension; i++) {
        testP.push_back(0);
    }

    std::vector<double> expectedClearancePoint;
    expectedClearancePoint.push_back(1);
    for (int i = 1; i < dimension; i++) {
        expectedClearancePoint.push_back(0);
    }

    std::vector<double> actualClearancePoint = testBox -> ClearancePoint(testP);

    if (expectedClearancePoint != actualClearancePoint) {
        passed = false;
        std::cout << "\n\tReturned clearance point is incorrect." << std::endl;
    }

    message = "\tFINISHED TestClearancePoint";
    return std::make_pair(passed, message);
} 


typename NBoxTest::TestResult NBoxTest::TestSample() {

    int dimension = 3;
    NBox* testBox = Construct(dimension);

    bool passed = true;
    std::string message = "";

    // sample 100 random points and test if they lie within the box
    for (int i = 0; i < 100; i++) {
        std::vector<double> randomSamplePoint = testBox -> Sample();
        if (!testBox -> Contains(randomSamplePoint)) {
            passed = false;
            std::cout << "\n\tSampled point not in the box." << std::endl;
        }
    }

    message = "\tFINISHED TestSample";
    return std::make_pair(passed, message);
} 

/*---------------------------- Helpers --------------------------*/

NBox* NBoxTest::Construct(int dimension) {
    // define a valid n dimensional box centered at origin with range (i - 1, i + 1)
    NBox* testBox = new NBox(dimension);
  
    for (int i = 0; i < dimension; i++) {
        testBox->SetRange(i, -i - 1, i + 1);
    }

    return testBox;
}

void NBoxTest::RunTestMessage(TestResult (*inputTestFunction)()) {
    TestResult testResult = inputTestFunction();
    std::cout << "NBoxTestMessage: "<< testResult.first << std::endl << testResult.second;
}

#endif