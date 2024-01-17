#include <catch2/catch_test_macros.hpp>

#include "Geometry/Shapes/NBoxTest.h"
#include "Geometry/Shapes/NSphereTest.h"
#include "Geometry/Shapes/NSphericalShellTest.h"

TEST_CASE( "Geometry TESTS", "[multi-file:2]" ) {
    typedef std::pair<bool,std::string> TestResult;

    SECTION("Geometry Tests: NBOX Tests") {
        NBoxTest* nboxt = new NBoxTest();

        std::cout << "\nGeometry TESTS: NBOX Tests" << std::endl;
        // NBox Contains
        TestResult containsTestResult = nboxt->TestContains();
        std::cout << containsTestResult.second << "\n";
        CHECK( containsTestResult.first == true);

        // NBox Clearance
        TestResult clearanceTestResult = nboxt->TestClearance();
        std::cout << clearanceTestResult.second << "\n";
        CHECK( clearanceTestResult.first == true );

        // NBox Clearance point
        TestResult clearancePointTestResult = nboxt->TestClearancePoint();
        std::cout << clearancePointTestResult.second << "\n";
        CHECK( clearancePointTestResult.first== true );

        // NBox Sample
        TestResult sampleTestResult = nboxt->TestSample();
        std::cout << sampleTestResult.second << "\n";
        CHECK( sampleTestResult.first == true );
    }

    SECTION("Geometry TESTS: NSphere Tests") {
        NSphereTest* nspheret = new NSphereTest();

        std::cout << "\nGeometry TESTS: NSphere Tests" << std::endl;
        // NSphere Contains
        TestResult containsTestResult = nspheret->TestContains();
        std::cout << containsTestResult.second << "\n";
        CHECK( containsTestResult.first == true);

        // NSphere Clearance
        TestResult clearanceTestResult = nspheret->TestClearance();
        std::cout << clearanceTestResult.second << "\n";
        CHECK( clearanceTestResult.first == true );

        // NSphere Clearance point
        TestResult clearancePointTestResult = nspheret->TestClearancePoint();
        std::cout << clearancePointTestResult.second << "\n";
        CHECK( clearancePointTestResult.first== true );

        // NSphere Sample
        TestResult sampleTestResult = nspheret->TestSample();
        std::cout << sampleTestResult.second << "\n";
        CHECK( sampleTestResult.first == true );
    }

    SECTION("Geometry TESTS: NSphericalShell Tests") {
        NSphericalShellTest* nsphericalshellt = new NSphericalShellTest();

        std::cout << "\nGeometry TESTS: NSphericalShell Tests" << std::endl;
        // NSphericalShell Contains
        TestResult containsTestResult = nsphericalshellt->TestContains();
        std::cout << containsTestResult.second << "\n";
        CHECK( containsTestResult.first == true);

        // NSphericalShell Clearance
        TestResult clearanceTestResult = nsphericalshellt->TestClearance();
        std::cout << clearanceTestResult.second << "\n";
        CHECK( clearanceTestResult.first == true );

        // NSphericalShell Clearance point
        TestResult clearancePointTestResult = nsphericalshellt->TestClearancePoint();
        std::cout << clearancePointTestResult.second << "\n";
        CHECK( clearancePointTestResult.first== true );

        // NSphericalShell Sample
        TestResult sampleTestResult = nsphericalshellt->TestSample();
        std::cout << sampleTestResult.second << "\n";
        CHECK( sampleTestResult.first == true );
    }   
}