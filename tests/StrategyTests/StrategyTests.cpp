#include <catch2/catch_test_macros.hpp>

#include <exception>
#include <limits>
#include <cstdlib>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <string>
#include <vector>
#include "MPProblem/MPProblem.h"
#include "MPLibrary/PMPL.h"

extern std::string test_sfile;

namespace fs = std::filesystem;

const std::string STRATEGY_TESTS = "../tests/StrategyTests/";
const std::string XML_DIR = "XMLs/";
const std::string OUTPUT_DIR = "Scratch/";

std::filesystem::path stratdir = fs::weakly_canonical(fs::absolute(STRATEGY_TESTS));
std::filesystem::path xmldir = fs::weakly_canonical(fs::absolute(STRATEGY_TESTS + XML_DIR));
std::filesystem::path output =  fs::weakly_canonical(fs::absolute(STRATEGY_TESTS + OUTPUT_DIR));


int RunXMLTest(const std::string& xml_file) {
    std::string xml_path{ xmldir.u8string() + "/" + xml_file};
    std::string pmpl_file{ output.u8string()  + "/" + xml_file + ".pmpl" };

    // Parse the Problem node into an MPProblem object.
    MPProblem* problem = new MPProblem(xml_path);

    // Parse the Library node into an MPLibrary object.
    MPLibrary* pmpl = new MPLibrary(xml_path);

    // Create storage for the solution and ask the library to solve our problem.
    Robot* const robot = problem->GetRobots().front().get();
    const auto robotTasks = problem->GetTasks(robot);
    for(auto task : robotTasks)
        if(!task->GetStatus().is_complete())
            pmpl->Solve(problem, task.get());

    // Also solve the group task(s).
    if(!problem->GetRobotGroups().empty()) {
        RobotGroup* const robotGroup = problem->GetRobotGroups().front().get();
        for(auto groupTask : problem->GetTasks(robotGroup))
            pmpl->Solve(problem, groupTask.get());
    }

    if(robotTasks.empty() and (problem->GetRobotGroups().empty() or
                               problem->GetTasks(problem->GetRobotGroups().front().get()).empty()))
        throw RunTimeException(WHERE) << "No tasks were specified!";

    // Release resources.
    delete problem;
    delete pmpl;

    return 0;
}


TEST_CASE("StrategyTest", "[test]") {

    if(!std::filesystem::exists(output)) {
        std::filesystem::create_directory(output);
    }

    std::ifstream testfile(stratdir.u8string() + "/" + test_sfile);
    std::cout << "Strategy file: " << (stratdir.u8string() + "/" + test_sfile) << "\n";

    if (testfile.is_open()) {
        std::string xmline;

        for (int j = 0; getline(testfile, xmline); j++) {
            // std::cout << xmline << "\n";

            // Delete any leftover files from the previous run.
            for (const auto &entry: std::filesystem::directory_iterator(output))
                std::filesystem::remove_all(entry.path());

            // Split the line (whitespace delimited only) into tokens.
            std::stringstream ss(xmline); // Turn the line string into a stream.
            std::istream_iterator<std::string> begin(ss);
            std::istream_iterator<std::string> end;
            std::vector<std::string> vstrings(begin, end);
            std::cout << "...Running test for " << vstrings[0] << "..." << std::endl;

            if (vstrings.empty()) {
                REQUIRE_FALSE(vstrings.empty());
            }

            // Execute the XML file.
            std::string xml_file = vstrings[0];

            int result = -1;
            try {
                result = RunXMLTest(xml_file);
            } catch(...) {
                result = -1;
            }

            // Check for aborted run.
            if (result != 0) {
                std::cout << "Error: test " << xml_file << "failed to run.\n";
                CHECK( result == 0 );

                continue;
            }
            // Check for errors (i.e. an output file was generated).
            for (int k = 1; k < vstrings.size(); k++) {
                std::string output_path = STRATEGY_TESTS + OUTPUT_DIR + vstrings[k];
                std::ifstream infile(output_path);

                // No file = no problem = success.
                if(!infile.good()) {
                    //std::cout << "Success: test " << xml_file << " on output " << vstrings[k] << "\n";
                    continue;
                }
                std::cout << "'Error: test " << xml_file << " failed to match correct output on " << vstrings[k] << "\n";
                CHECK_FALSE( !infile.good() );
            }
            SUCCEED();
        }
    } else {
        std::cout << "Error: cannot find test file " << (stratdir.u8string() + "/" + test_sfile) << "\n";
        CHECK(testfile.is_open());
    }
}
