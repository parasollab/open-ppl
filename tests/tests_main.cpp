#include <catch2/catch_session.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <filesystem>

std::string test_ufile;

namespace fs = std::filesystem;

const std::filesystem::path xmlUnitDir = fs::weakly_canonical(fs::absolute("../tests/"));

int main( int argc, char* argv[] ) {
    Catch::Session session;

    std::string filename;

    // Build a new command line parser on top of Catch2's
    using namespace Catch::Clara;
    auto cli = session.cli()           // Catch2's command line parser
              | Opt( filename, "Unit Test input XML file" )
              ["-U"]["--UTest_file"] ("Unit Test input XML file?");
    session.cli( cli );

    int returnCode = session.applyCommandLine( argc, argv );
    if( returnCode != 0 )       // Indicates a cli session and command line error
        return returnCode;

    if(filename.length() == 0) {
        test_ufile = xmlUnitDir.u8string() + "/FullUnitTest.xml";
    }
    else {
        std::string::size_type idx;
        idx = filename.rfind('.');

        if ( idx != std::string::npos && filename.substr(idx+1) == "xml" ) {
            test_ufile = filename;
        }
    }

    return session.run();
}
