#include <catch2/catch_session.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

std::string test_sfile;

int main( int argc, char* argv[] ) {
    Catch::Session session;

    std::string filename;

    // Build a new command line parser on top of Catch2's
    using namespace Catch::Clara;
    auto cli = session.cli()           // Catch2's command line parser
              | Opt( filename, "Strategy Test input file" )
              ["-F"]["--FTest_file"] ("Strategy Test input file?");
    session.cli( cli );

    int returnCode = session.applyCommandLine( argc, argv );
    if( returnCode != 0 )
        return returnCode;

    if(filename.length() == 0) {
        test_sfile = "BasicTests";
    }
    else {
        test_sfile = filename;
    }

    return session.run();
}