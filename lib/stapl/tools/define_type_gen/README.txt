define_type_gen DOCUMENTATION
----------------------------------------------------

define_type_gen is tool designed to add the define_type functions necessary for serialization in
stapl code. It runs using LLVM's clang tooling library.

----------------------------------------------------

1. Requirements:
-----------------

    - LLVM/Clang (tested with version 5.0.0)

    - cmake (version 3.4.3 or later)

2. How to compile
------------------

    - navigate to the directory in which you would like the tool's binary to be. A build directory
        within the define_type_gen directory is recommended.

    - run "cmake /{path to STAPL root}/tools/define_type_gen/source"

    - If cmake cannot find clang or LLVM, specify the directory containing 'ClangConfig.cmake'
        using '-DClang_DIR'. This can usually be found in '/{path to LLVM build root}/lib/cmake/clang'.
        In other words, you would run:
        "cmake /{path to STAPL root}/tools/define_type_gen/source -DClang_DIR="/{path to LLVM build root}/lib/cmake/clang"

    - After cmake runs successfully, the program can be compiled with the command "make".

3. How to run
--------------

    - WARNING: This program defaults to editing input files in place. Make sure you back up your project
        before running the program in case of errors.

    - define_type_gen expects code that has no compilation errors other than those related to missing define_type methods

    - the program can be run with the following:
        "./define_type_gen {INPUT FILES} {DEFINE_TYPE_GEN COMPILER OPTIONS}  --
        -I/{path to LLVM build directory}/bin/../lib/clang/5.0.0/include {NORMAL COMPILER OPTIONS}"

        - INPUT FILES: source files on which to run
            define_type_gen will automatically traverse and edit included headers
    
        - DEFINE_TYPE_GEN COMPILER OPTIONS: see './define_type_gen --help' for more information
            The following flags are the most important:
                -dir=/some/directory - specify a project directory. Only files within this directory and it's
                    subdirectories will be edited
                -newfile - write changed files to a new file instead of editing in place
                -regen - regenerate the define_type function if it is found in the class
                -ignore-classes=Comma,Separated,Qualified,Class,Names - specify a list of classes for the program
                    to ignore.
    
        - NORMAL COMPILER OPTIONS: use the options you intend to pass to the compiler when you compile your project
        
    -Note: since the tool runs with clang, STAPL needs to be configured to work with clang. This likely involves using
    "-DSTAPL__GNUC__=4 -DSTAPL__GNUC_MINOR__=8 -DSTAPL__GNUC_PATCHLEVEL=5" in NORMAL COMPILER OPTIONS.
        
    - Example run command:
        "./define_type_gen test1.cpp test2.cpp -regen -classes=MyNamespace::MyClass,MyClass2,MyNamespace::MyClass3 -dir=/path/to/project
        -- -I/path/to/llvm/build/bin/../lib/clang/5.0.0/include" -x c++ -D_STAPL -I/path/to/stapl/trunk/./tools/libstdc++/4.8.5
        -I/path/to/stapl/trunk/tools -I/path/to/stapl/trunk -Wall -Wno-unused-local-typedefs -Wno-unknown-pragmas -Wno-deprecated-declarations
        -std=c++11 -I/usr/local/boost/boost-1.63/include -DBOOST_RESULT_OF_USE_TR1_WITH_DECLTYPE_FALLBACK -I/scratch/mashley310/OPENMPI/build/include
        -DSTAPL__GNUC__=4 -DSTAPL__GNUC_MINOR__=8 -DSTAPL__GNUC_PATCHLEVEL=5"

4. Known issues
----------------

    - None
    
