# Getting started with STAPL

STAPL is a C++ parallel programming framework. 

## Prerequisites

STAPL is a library, requiring only a C++ compiler, the Boost libraries, and
established communication libraries such as MPI.

Compilers supported are:
- GCC 4.8.2 - 6.3.0
- Clang 3.6 - 3.7 (using libstdc++ for STL)
- Intel C++ 17 (using gcc compatability)

The version of Boost required is 1.63.

## Compiling the STAPL library

Compiling the STAPL library requires setting the platform and stl variables.
For example, on a Linux workstation with MPICH, Boost, and GCC 4.8.4 installed
the library can be built by running

```
gmake platform=LINUX_gcc stl=./tools/libstdc++/4.8.4
```

The result of this command will be `lib/libstapl.a` and `libstapl_rt.a`.


## Compiling STAPL programs

When compiling a STAPL program there are several flags required.

- `-D_STAPL`: used in some components that are used in other Parasol projects to
  detect a STAPL application is being compiled rather than a
  sequential application

- `-I $(STAPL_ROOT)/$(stl)`: required so that versions of the STL and Boost

- `-I $(STAPL_ROOT)/tools`:  headers with define_type methods (used in
   serialization) are available

- `-I $(STAPL_ROOT)/`: required so the stapl headers in stapl/ are
  available.  Include statements are of the form
  `#include <stapl/array.hpp>`

- `-I $(BOOST_ROOT)/include`: STAPL requires Boost 1.63`

- `-DBOOST_RESULT_OF_USE_TR1_WITH_DECLTYPE_FALLBACK`:
    Required by mechanism that detects work function return types

- `-std=c++11`: STAPL requires c++11 support

- `-L$(STAPL_ROOT)/lib -lstapl -lrt`: path to libstapl.a and the libraries
  required

- `-L$(BOOST_ROOT)/lib -lboost_serialization -lboost_thread -lboost_system`:
   Boost libraries required by STAPL runtime. These options must come after
   `-lstapl` in the compile line.

There are also several optional compile flags:

- `-DSTAPL_NDEBUG`: disable runtime assertions, which can perform expensive
  computations as part of their check.  This flag should be
  used when compiling executables for timing experiments.

- `-DSTAPL_PRODUCTION`: enable single core optimizations

- `-DSTAPL_PER_VIEW_LOCALIZATION`: enable independent localization of views
  passed to algorithms. This flag should be
  used when compiling executables for timing
  experiments.

- `-DSTAPL_RUNTIME_USE_OMP -fopenmp`: explicitly specify runtime used for
  mixed-mode


To see a full command line cd to examples/statistics and run `gmake mean_standard_deviation`. Note that in addition to the Boost libraries mentioned above, this test also requires the Boost program options library to link (i.e., `-lboost_program_options`).

Additional compiler options can be specified as part of the make command through the USER_CXXFLAGS and USER_LIB variables if the STAPL Makefiles are used.


## Running STAPL programs

STAPL programs execute in mixed-mode at all times.  To run MPI-only with a single thread per process you set `STAPL_NUM_THREADS` to 1.  If `STAPL_NUM_THREADS` is not set execution will terminate.


## Where to start

- [docs/html](docs/html): directory contains a build of the STAPL doxygen comments

- [examples](examples): directory used at the February 2015 CERT Deep Dive meeting on STAPL.
            PDFs of the slides are there. Each directory contains different
            types of simple examples.  All Project Euler examples include a
            report by the student that developed the example including
            pseudocode, design with STAPL components, and a scaling study using
            a Cray system in the TAMU CSE department.

- [docs/tutorial_guide](docs/tutorial_guide): tutorial_guide.pdf is a document written to walk the
                       reader through progressively more advanced uses of STAPL
                       components. The tutorial_guide/ directory includes the
                       source code for those examples.

- [benchmarks](benchmarks): Implementations of benchmarks using STAPL. The benchmarks have
              been collected from various sources. For example, the runtime OSU
              benchmarks are STAPL ARMI implementations of the OSU
              microbenchmarks for message passing.

Each code directory has a GNUmakefile in it that is setup to allow compilation
given that the directory hasn't been moved outside the working copy.


## Getting Help

The STAPL development community can be found online at https://gitlab.com/parasol-lab/stapl . If you need more direct support please send email to
stapl-support@tamu.edu.
