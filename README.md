# Parasol Motion Planning Library

The PMPL library is a general code base for studying motion planning algorithms.

## Migration from SVN

As part of the migration from SVN, we have separated our examples into their own repository at [pmpl\_envs](https://gitlab.engr.illinois.edu/parasol/envs.git). If you find that an input file you need is missing, please import it from the archived SVN repo to the new envs repo or ask Read for help.

Another important change is the removal of SVN externals. Our utilities now live at [pmpl\_utils](https://gitlab.engr.illinois.edu/parasol/pmpl_utils.git), and must be cloned separately. We will likely move to either git submodules or subtress eventually, but for now here is how to set up your utilities:
- Clone the utilities repo, which will produce a directory called `pmpl_utils`.
- Go to the root of your PMPL checkout and create a soft-link to the `pmpl_utils` directory with `ln -s /my/path/to/pmpl_utils`.
- Make PMPL.
Note that you generally only need one utilities checkout; multiple working copies of PMPL can and should share the same utilities folder.

If you are working on a branch other than trunk, you may have issues with the utilities versions not matching your branch state. If this occurs, please try to update your branch to use the latest utilities. This should be straight-forward, but if it proves difficult please ask Read for help.

## Requirements

In addition to the dedicated utilities, PMPL requires several other libraries:
- gcc
- boost
- bash (for testing script)
- CGAL v4.6 - 4.11
- OpenCV (for marker detection only)
- Qt4 (for simulator only)

**TODO**: Determine supported versions for each utility.

**TODO**: List libraries from the dedicated utilities to facilitate converting to pulling from their home repos.
- aruco
- bullet v2.87
- dlib
- gl\_visualizer (nonstd, glutils, sandbox)
- MPNN
- player
- PQP
- RAPID
- stapl
- tetgen
- tinyxml

## Build

To build the traditional PMPL executable, go to `src` and run `make pmpl`.

To build the simulator, go to `src` and run `make sim`.

## Usage

**TODO**: Document the various ways to invoke the program.

## Tests

To run the tests, go to `src/Test` and run `tests.sh`.
