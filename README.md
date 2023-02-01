# Parasol Motion Planning Library

The PMPL library is a general code base for studying motion planning algorithms.
This file lists the package dependencies for PMPL and how to install them.
> Tested on Ubuntu 20.04

## Requirements
Required Packages:
- build-essential
- make
- cmake

It is recommended to update programs on your system before continuing. However,
this can sometimes break certain programs that require a specific package
version, such as a graphics driver and CUDA library.

To update your system, run the following commands:
```
sudo apt-get update
sudo apt-get upgrade
```

To install the required packages, run the following commands:
```
sudo apt-get update
sudo apt-get install build-essential make cmake libmpfr-dev
```

**TODO** QT6 Step

<!--
## Migration from SVN

As part of the migration from SVN, we have separated our examples into their own repository at [pmpl\_envs](https://gitlab.engr.illinois.edu/parasol/envs.git). If you find that an input file you need is missing, please import it from the archived SVN repo to the new envs repo or ask Read for help.

Another important change is the removal of SVN externals. Our utilities now live at [pmpl\_utils](https://gitlab.engr.illinois.edu/parasol/pmpl_utils.git), and must be cloned separately. We will likely move to either git submodules or subtress eventually, but for now here is how to set up your utilities:
- Clone the utilities repo, which will produce a directory called `pmpl_utils`.
- Go to the root of your PMPL checkout and create a soft-link to the `pmpl_utils` directory with `ln -s /my/path/to/pmpl_utils`.
- Make PMPL.
Note that you generally only need one utilities checkout; multiple working copies of PMPL can and should share the same utilities folder.

If you are working on a branch other than trunk, you may have issues with the utilities versions not matching your branch state. If this occurs, please try to update your branch to use the latest utilities. This should be straight-forward, but if it proves difficult please ask Read for help.

In addition to the dedicated utilities, PMPL requires several other libraries:
- gcc
- boost
- bash (for testing script)
- CGAL v4.6 - 4.11
- OpenCV (for marker detection only)
- Qt6 (for simulator only)

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

-->
## Build

**TODO** Update Build Instructions

### CGAL Runtime Error
There is currently a bug in the CGAL library which causes a runtime assertion in pmpl.  In order to work around this, after cmake has been configured and vcpkg has downloaded the CGAL library, you will need to comment out lines 171 and 172 of  the file 
build/vcpkg_installed/x64-linux/include/CGAL/Interval_nt.h, which read as follows:
```
    CGAL_assertion_msg( (!is_valid(i)) || (!is_valid(s)) || (!(i>s)),
              "Variable used before being initialized (or CGAL bug)");
```

### Docker
In order to build in a docker container, first clone this repository to your Docker host system.  In the top level directory of the cloned repository, execute the following commands
```
    docker build -t pmpl-build .
    docker run -it pmpl-build 
```

The executable built resides in /pmp/build/pmpl_exec

## Tests

**TODO** Update Test Instructions
