# Parasol Motion Planning Library

The PMPL library is a general code base for studying motion planning algorithms.
This file lists the package dependencies for PMPL and how to install them.
> Tested on Ubuntu 20.04

## Clone repository
```bash
git clone https://gitlab.engr.illinois.edu/parasol-group/parasol/pmpl.git
```

## Requirements
### Required Tools:
- vcpkg

### Required Packages:
- build-essential
- make
- cmake
- ninja-build
- doxygen

It is recommended to update programs on your system before continuing. However,
this can sometimes break certain programs that require a specific package
version, such as a graphics driver and CUDA library.

### Required External Dependencies
- opencv _(note: can be removed but build image needs updating)_
- eigen3
- cgal
- bullet3
- boost
- nlohmann-json
- qtbase
- qttools
- catch2
- tinyxml2

### To update your system, run the following commands:
```bash
sudo apt-get update
```
```bash
sudo apt-get upgrade
```


### To install the required packages, run the following commands:
```bash
sudo apt-get update
```
```bash
sudo apt-get install tzdata build-essential gdb python3 gperf libclang-dev gfortran \
        ninja-build pkg-config make wget curl zip unzip \
        software-properties-common git-all libtool autoconf-archive texinfo bison \
        libmpfr-dev libeigen3-dev libboost-all-dev libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev \
        libxft-dev libfontconfig1-dev libfreetype6-dev libx11-dev libx11-xcb-dev libxext-dev libxfixes-dev libxi-dev libxrender-dev \
        libxcb1-dev libxcb-glx0-dev libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync-dev \
        libxcb-xfixes0-dev libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev libxcb-util-dev libxcb-xinerama0-dev libxcb-xkb-dev \
        libxkbcommon-dev libxkbcommon-x11-dev libatspi2.0-dev libxrandr-dev libxcursor-dev \ 
        libxdamage-dev libxinerama1 libxinerama-dev libxcomposite-dev libxkbfile-dev libxmuu-dev libxres-dev \
        libxcb-dri3-dev libxcb-cursor-dev libssl-dev doxygen graphviz
```

### Remove previous versions of cmake and install latest cmake
```bash
sudo apt remove -y --purge --auto-remove cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
```

For Ubuntu Jamming Jellyfish (22.04)
```bash
sudo apt-add-repository -y 'deb https://apt.kitware.com/ubuntu/ jammy main'
```
For Ubuntu Focal Fossa (20.04)
```bash
sudo apt-add-repository -y 'deb https://apt.kitware.com/ubuntu/ focal main'   
```
For Ubuntu Bionic Beaver (18.04)
```bash
sudo apt-add-repository -y 'deb https://apt.kitware.com/ubuntu/ bionic main'
```

### Install latest cmake
```bash
sudo apt update
sudo apt install cmake
```



### Install and build using vcpkg

#### Install vcpkg
Full instructions available at https://vcpkg.io/en/getting-started.html

```bash
cd /opt
```
```bash
sudo git clone https://github.com/Microsoft/vcpkg.git
```
```bash
sudo chmod 777 /opt/vcpkg && sudo chmod 666 vcpkg/.vcpkg-root
```
```bash
sudo ./vcpkg/bootstrap-vcpkg.sh
```

#### Build pmpl with vcpkg
Run cmake, specify build type (e.g. Release or Debug), generator (e.g. Ninja), vcpkg installation path, source path, output path
cd into the cloned repo
```bash
cd pmpl
```
```bash
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake -S . -B build
```

Build PPL with however many cores you'd prefer using `-j(number)`. By default, all cores will be used, which can potentially overwhelm your computer.
```bash
cmake --build build -j3
```

### Install and build using Conan
#### Install conan 
Alternate installation instructions available at https://docs.conan.io/en/latest/installation.html
```bash
pip install conan
```

Make sure that the installed version of conan is > 2.0
```bash
conan --version
```

Setup default conan profile for machine.
```bash
conan profile detect
```

#### Install conan packages
For Ubuntu 22.04, qt/6.3.2 requires this export for C3I until conan-io/conan-center-index#13472 is fixed
```bash
export NOT_ON_C3I=1
```
```bash
conan install . --output-folder=build --build=missing -c tools.system.package_manager:mode=install -c tools.system.package_manager:sudo=true -s build_type=Debug -s compiler.cppstd=gnu17
```

* make sure to set tools.system.package_manager:sudo=**false** if using docker

#### Build ppl with conan
```bash
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -B build/
```
Build PPL with however many cores you'd prefer using `-j(number)`. By default, all cores will be used, which can potentially overwhelm your computer.
```bash
cmake --build build -j3
```



## Docker
Alternatively a default docker file is provided, with the above instructions pre-built.
More docker configurations and instructions are available [here](docker/README.md).

### Install Docker
Follow the instructions on https://docs.docker.com/get-docker/ for your operating system

### Build Docker Container
In order to build in a docker container, first clone this repository to your Docker host system.  In the top level directory of the cloned repository, execute the following commands
```bash
docker build -t pmpl-build .
docker run -it pmpl-build 
```

The executable built resides in /pmpl/build/pmpl_exec within the docker container

## Tests

### To run tests start by building the test target
```bash
cmake --build build --target tests
```

### You can run the by running CTest
```bash
cd build
ctest --output-on-failure
```

this will run the Basic PRM Tests by default

### Or by running the generated test executable
```bash
cd build
./ppl_tests -F CfgTests
```

## Consuming PPL Libraries

### Install from source
To install the PPL libraries and related headers, run the following in this directory.

```bash
cmake --build build --target install
```

### Linking
In order to have access to the dependencies of PPL, for things like header files included in PPL's header files, you should include the following commands in the CMakeLists file for your ptroject.
```cmake
find_package(Qt6 CONFIG COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Bullet CONFIG REQUIRED
         LinearMath Bullet3Common BulletDynamics BulletSoftBody BulletCollision BulletInverseDynamics)
find_package(modelloader CONFIG REQUIRED)
find_package(RAPID CONFIG REQUIRED)
find_package(Tetgen CONFIG REQUIRED)
find_package(PQP CONFIG REQUIRED)
find_package(gl_visualizer CONFIG COMPONENTS nonstd glutils sandbox REQUIRED)
find_package(PPL CONFIG REQUIRED)
```

Then you will need to link to either ppl::ppl_mp_library or ppl::ppl_library

## binary caching of dependencies
Libraries installed with vcpkg can always be built from source. However, this can duplicate work and waste time across multiple developers or machines.

Binary caching saves copies of library binaries in a shared location that can be accessed by vcpkg for future installs. Caches can be hosted in a variety of environments. The most basic examples are a folder on the local machine or a network file share. Caches can also be stored in any NuGet feed (such as GitHub Packages or Azure DevOps Artifacts), Azure Blob Storage, Google Cloud Storage, and many other services.

https://learn.microsoft.com/en-us/vcpkg/users/binarycaching

## Documentation
We use `doxygen` for documentation, which is automatically generated when you build PPL. To view it, open `docs/Doxygen/html/index.html`

## Running Examples
We provide `Examples/CfgExamples.xml` as an example of how to create an xml file to run a motion planning scenaio. Run the following command to use it:
```bash
./build/ppl_mp -f Examples/CfgExamples.xml
```
