# Parasol Motion Planning Library

The PMPL library is a general code base for studying motion planning algorithms.
This file lists the package dependencies for PMPL and how to install them.
> Tested on Ubuntu 20.04

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
sudo apt-get install tzdata build-essential gdb python gperf libclang-dev gfortran \
        ninja-build pkg-config make wget curl zip unzip \
        software-properties-common git-all libtool autoconf-archive texinfo bison \
        libmpfr-dev libeigen3-dev libboost-all-dev libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev \
        libxft-dev libfontconfig1-dev libfreetype6-dev libx11-dev libx11-xcb-dev libxext-dev libxfixes-dev libxi-dev libxrender-dev \
        libxcb1-dev libxcb-glx0-dev libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync-dev \
        libxcb-xfixes0-dev libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev libxcb-util-dev libxcb-xinerama0-dev libxcb-xkb-dev \
        libxkbcommon-dev libxkbcommon-x11-dev libatspi2.0-dev libxrandr-dev libxcursor-dev \ 
        libxdamage-dev libxinerama1 libxinerama-dev libssl-dev doxygen graphviz
```

### Remove previous versions of cmake and install latest cmake
```bash
sudo apt remove --purge --auto-remove cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
```
For Ubuntu Focal Fossa (20.04)
```bash
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'   
```
For Ubuntu Bionic Beaver (18.04)
```bash
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
```

### Install latest cmake
```bash
sudo apt update
sudo apt install cmake
```

### Install vcpkg
Full instructions available at https://vcpkg.io/en/getting-started.html 

```bash
git clone https://github.com/Microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh
```

### Build
Run cmake, specify build type (e.g. Release or Debug), generator (e.g. Ninja), vcpkg installation path, source path, output path
```bash
/usr/bin/cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake -S /pmpl -B /pmpl/build
```

<!---
### CGAL Runtime Error
There is currently a bug in the CGAL library which causes a runtime assertion in pmpl.  In order to work around this, after cmake has been configured and vcpkg has downloaded the CGAL library, you will need to comment out lines 171 and 172 of  the file 
build/vcpkg_installed/x64-linux/include/CGAL/Interval_nt.h, which read as follows:
```
    CGAL_assertion_msg( (!is_valid(i)) || (!is_valid(s)) || (!(i>s)),
              "Variable used before being initialized (or CGAL bug)");
```
--->

## Docker
Alternatively a docker file is provided, with the above instructions pre-built.

### Install Docker
Follow the instructions on https://docs.docker.com/get-docker/ for your operating system

### Build Docker Container
In order to build in a docker container, first clone this repository to your Docker host system.  In the top level directory of the cloned repository, execute the following commands
```bash
docker build -t pmpl-build .
docker run -it pmpl-build 
```

The executable built resides in /pmp/build/pmpl_exec within the docker container

## Tests

**TODO** Update Test Instructions
