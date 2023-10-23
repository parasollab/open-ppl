#!/bin/bash

# This script builds the Planning Motion (PPL) library locally from gitLab repository in following steps:
# 1. It installs all required libraries.
# 2. It installs CMake, it removes the existing one firstPMP.
# 3. It installs vcpkg, a package management environment in local /dev directory.
# 4. This step is commented out. PPL source code can be cloned to /dev out. 
#       You might be asked to provide gitLab credentials. The default branch is release-stage-one
# 5. Finally it builds the executable files/targets in pmpl/build folder.

# install all required libraries for building PPL
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get install -y tzdata build-essential gdb python gperf libclang-dev gfortran \
        ninja-build pkg-config make wget curl zip unzip \
        software-properties-common git-all libtool autoconf-archive texinfo bison \
        libmpfr-dev libeigen3-dev libboost-all-dev libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev \
        libxft-dev libfontconfig1-dev libfreetype6-dev libx11-dev libx11-xcb-dev libxext-dev libxfixes-dev libxi-dev libxrender-dev \
        libxcb1-dev libxcb-glx0-dev libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync-dev \
        libxcb-xfixes0-dev libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev libxcb-util-dev libxcb-xinerama0-dev libxcb-xkb-dev \
        libxkbcommon-dev libxkbcommon-x11-dev libatspi2.0-dev libxrandr-dev libxcursor-dev \
        libxdamage-dev libxinerama1 libxinerama-dev libssl-dev doxygen graphviz

# install cmake, remove an existing one first
sudo apt remove --purge --auto-remove cmake -y
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'
sudo apt update
sudo apt install -y cmake

# install vcpkg in dev directory, remove an existing one first
cd ~/dev || exit
sudo rm -rf vcpkg ~/dev/vcpkg/ports/qt
git clone https://github.com/Microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh -disableMetrics

# checkout PPL project to dev
#cd ~/dev || exit
#cd pm	sudo apt-get update
#git clone https://gitlab.engr.illinois.edu/parasol-group/parasol/pmpl.git
#cd pmpl || exit
#git checkout release-stage-one

# build, assuming there is a dev/pmpl code repository
cd ~/dev || exit
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -DBUILD_TESTS=true -DCMAKE_TOOLCHAIN_FILE=~/dev/vcpkg/scripts/buildsystems/vcpkg.cmake -S pmpl -B pmpl/build
cmake --build pmpl/build