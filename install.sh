# Under Construction

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install tzdata build-essential gdb python gperf libclang-dev gfortran \
        ninja-build pkg-config make wget curl zip unzip \
        software-properties-common git-all libtool autoconf-archive texinfo bison \
        libmpfr-dev libeigen3-dev libboost-all-dev libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev \
        libxft-dev libfontconfig1-dev libfreetype6-dev libx11-dev libx11-xcb-dev libxext-dev libxfixes-dev libxi-dev libxrender-dev \
        libxcb1-dev libxcb-glx0-dev libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync-dev \
        libxcb-xfixes0-dev libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev libxcb-util-dev libxcb-xinerama0-dev libxcb-xkb-dev \
        libxkbcommon-dev libxkbcommon-x11-dev libatspi2.0-dev libxrandr-dev libxcursor-dev \
        libxdamage-dev libxinerama1 libxinerama-dev libssl-dev doxygen graphviz
sudo apt remove --purge --auto-remove cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'
sudo apt update
sudo apt install cmake
cd /opt || exit
git clone https://github.com/Microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh

# checkout project
git clone https://gitlab.engr.illinois.edu/parasol-group/parasol/pmpl.git
git checkout release-stage-one

# build
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake -S /pmpl -B /pmpl/build
cmake --build build
