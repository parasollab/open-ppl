FROM ubuntu:20.04

#Installs prerequisites for pmpl
RUN  apt-get update \
    && apt-get -y upgrade \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        tzdata \
        build-essential gdb \
        python3 python3-pip \
        gperf libclang-dev \
        gfortran \
        ninja-build \
        pkg-config \
        make wget curl zip unzip \
        software-properties-common \
        git-all \
        libtool \
        autoconf-archive \
        texinfo \
        bison \
        libmpfr-dev libfontenc-dev libxaw7-dev\
        libeigen3-dev \
        libboost-all-dev \
        libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev \
        libxft-dev libfontconfig1-dev libfreetype6-dev libx11-dev libx11-xcb-dev libxext-dev libxfixes-dev libxi-dev libxrender-dev \
        libxcb1-dev libxcb-glx0-dev libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync-dev \
        libxcb-xfixes0-dev libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev libxcb-util-dev libxcb-xinerama0-dev libxcb-xkb-dev libxkbcommon-dev libxkbcommon-x11-dev libatspi2.0-dev \
        libxrandr-dev libxcursor-dev libxdamage-dev libxinerama1 libxinerama-dev \
        libssl-dev \
        doxygen graphviz lcov gcovr
        
# install latest cmake > 3.24
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \ 
    && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null
RUN apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'
RUN apt update
RUN apt-get -y install cmake

#install conan
RUN pip install conan

#Copy source code from host system
COPY .. /pmpl

WORKDIR /pmpl
RUN conan profile detect
RUN conan install . --output-folder=build_conan --profile:build=default -c tools.system.package_manager:mode=install -s compiler.cppstd=gnu17 -s build_type=Debug --build missing

#Build pmpl executable
RUN cmake -B build_conan -S . -G  Ninja -DBUILD_TESTS=true -DCMAKE_TOOLCHAIN_FILE=build_conan/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug
RUN cmake --build build_conan
