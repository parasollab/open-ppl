conan install . --output-folder=build --build=missing -c tools.system.package_manager:mode=install -c tools.system.package_manager:sudo=true -s build_type=Debug -s compiler.cppstd=gnu17
cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -B build/
cmake --build build -j6
