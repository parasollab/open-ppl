#!/bin/bash
#
# Script to run the clang-tidy tool on STAPL files.
# Must be called from the STAPL root.
# This can be used on .cc files like so:
#
# $ find test/ -name "*.cc" -exec ./tools/code_quality/tidy.sh {} \;

set -euo pipefail

clangtidy="clang-tidy"

files=()
while [[ $# -gt 0 ]]
do
  key="$1"
  case $key in
      -t|--clang-tidy)
        clangtidy="$2"
        shift
        shift
      ;;
      *)    # unknown option
        files+=("$1")
        shift
      ;;
  esac
done


STAPL=$PWD
MPI_INCLUDE=/usr/include/mpich-x86_64
GCC_TOOLCHAIN_ROOT=/usr/local/gnu/gcc-4.9.3

$clangtidy "${files[@]}" \
  -checks=modernize-use-equals-default -fix \
  -header-filter="stapl/*" -- \
  -gcc-toolchain $GCC_TOOLCHAIN_ROOT -std=c++11 -D_STAPL \
  -DSTAPL__GNUC__=4 -DSTAPL__GNUC_MINOR__=9 -DSTAPL__GNUC_PATCHLEVEL__=3 \
  -I${STAPL}/tools/libstdc++/4.9.3 \
  -I${STAPL}/tools -I${STAPL} \
  -I${BOOST_ROOT}/include -DBOOST_RESULT_OF_USE_TR1_WITH_DECLTYPE_FALLBACK \
  -I${MPI_INCLUDE} \
  -I${GCC_TOOLCHAIN_ROOT}/include/c++/4.9.3/ \
  -I${GCC_TOOLCHAIN_ROOT}/include/c++/4.9.3/x86_64-unknown-linux-gnu/ \
  -I/usr/include
