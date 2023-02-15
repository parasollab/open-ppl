#!/bin/bash
# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

# output directory
BUILDDIR="build"

# directories to be copied intact
REQUIRED=("stapl" "tools")

# debug flags
DEBUG_FLAGS="-O0 -g -std=c++11"

# release flags
RELEASE_FLAGS="-O3 -DSTAPL_NDEBUG -std=c++11"

# OpenMP related flags
OMP_FLAGS="-fopenmp -DSTAPL_RUNTIME_USE_OMP"

# make parallelism
num_makes=1

# error handling
# from http://stackoverflow.com/questions/64786/error-handling-in-bash
tempfiles=()
function cleanup()
{
  rm -rf "${tempfiles[@]}"
}
trap cleanup 0

function error()
{
  local parent_lineno="$1"
  local message="$2"
  local code="${3:-1}"
  if [[ -n "$message" ]] ; then
    echo -e "\nLine ${parent_lineno}: ${message}; exiting with status ${code}"
  else
    echo -e "\nLine ${parent_lineno}; exiting with status ${code}"
  fi
  exit "${code}"
}
trap 'error ${LINENO}' ERR

# command line argument parsing
args=$(getopt -l "help" -l "openmp" -l "CC:" -l "AR:" -l "FLAGS:" -l "j:" -o "h" -- "$@")
eval set -- "$args"
while [ $# -ge 1 ]; do
  case "$1" in
    --)
      # No more options left.
      shift
      break
      ;;
    --openmp)
      compile_omp=true
      shift
      ;;
    --CC)
      compiler="$2"
      shift
      ;;
    --FLAGS)
      flags="$2"
      shift
      ;;
    --AR)
      archiver="$2"
      shift
      ;;
    --j)
      num_makes="$2"
      shift
      ;;
    -h|--help)
      echo "STAPL compilation script"
      echo "  Available options:"
      echo "      --openmp Produce OpenMP enabled mixed-mode STAPL libraries"
      echo "      --CC     Provide compiler (default: \"mpic++\")"
      echo "      --FLAGS  Provide additional flags"
      echo "      --AR     Provide archiver along with any options if needed (default: \"ar cr\")"
      echo "      --j      Number of make instances"
      echo "  -h, --help   Display this message and exit"
      exit 0
      ;;
    esac
      shift
done

# override compiler
if [[ "x$compiler" != "x" ]] ; then
  CC_USER="${compiler}"
  export CC_USER
else
  if [[ $(type -P "mpic++") ]] ; then
    echo "Found mpic++ in PATH."
  else
    error ${LINENO} "mpic++ not found. Please use --CC to specify the mpic++ binary or update the PATH environment variable." 1
  fi
fi

# override archiver
if [[ "x$archiver" != "x" ]] ; then
  AR_USER="${archiver}"
  export AR_USER
fi

# check if BOOST_ROOT has been defined
if [[ "x${BOOST_ROOT}" = "x" ]] ; then
  error ${LINENO} "Please set BOOST_ROOT to point to the directory that Boost 1.53 is installed." 1
fi

# set Boost_LIBRARY_DIRS
if [[ "x${Boost_LIBRARY_DIRS}" = "x" ]] ; then
  Boost_LIBRARY_DIRS=`find ${BOOST_ROOT} -depth -name 'lib*' -type d | tail -1`
  echo "Boost_LIBRARY_DIRS set to ${Boost_LIBRARY_DIRS}"
  export Boost_LIBRARY_DIRS
fi

# OpenMP related libraries
if [[ "x${compile_omp}" != "x" ]] ; then
  OMP_LIBS=`ls ${Boost_LIBRARY_DIRS} | grep -m 1 -e boost_thread | sed 's/lib//g' | sed 's/\.so//g' | sed 's/\.a//g'`
fi

# find which libstd we are using
if [[ "x${STAPL_STL_DIR}" = "x" ]] ; then
  # check if gcc version was given
  if [[ "x${GCC_VERSION}" = "x" ]] ; then
    GCC_VERSION=`gcc -dumpversion`
    # In some gcc installation, dumpversion is not working correctly and gives
    # only major and minor version. Use this line to attempt to fix that.
    #GCC_VERSION=`gcc --version | grep ^gcc | sed 's/^.* //g'`
    if [[ "x${GCC_VERSION}" = "x" ]] ; then
      error ${LINENO} "Could not automatically find gcc version. Either define one with GCC_VERSION or set STAPL_STL_DIR to the directory in tools/ that contains the headers for the compiler that will be used to compile STAPL." 1
    fi
    STAPL_STL_DIR="./tools/libstdc++/${GCC_VERSION}"
  fi
fi

# check if the required STL directory exists
if [ ! -d ${STAPL_STL_DIR} ] ; then
  error ${LINENO} "${STAPL_STL_DIR} could not be found" 1
fi

# detect autoflags and autolibs
AUTOFLAGS=`make platform=generic stl="./tools/libstdc++/${GCC_VERSION}" showconf | grep AUTOFLAGS`
while IFS='=' read -ra TMP; do
  AUTOFLAGS=${TMP[1]}
done <<< "$AUTOFLAGS"

AUTOLIB=`make platform=generic stl="./tools/libstdc++/${GCC_VERSION}" showconf | grep AUTOLIB`
while IFS='=' read -ra TMP; do
  AUTOLIB=${TMP[1]}
done <<< "$AUTOLIB"

# create output directory
printf "Creating \"${BUILDDIR}\" and copying headers..."
rm -rf "${BUILDDIR}"
mkdir "${BUILDDIR}"
mkdir "${BUILDDIR}/bin"
mkdir "${BUILDDIR}/lib"
# copy required directories
cp -R "${REQUIRED[@]}" "${BUILDDIR}"
printf "done!\n"

# build debug library
`make clean &> /dev/null`
printf "Building debug library...\n"
make platform=generic stl="./tools/libstdc++/${GCC_VERSION}" USER_CXXFLAGS="${flags} ${DEBUG_FLAGS}" -j"${num_makes}"
cp lib/libstapl.a "${BUILDDIR}/lib/libstapl_debug.a"
printf "Building debug library... done!\n"

# build release library
`make clean &> /dev/null`
printf "Building release library...\n"
make platform=generic stl="./tools/libstdc++/${GCC_VERSION}" USER_CXXFLAGS="${flags} ${RELEASE_FLAGS}" -j"${num_makes}"
cp lib/libstapl.a "${BUILDDIR}/lib/libstapl.a"
printf "Building release library... done!\n"

# variables script
vars_script="staplvars.sh"
cp "tools/build/staplvars.sh" "${BUILDDIR}/bin/${vars_script}"
`sed -i "s@%ADDITIONAL_FLAGS%@${AUTOFLAGS}@g" ${BUILDDIR}/bin/${vars_script}`
`sed -i "s@%ADDITIONAL_LIBS%@${AUTOLIB}@g" ${BUILDDIR}/bin/${vars_script}`
`sed -i "s@%LIB_SUFFIX%@@g" ${BUILDDIR}/bin/${vars_script}`

# build openmp-backed libraries
if [[ "x$compile_omp" != "x" ]] ; then
  printf "Compiling with OpenMP. Assuming that the compiler accepts \"-fopenmp\".\n"

  TMP_DEBUG_FLAGS="${flags} ${DEBUG_FLAGS} ${OMP_FLAGS}"
  TMP_RELEASE_FLAGS="${flags} ${RELEASE_FLAGS} ${OMP_FLAGS}"

  # build debug version
  `make clean &> /dev/null`
  make platform=generic stl="./tools/libstdc++/${GCC_VERSION}" USER_CXXFLAGS="${TMP_DEBUG_FLAGS}" -j"${num_makes}"
  cp lib/libstapl.a "${BUILDDIR}/lib/libstapl_omp_debug.a"

  # build release version
  `make clean &> /dev/null`
  make platform=generic stl=./tools/libstdc++/${GCC_VERSION} USER_CXXFLAGS="${TMP_RELEASE_FLAGS}" -j"${num_makes}"
  cp lib/libstapl.a "${BUILDDIR}/lib/libstapl_omp.a"

  # variables script
  vars_script="staplvars_omp.sh"
  cp "tools/build/staplvars.sh" "${BUILDDIR}/bin/${vars_script}"
  `sed -i "s@%ADDITIONAL_FLAGS%@${AUTOFLAGS}@g" ${BUILDDIR}/bin/${vars_script}`
  `sed -i "s@%ADDITIONAL_LIBS%@${AUTOLIB}@g" ${BUILDDIR}/bin/${vars_script}`
  `sed -i "s@%LIB_SUFFIX%@_omp@g" ${BUILDDIR}/bin/${vars_script}`
fi
