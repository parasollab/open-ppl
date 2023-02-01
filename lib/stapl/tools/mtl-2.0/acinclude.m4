
dnl AC_MTL_TRY_COMPILE(INCLUDES, FUNCTION-BODY,
dnl             [ACTION-IF-FOUND [, ACTION-IF-NOT-FOUND]])
dnl 
dnl  Just like AC_TRY_COMPILE except it does not #include "confdefs.h"
dnl   which is not available in cygwin environment. Also
dnl   changed the file extension used for the temporary file.
dnl
AC_DEFUN(AC_MTL_TRY_COMPILE,
[cat > conftest.cpp <<EOF
ifelse(AC_LANG, [FORTRAN77],
[      program main
[$2]
      end],
[dnl This sometimes fails to find confdefs.h, for some reason.
dnl [#]line __oline__ "[$]0"
[#]line __oline__ "configure"
[$1]
int main() {
[$2]
; return 0; }
])EOF
ac_mtl_compile='${CXX} -c $CFLAGS conftest.cpp 1>&5'
if AC_TRY_EVAL(ac_mtl_compile); then
  ifelse([$3], , :, [rm -rf conftest*
  $3])
else
  echo "configure: failed program was:" >&AC_FD_CC
  cat conftest.cpp >&AC_FD_CC
ifelse([$4], , , [  rm -rf conftest*
  $4
])dnl
fi
rm -f conftest*])


AC_DEFUN(AC_PROG_CXX_MWERKS,
[AC_CACHE_CHECK(whether we are using Metrowerks Codewarrior C++, MWERKS_CXX,
[cat > conftest.c <<EOF
#ifdef __MWERKS__
  yes;
#endif
EOF
if AC_TRY_COMMAND(${CXX} -E conftest.c) | egrep yes >/dev/null 2>&1; then
  MWERKS_CXX=yes
  compiler=mwerks
else
  MWERKS_CXX=no
fi])])


AC_DEFUN(AC_PROG_CXX_MSVCPP,
[AC_CACHE_CHECK(whether we are using Visual C++, MSVCPP_CXX,
[cat > conftest.c <<EOF
#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(__ICL)
  yes;
#endif
EOF
if AC_TRY_COMMAND(${CXX} -E conftest.c) | egrep yes >/dev/null 2>&1; then
  MSVCPP_CXX=yes
  compiler=msvcpp
else
  MSVCPP_CXX=no
fi])])

AC_DEFUN(AC_PROG_CXX_SGICC,
[AC_CACHE_CHECK(whether we are using SGI MIPSpro C++, SGI_CXX,
[cat > conftest.c <<EOF
# if defined(__sgi) && !defined(__GNUC__)
  yes;
#endif
EOF
if AC_TRY_COMMAND(${CXX} -E conftest.c) | egrep yes >/dev/null 2>&1; then
  SGI_CXX=yes
  compiler=sgicc
else
  SGI_CXX=no
fi])])

AC_DEFUN(AC_PROG_CXX_SUNCC,
[AC_CACHE_CHECK(whether we are using Sun C++, SUN_CXX,
[cat > conftest.c <<EOF
# if defined(__SUNPRO_CC) 
  yes;
#endif
EOF
if AC_TRY_COMMAND(${CXX} -E conftest.c) | egrep yes >/dev/null 2>&1; then
  SUN_CXX=yes
  compiler=suncc
else
  SUN_CXX=no
fi])])


AC_DEFUN(AC_PROG_CXX_INTELCC,
[AC_CACHE_CHECK(whether we are using Intel C++, INTEL_CXX,
[cat > conftest.c <<EOF
# if defined(__ICC)
  yes;
#endif
EOF
if AC_TRY_COMMAND(${CXX} -E conftest.c) | egrep yes >/dev/null 2>&1; then
  INTEL_CXX=yes
  compiler=intelcc
else
  INTEL_CXX=no
fi])])

AC_DEFUN(AC_PROG_CXX_KAICC,
[AC_CACHE_CHECK(whether we are using KAI C++, KAI_CXX,
[cat > conftest.c <<EOF
# if defined(__KCC)
  yes;
#endif
EOF
if AC_TRY_COMMAND(${CXX} -E conftest.c) | egrep yes >/dev/null 2>&1; then
  KAI_CXX=yes
  compiler=kaicc
else
  KAI_CXX=no
fi])])


AC_DEFUN(AC_BZ_SET_COMPILER,
  [cxxwith=`echo $1 | sed -e 's/ /@/'`
   case "$cxxwith" in
     *:*@*)                 # Full initialization syntax
       CXX=`echo "$cxxwith" | sed  -n -e 's/.*:\(.*\)@.*/\1/p'`
       CXXFLAGS=`echo "$cxxwith" | sed  -n -e 's/.*:.*@\(.*\)/\1/p'`
     ;;
     *:*)                   # Simple initialization syntax
       CXX=`echo "$cxxwith" | sed  -n -e 's/.*:\(.*\)/\1/p'`
       CXXFLAGS=$3
     ;;
     *)                     # Default values
       CXX=$2
       CXXFLAGS=$3
     ;;
   esac])

dnl Determine a Fortran 77 compiler to use.  If `F77' is not already set
dnl in the environment, check for `g77', `f77' and `f2c', in that order.
dnl Set the output variable `F77' to the name of the compiler found.
dnl 
dnl If using `g77' (the GNU Fortran 77 compiler), then `AC_PROG_F77'
dnl will set the shell variable `G77' to `yes', and empty otherwise.  If
dnl the output variable `FFLAGS' was not already set in the environment,
dnl then set it to `-g -02' for `g77' (or `-O2' where `g77' does not
dnl accept `-g').  Otherwise, set `FFLAGS' to `-g' for all other Fortran
dnl 77 compilers.
dnl 
dnl AC_PROG_F77()
AC_DEFUN(AC_MTL_PROG_F77,
[AC_BEFORE([$0], [AC_PROG_CPP])dnl
if test -z "$F77"; then
  AC_CHECK_PROGS(F77, g77 f77 f2c)
    test -z "$F77" && AC_MSG_WARN([no acceptable Fortran 77 compiler found in \$PATH])
fi

AC_PROG_F77_WORKS
AC_PROG_F77_GNU

if test $ac_cv_prog_g77 = yes; then
  G77=yes
dnl Check whether -g works, even if FFLAGS is set, in case the package
dnl plays around with FFLAGS (such as to build both debugging and
dnl normal versions of a library), tasteless as that idea is.
  ac_test_FFLAGS="${FFLAGS+set}"
  ac_save_FFLAGS="$FFLAGS"
  FFLAGS=
  AC_PROG_F77_G
  if test "$ac_test_FFLAGS" = set; then
    FFLAGS="$ac_save_FFLAGS"
  elif test $ac_cv_prog_f77_g = yes; then
    FFLAGS="-g -O2"
  else
    FFLAGS="-O2"
  fi
else
  G77=
  test "${FFLAGS+set}" = set || FFLAGS="-g"
fi
])
