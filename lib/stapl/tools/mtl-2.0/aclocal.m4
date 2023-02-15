dnl aclocal.m4 generated automatically by aclocal 1.4-p5

dnl Copyright (C) 1994, 1995-8, 1999, 2001 Free Software Foundation, Inc.
dnl This file is free software; the Free Software Foundation
dnl gives unlimited permission to copy and/or distribute it,
dnl with or without modifications, as long as this notice is preserved.

dnl This program is distributed in the hope that it will be useful,
dnl but WITHOUT ANY WARRANTY, to the extent permitted by law; without
dnl even the implied warranty of MERCHANTABILITY or FITNESS FOR A
dnl PARTICULAR PURPOSE.


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

# Do all the work for Automake.  This macro actually does too much --
# some checks are only needed if your package does certain things.
# But this isn't really a big deal.

# serial 1

dnl Usage:
dnl AM_INIT_AUTOMAKE(package,version, [no-define])

AC_DEFUN([AM_INIT_AUTOMAKE],
[AC_REQUIRE([AC_PROG_INSTALL])
PACKAGE=[$1]
AC_SUBST(PACKAGE)
VERSION=[$2]
AC_SUBST(VERSION)
dnl test to see if srcdir already configured
if test "`cd $srcdir && pwd`" != "`pwd`" && test -f $srcdir/config.status; then
  AC_MSG_ERROR([source directory already configured; run "make distclean" there first])
fi
ifelse([$3],,
AC_DEFINE_UNQUOTED(PACKAGE, "$PACKAGE", [Name of package])
AC_DEFINE_UNQUOTED(VERSION, "$VERSION", [Version number of package]))
AC_REQUIRE([AM_SANITY_CHECK])
AC_REQUIRE([AC_ARG_PROGRAM])
dnl FIXME This is truly gross.
missing_dir=`cd $ac_aux_dir && pwd`
AM_MISSING_PROG(ACLOCAL, aclocal, $missing_dir)
AM_MISSING_PROG(AUTOCONF, autoconf, $missing_dir)
AM_MISSING_PROG(AUTOMAKE, automake, $missing_dir)
AM_MISSING_PROG(AUTOHEADER, autoheader, $missing_dir)
AM_MISSING_PROG(MAKEINFO, makeinfo, $missing_dir)
AC_REQUIRE([AC_PROG_MAKE_SET])])

#
# Check to make sure that the build environment is sane.
#

AC_DEFUN([AM_SANITY_CHECK],
[AC_MSG_CHECKING([whether build environment is sane])
# Just in case
sleep 1
echo timestamp > conftestfile
# Do `set' in a subshell so we don't clobber the current shell's
# arguments.  Must try -L first in case configure is actually a
# symlink; some systems play weird games with the mod time of symlinks
# (eg FreeBSD returns the mod time of the symlink's containing
# directory).
if (
   set X `ls -Lt $srcdir/configure conftestfile 2> /dev/null`
   if test "[$]*" = "X"; then
      # -L didn't work.
      set X `ls -t $srcdir/configure conftestfile`
   fi
   if test "[$]*" != "X $srcdir/configure conftestfile" \
      && test "[$]*" != "X conftestfile $srcdir/configure"; then

      # If neither matched, then we have a broken ls.  This can happen
      # if, for instance, CONFIG_SHELL is bash and it inherits a
      # broken ls alias from the environment.  This has actually
      # happened.  Such a system could not be considered "sane".
      AC_MSG_ERROR([ls -t appears to fail.  Make sure there is not a broken
alias in your environment])
   fi

   test "[$]2" = conftestfile
   )
then
   # Ok.
   :
else
   AC_MSG_ERROR([newly created file is older than distributed files!
Check your system clock])
fi
rm -f conftest*
AC_MSG_RESULT(yes)])

dnl AM_MISSING_PROG(NAME, PROGRAM, DIRECTORY)
dnl The program must properly implement --version.
AC_DEFUN([AM_MISSING_PROG],
[AC_MSG_CHECKING(for working $2)
# Run test in a subshell; some versions of sh will print an error if
# an executable is not found, even if stderr is redirected.
# Redirect stdin to placate older versions of autoconf.  Sigh.
if ($2 --version) < /dev/null > /dev/null 2>&1; then
   $1=$2
   AC_MSG_RESULT(found)
else
   $1="$3/missing $2"
   AC_MSG_RESULT(missing)
fi
AC_SUBST($1)])

# Like AC_CONFIG_HEADER, but automatically create stamp file.

AC_DEFUN([AM_CONFIG_HEADER],
[AC_PREREQ([2.12])
AC_CONFIG_HEADER([$1])
dnl When config.status generates a header, we must update the stamp-h file.
dnl This file resides in the same directory as the config header
dnl that is generated.  We must strip everything past the first ":",
dnl and everything past the last "/".
AC_OUTPUT_COMMANDS(changequote(<<,>>)dnl
ifelse(patsubst(<<$1>>, <<[^ ]>>, <<>>), <<>>,
<<test -z "<<$>>CONFIG_HEADERS" || echo timestamp > patsubst(<<$1>>, <<^\([^:]*/\)?.*>>, <<\1>>)stamp-h<<>>dnl>>,
<<am_indx=1
for am_file in <<$1>>; do
  case " <<$>>CONFIG_HEADERS " in
  *" <<$>>am_file "*<<)>>
    echo timestamp > `echo <<$>>am_file | sed -e 's%:.*%%' -e 's%[^/]*$%%'`stamp-h$am_indx
    ;;
  esac
  am_indx=`expr "<<$>>am_indx" + 1`
done<<>>dnl>>)
changequote([,]))])

