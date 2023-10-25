# Generated automatically from Makefile.in by configure.
# Makefile.in generated automatically by automake 1.4-p5 from Makefile.am

# Copyright (C) 1994, 1995-8, 1999, 2001 Free Software Foundation, Inc.
# This Makefile.in is free software; the Free Software Foundation
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.


SHELL = /bin/sh

srcdir = .
top_srcdir = ..
prefix = /usr/local
exec_prefix = ${prefix}

bindir = ${exec_prefix}/bin
sbindir = ${exec_prefix}/sbin
libexecdir = ${exec_prefix}/libexec
datadir = ${prefix}/share
sysconfdir = ${prefix}/etc
sharedstatedir = ${prefix}/com
localstatedir = ${prefix}/var
libdir = ${exec_prefix}/lib
infodir = ${prefix}/info
mandir = ${prefix}/man
oldincludedir = /usr/include

DESTDIR =

pkgdatadir = $(datadir)/mtl
pkglibdir = $(libdir)/mtl
pkgincludedir = $(includedir)/mtl

top_builddir = ..

ACLOCAL = aclocal
AUTOCONF = autoconf
AUTOMAKE = automake
AUTOHEADER = autoheader

INSTALL = /usr/bin/install -c
INSTALL_PROGRAM = ${INSTALL} $(AM_INSTALL_PROGRAM_FLAGS)
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_SCRIPT = ${INSTALL_PROGRAM}
transform = s,x,x,

NORMAL_INSTALL = :
PRE_INSTALL = :
POST_INSTALL = :
NORMAL_UNINSTALL = :
PRE_UNINSTALL = :
POST_UNINSTALL = :
build_alias = i686-pc-linux-gnu
build_triplet = i686-pc-linux-gnu
host_alias = i686-pc-linux-gnu
host_triplet = i686-pc-linux-gnu
target_alias = i686-pc-linux-gnu
target_triplet = i686-pc-linux-gnu
BLASLIB =  
CC = gcc
CCFLAGS = 
CC_DEBUG_FLAGS = 
CC_OPTIMIZE_FLAGS = 
CPP = @CPP@
CXX = g++
CXXFLAGS = -ftemplate-depth-30 -Wall
CXXLD = g++
CXX_DEBUG_FLAGS = -g -DBZ_DEBUG
CXX_OPTIMIZE_FLAGS = -O2 -funroll-loops -fstrict-aliasing -fno-gcse
F77 = g77
FLIBS = 
FORTLIBS = -lg2c
LAPACK_EXAMPLES = 
LDFLAGS = 
LDOUT = -o 
MAKEINFO = makeinfo
MATHLIB = -lm
MTLRELEASE = 21
MTLVERSION = 2.1.2
PACKAGE = mtl
VERSION = 2.1.2-21
contrib_dir = $(top_srcdir)/contrib
mtl_include_dir = $(top_srcdir)

AUTOMAKE_OPTIONS = foreign

includedir = ${prefix}/include/mtl

EXTRA_HEADERS = \
	abs.h			\
	array2D.h		\
	banded_indexer.h	\
	bare_bones_array.h	\
	blais.h			\
	blas_interface.cc	\
	blas_interface.h	\
	block1D.h		\
	block2D.h		\
	compressed1D.h		\
	compressed2D.h		\
	compressed_iter.h	\
	mtl_config.h.in		\
	mtl_config.h		\
	conj.h			\
	cplx_dbldbl.h		\
	dense1D.h		\
	dense2D.h		\
	dense_iterator.h	\
	diagonal_indexer.h	\
	dimension.h		\
	dim_calc.h		\
	dumptofile.h		\
	elt.h			\
	entry.h			\
	envelope2D.h		\
	external_vector.h	\
	fast.h			\
	harwell_boeing_stream.h	\
	initialize.h		\
	iohb.c			\
	iohb.h			\
	iterator_adaptor.h	\
	lapack_interface.h	\
	light1D.h		\
	light_matrix.h		\
	linalg_vec.h		\
	matlabio.h		\
	matrix.h		\
	matrix_implementation.h	\
	matrix_market_stream.h	\
	matrix_stream.h		\
	matrix_traits.h		\
	meta_equal.h		\
	meta_if.h		\
	mmio.c			\
	mmio.h			\
	mtl.h			\
	mtl_limits.h		\
	mtl_set.h		\
	mtl_complex.h		\
	mtl_iterator.h		\
	lu.h                    \
	mtl2lapack.h		\
	mtl_algo.h		\
	mtl_exception.h		\
	norm.h			\
	not_at.h		\
	oned_part.h		\
	orien.h			\
	partition.h             \
	rect_indexer.h		\
	refcnt_ptr.h		\
	reverse_iter.h		\
	scale_iterator.h	\
	scaled1D.h		\
	scaled2D.h		\
	sparse1D.h		\
	sparse_iterator.h	\
	stamp-h.in		\
	strided1D.h		\
	strided_iterator.h	\
	transform_iterator.h	\
	uplo.h			\
	utils.h


EXTRA_DIST = $(EXTRA_HEADERS)
mkinstalldirs = $(SHELL) $(top_srcdir)/mkinstalldirs
CONFIG_HEADER = mtl_config.h
CONFIG_CLEAN_FILES = 
DIST_COMMON =  ./stamp-h.in Makefile.am Makefile.in mtl_config.h.in


DISTFILES = $(DIST_COMMON) $(SOURCES) $(HEADERS) $(TEXINFOS) $(EXTRA_DIST)

TAR = gtar
GZIP_ENV = --best
all: all-redirect
.SUFFIXES:
$(srcdir)/Makefile.in: Makefile.am $(top_srcdir)/configure.in $(ACLOCAL_M4) 
	cd $(top_srcdir) && $(AUTOMAKE) --foreign --include-deps mtl/Makefile

Makefile: $(srcdir)/Makefile.in  $(top_builddir)/config.status
	cd $(top_builddir) \
	  && CONFIG_FILES=$(subdir)/$@ CONFIG_HEADERS= $(SHELL) ./config.status


mtl_config.h: stamp-h
	@if test ! -f $@; then \
		rm -f stamp-h; \
		$(MAKE) stamp-h; \
	else :; fi
stamp-h: $(srcdir)/mtl_config.h.in $(top_builddir)/config.status
	cd $(top_builddir) \
	  && CONFIG_FILES= CONFIG_HEADERS=mtl/mtl_config.h \
	     $(SHELL) ./config.status
	@echo timestamp > stamp-h 2> /dev/null
$(srcdir)/mtl_config.h.in: $(srcdir)/stamp-h.in
	@if test ! -f $@; then \
		rm -f $(srcdir)/stamp-h.in; \
		$(MAKE) $(srcdir)/stamp-h.in; \
	else :; fi
$(srcdir)/stamp-h.in: $(top_srcdir)/configure.in $(ACLOCAL_M4) 
	cd $(top_srcdir) && $(AUTOHEADER)
	@echo timestamp > $(srcdir)/stamp-h.in 2> /dev/null

mostlyclean-hdr:

clean-hdr:

distclean-hdr:
	-rm -f mtl_config.h

maintainer-clean-hdr:

tags: TAGS

ID: $(HEADERS) $(SOURCES) $(LISP)
	list='$(SOURCES) $(HEADERS)'; \
	unique=`for i in $$list; do echo $$i; done | \
	  awk '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	here=`pwd` && cd $(srcdir) \
	  && mkid -f$$here/ID $$unique $(LISP)

TAGS:  $(HEADERS) $(SOURCES) mtl_config.h.in $(TAGS_DEPENDENCIES) $(LISP)
	tags=; \
	here=`pwd`; \
	list='$(SOURCES) $(HEADERS)'; \
	unique=`for i in $$list; do echo $$i; done | \
	  awk '    { files[$$0] = 1; } \
	       END { for (i in files) print i; }'`; \
	test -z "$(ETAGS_ARGS)mtl_config.h.in$$unique$(LISP)$$tags" \
	  || (cd $(srcdir) && etags $(ETAGS_ARGS) $$tags mtl_config.h.in $$unique $(LISP) -o $$here/TAGS)

mostlyclean-tags:

clean-tags:

distclean-tags:
	-rm -f TAGS ID

maintainer-clean-tags:

distdir = $(top_builddir)/$(PACKAGE)-$(VERSION)/$(subdir)

subdir = mtl

distdir: $(DISTFILES)
	@for file in $(DISTFILES); do \
	  d=$(srcdir); \
	  if test -d $$d/$$file; then \
	    cp -pr $$d/$$file $(distdir)/$$file; \
	  else \
	    test -f $(distdir)/$$file \
	    || ln $$d/$$file $(distdir)/$$file 2> /dev/null \
	    || cp -p $$d/$$file $(distdir)/$$file || :; \
	  fi; \
	done
info-am:
info: info-am
dvi-am:
dvi: dvi-am
check-am: all-am
check: check-am
installcheck-am:
installcheck: installcheck-am
all-recursive-am: mtl_config.h
	$(MAKE) $(AM_MAKEFLAGS) all-recursive

install-exec-am:
install-exec: install-exec-am

install-data-am:
	@$(NORMAL_INSTALL)
	$(MAKE) $(AM_MAKEFLAGS) install-data-hook
install-data: install-data-am

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am
install: install-am
uninstall-am:
uninstall: uninstall-am
all-am: Makefile mtl_config.h
all-redirect: all-am
install-strip:
	$(MAKE) $(AM_MAKEFLAGS) AM_INSTALL_PROGRAM_FLAGS=-s install
installdirs:


mostlyclean-generic:

clean-generic:

distclean-generic:
	-rm -f Makefile $(CONFIG_CLEAN_FILES)
	-rm -f config.cache config.log stamp-h stamp-h[0-9]*

maintainer-clean-generic:
mostlyclean-am:  mostlyclean-hdr mostlyclean-tags mostlyclean-generic

mostlyclean: mostlyclean-am

clean-am:  clean-hdr clean-tags clean-generic mostlyclean-am

clean: clean-am

distclean-am:  distclean-hdr distclean-tags distclean-generic clean-am

distclean: distclean-am

maintainer-clean-am:  maintainer-clean-hdr maintainer-clean-tags \
		maintainer-clean-generic distclean-am
	@echo "This command is intended for maintainers to use;"
	@echo "it deletes files that may require special tools to rebuild."

maintainer-clean: maintainer-clean-am

.PHONY: mostlyclean-hdr distclean-hdr clean-hdr maintainer-clean-hdr \
tags mostlyclean-tags distclean-tags clean-tags maintainer-clean-tags \
distdir info-am info dvi-am dvi check check-am installcheck-am \
installcheck all-recursive-am install-exec-am install-exec \
install-data-am install-data install-am install uninstall-am uninstall \
all-redirect all-am all installdirs mostlyclean-generic \
distclean-generic clean-generic maintainer-clean-generic clean \
mostlyclean distclean maintainer-clean


make-install-dirs:
	-if test '!' -d $(includedir); then mkdir -p $(includedir); fi

install-data-hook: make-install-dirs
	-@ echo Installing $(includedir) ; \
	$(INSTALL_DATA) *.h $(includedir)

# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
