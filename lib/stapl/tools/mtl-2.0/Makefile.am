EXTRA_DIST 		= LICENSE insertlic.csh configure acconfig.h
DIST_SUBDIRS		= test contrib mtl
SUBDIRS 		= test contrib mtl
AUTOMAKE_OPTIONS 	= foreign
TAR			= tar

dist-hook:
	-@ autoconf
	-@ echo 'Stamping licenses'
	-@ ./insertlic.csh `find $(distdir) -type f`
