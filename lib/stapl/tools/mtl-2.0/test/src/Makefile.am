EXTRA_DIST 		= makefile.mak qtest alltests
AUTOMAKE_OPTIONS= no-dependencies

EXTRA_PROGRAMS	= matrix_iterator_test \
		matrix_oned_test \
		matrix_indices_test \
		matrix_ij_test \
		matrix_iter_ij_test \
		matrix_row_col_test \
		matrix_trans_test \
		matrix_scaled_test \
		matrix_mat_algo_test \
		matvec_mult_test \
		matvec_rankone_test \
		ranktwo_test \
		tri_solve_test \
		matmat_add_test \
		matmat_mult_test \
		matmat_copy_test \
		quick_test

Q_PROGRAM	= matrix_mat_algo_test

LDADD =  @MATHLIB@

CXXLD    = @CXXLD@
CXXLINK  = $(CXXLD) $(AM_CXXFLAGS) $(CXXFLAGS) $(LDFLAGS) @LDOUT@$@

tests		: $(EXTRA_PROGRAMS)

all		: tests

qtest		: $(Q_PROGRAM)

echo-tests	:
	@echo $(EXTRA_PROGRAMS)

echo-qtest	:
	@echo $(Q_PROGRAM)

clean		:
		  /bin/rm -f $(EXTRA_PROGRAMS) *.exe

matrix_iterator_test_SOURCES = matrix_iterator_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matrix_oned_test_SOURCES = matrix_oned_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matrix_indices_test_SOURCES = matrix_indices_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matrix_ij_test_SOURCES = matrix_ij_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matrix_iter_ij_test_SOURCES = matrix_iter_ij_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h     \
		      iter_ij_test.h

matrix_row_col_test_SOURCES = matrix_row_col_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matrix_trans_test_SOURCES = matrix_trans_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matrix_scaled_test_SOURCES = matrix_scaled_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matrix_mat_algo_test_SOURCES = matrix_mat_algo_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matvec_mult_test_SOURCES = matvec_mult_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matvec_rankone_test_SOURCES = matvec_rankone_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

ranktwo_test_SOURCES = ranktwo_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h


tri_solve_test_SOURCES = tri_solve_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matmat_add_test_SOURCES = matmat_add_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matmat_mult_test_SOURCES = matmat_mult_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h

matmat_copy_test_SOURCES = matmat_copy_test.cc \
		      matrix_test.h	\
		      algo_test.h	\
		      fill_matrix.h	\
		      matrix_attr.h


quick_test_SOURCES	= quick_test.cc
