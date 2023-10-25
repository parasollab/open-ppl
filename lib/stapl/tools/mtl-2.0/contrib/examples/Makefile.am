EXTRA_DIST 		= makefile.mak
AUTOMAKE_OPTIONS 	= no-dependencies

MTL_EXAMPLES 		=						\
		abs_sum apply_givens banded_matvec_mult dot_prod 	\
		euclid_norm external_matrix general_matvec_mult 	\
		max_index rank_1_gen_mat sparse_copy sparse_matrix 	\
		sparse_vec_prod swap_rows symm_banded_vec_prod 		\
		symm_packed_vec_prod symm_sparse_vec_prod 		\
		tri_band_sol tri_band_vect tri_matvec_mult tri_pack_sol \
		tri_pack_vect tri_solve trans_mult vec_copy 		\
		vec_max_index vec_scale y_ax_y gather_scatter 		\
		partition lu_factorization 				\
		banded_view_test blocked_matrix 			\
		blocked_vector sparse_mult sparse_mult_col              \
		transpose array2D rank_1_symm_sparse rank_one           \
		symm_matvec_mult \
		vecvec_add vecvec_add3 vecvec_copy vecvec_dot vecvec_ele_div \
		vecvec_ele_mult vecvec_swap vec_two_norm vec_max vec_min \
		range lu_solve neg_stride subrange_vector resize \
	        banded_matmat sparse_tri_solve

MTL_LAPACK_EXAMPLES	= getrf geev gecon geequ kiester_geev

EXTRA_PROGRAMS = $(MTL_EXAMPLES) $(MTL_LAPACK_EXAMPLES)

# don't want any optimizations here
OPTFLAGS = 

LDADD           = @MATHLIB@

getrf_LDADD	= @BLASLIB@
geev_LDADD	= @BLASLIB@
kiester_geev_LDADD	= @BLASLIB@
gecon_LDADD	= @BLASLIB@
geequ_LDADD	= @BLASLIB@

CXXLD    = @CXXLD@
CXXLINK  = $(CXXLD) $(AM_CXXFLAGS) $(CXXFLAGS) $(LDFLAGS) @LDOUT@$@

mtl_examples			: $(MTL_EXAMPLES)
lapack_examples			: $(MTL_LAPACK_EXAMPLES)
examples			: mtl_examples lapack_examples

all				: mtl_examples @LAPACK_EXAMPLES@
clean				:
				  /bin/rm -f $(EXTRA_PROGRAMS)

abs_sum_SOURCES 		= abs_sum.cc
apply_givens_SOURCES 		= apply_givens.cc
array2D_SOURCES			= array2D.cc
banded_matvec_mult_SOURCES 	= banded_matvec_mult.cc
dot_prod_SOURCES 		= dot_prod.cc
euclid_norm_SOURCES 		= euclid_norm.cc
external_matrix_SOURCES 	= external_matrix.cc
general_matvec_mult_SOURCES 	= general_matvec_mult.cc
max_index_SOURCES 		= max_index.cc
rank_1_gen_mat_SOURCES 		= rank_1_gen_mat.cc
rank_1_symm_sparse_SOURCES	= rank_1_symm_sparse.cc
rank_one_SOURCES		= rank_one.cc
sparse_copy_SOURCES 		= sparse_copy.cc
sparse_matrix_SOURCES 		= sparse_matrix.cc
sparse_vec_prod_SOURCES 	= sparse_vec_prod.cc
swap_rows_SOURCES 		= swap_rows.cc
symm_banded_vec_prod_SOURCES 	= symm_banded_vec_prod.cc
symm_packed_vec_prod_SOURCES 	= symm_packed_vec_prod.cc
symm_sparse_vec_prod_SOURCES 	= symm_sparse_vec_prod.cc
trans_mult_SOURCES		= trans_mult.cc
tri_band_sol_SOURCES 		= tri_band_sol.cc
tri_band_vect_SOURCES 		= tri_band_vect.cc
tri_matvec_mult_SOURCES 	= tri_matvec_mult.cc
tri_pack_sol_SOURCES 		= tri_pack_sol.cc
tri_pack_vect_SOURCES 		= tri_pack_vect.cc
tri_solve_SOURCES 		= tri_solve.cc
vec_copy_SOURCES 		= vec_copy.cc
vec_max_index_SOURCES 		= vec_max_index.cc
vec_max_SOURCES 		= vec_max.cc
vec_min_SOURCES 		= vec_min.cc
vec_scale_SOURCES 		= vec_scale.cc
y_ax_y_SOURCES 			= y_ax_y.cc
gather_scatter_SOURCES		= gather_scatter.cc
partition_SOURCES		= partition.cc
lu_factorization_SOURCES	= lu_factorization.cc
banded_view_test_SOURCES	= banded_view_test.cc
blocked_matrix_SOURCES		= blocked_matrix.cc
blocked_vector_SOURCES		= blocked_vector.cc
sparse_mult_SOURCES		= sparse_mult.cc
sparse_mult_col_SOURCES		= sparse_mult_col.cc
getrf_SOURCES			= getrf.cc
geev_SOURCES			= geev.cc
kiester_geev_SOURCES		= kiester_geev.cc
gecon_SOURCES			= gecon.cc
geequ_SOURCES			= geequ.cc
transpose_SOURCES		= transpose.cc
symm_matvec_mult_SOURCES	= symm_matvec_mult.cc
vecvec_add_SOURCES		= vecvec_add.cc
vecvec_add3_SOURCES		= vecvec_add3.cc
vecvec_copy_SOURCES		= vecvec_copy.cc
vecvec_dot_SOURCES		= vecvec_dot.cc
vecvec_ele_div_SOURCES		= vecvec_ele_div.cc
vecvec_ele_mult_SOURCES		= vecvec_ele_mult.cc
vecvec_swap_SOURCES		= vecvec_swap.cc
vec_two_norm_SOURCES		= vec_two_norm.cc
range_SOURCES			= range.cc
lu_solve_SOURCES		= lu_solve.cc
neg_stride_SOURCES		= neg_stride.cc
subrange_vector_SOURCES		= subrange_vector.cc
resize_SOURCES			= resize.cc
banded_matmat_SOURCES		= banded_matmat.cc
sparse_tri_solve_SOURCES	= sparse_tri_solve.cc
