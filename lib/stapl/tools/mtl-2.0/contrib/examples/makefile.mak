# -*- Makefile -*-

LINK32_COMMON=kernel32.lib user32.lib gdi32.lib winspool.lib \
   comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib \
  uuid.lib odbc32.lib odbccp32.lib /subsystem:console /incremental:no /machine:I386 

MTLDIR=..\..
ITLDIR=..\..\contrib\itl
STLDIR=C:\Program Files\Microsoft Visual Studio\VC98\Include
VCLIB=C:\Program Files\Microsoft Visual Studio\VC98\Lib

CPP_PROJ=/TP /ML /GX /O2 /I "$(MTLDIR)" /I "$(ITLDIR)" /I "$(STLDIR)" /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c 


CPP=cl.exe
LINK32=link.exe


ALL : \
	"abs_sum.exe" \
	"apply_givens.exe" \
	"banded_matmat.exe" \
	"banded_matvec_mult.exe" \
"banded_view_test.exe" \
"dot_prod.exe" \
"euclid_norm.exe" \
"external_matrix.exe" \
"gather_scatter.exe" \
"general_matvec_mult.exe" \
"lu_factorization.exe" \
"max_index.exe" \
"partition.exe" \
"rank_1_gen_mat.exe" \
"sparse_copy.exe" \
"sparse_matrix.exe" \
"sparse_mult.exe" \
"sparse_mult_col.exe" \
"sparse_tri_solve.exe" \
"sparse_vec_prod.exe" \
"swap_rows.exe" \
"symm_banded_vec_prod.exe" \
"symm_packed_vec_prod.exe" \
"symm_sparse_vec_prod.exe" \
"trans_mult.exe" \
"tri_band_sol.exe" \
"tri_band_vect.exe" \
"tri_matvec_mult.exe" \
"tri_pack_sol.exe" \
"tri_pack_vect.exe" \
"tri_solve.exe" \
"vec_copy.exe" \
"vec_max_index.exe" \
"vec_scale.exe" \
"y_ax_y.exe"

CLEAN :
	-if exist vc60.idb erase vc60.idb
	-if exist *.obj erase *.obj
	-if exist *.pch erase *.pch
	-if exist *.pch erase *.pch
	-if exist *.pdb erase *.pdb
	-if exist *.idb erase *.idb
	-if exist *.pdb erase *.pdb
	-if exist *.idb erase *.idb
	-if exist *.ilk erase *.ilk

SQUEAKY : CLEAN
	-if exist *.exe erase *.exe

.SUFFIXES : .cc

.c{}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cpp{}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cc{}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cxx{}.obj::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.c{}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cpp{}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<

.cxx{}.sbr::
   $(CPP) @<<
   $(CPP_PROJ) $< 
<<


SOURCE=abs_sum.cc
OBJ="abs_sum.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"abs_sum.pdb" /out:"abs_sum.exe" 
"abs_sum.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<


SOURCE=apply_givens.cc
OBJ="apply_givens.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"aapply_givens.pdb" /out:"apply_givens.exe" 
"apply_givens.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

FNAME=banded_matvec_mult
SOURCE=$(FNAME).cc
OBJ="$(FNAME).obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"$(FNAME).pdb" /out:"$(FNAME).exe" 
"$(FNAME).exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<


SOURCE=banded_matmat.cc
OBJ="banded_matmat.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"banded_matmat.pdb" /out:"banded_matmat.exe" 
"banded_matmat.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=banded_view_test.cc
OBJ="banded_view_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"banded_view_test.pdb" /out:"banded_view_test.exe" 
"banded_view_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=cholesky.cc
OBJ="cholesky.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"cholesky.pdb" /out:"cholesky.exe" 
"cholesky.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=cholesky_external.cc
OBJ="cholesky_external.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"cholesky_external.pdb" /out:"cholesky_external.exe" 
"cholesky_external.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=dot_prod.cc
OBJ="dot_prod.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"dot_prod.pdb" /out:"dot_prod.exe" 
"dot_prod.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=euclid_norm.cc
OBJ="euclid_norm.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"euclid_norm.pdb" /out:"euclid_norm.exe" 
"euclid_norm.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=external_matrix.cc
OBJ="external_matrix.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"external_matrix.pdb" /out:"external_matrix.exe" 
"external_matrix.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=gather_scatter.cc
OBJ="gather_scatter.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"gather_scatter.pdb" /out:"gather_scatter.exe" 
"gather_scatter.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=general_matvec_mult.cc
OBJ="general_matvec_mult.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"general_matvec_mult.pdb" /out:"general_matvec_mult.exe" 
"general_matvec_mult.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=lu_factorization.cc
OBJ="lu_factorization.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"lu_factorization.pdb" /out:"lu_factorization.exe" 
"lu_factorization.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=max_index.cc
OBJ="max_index.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"max_index.pdb" /out:"max_index.exe" 
"max_index.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=partition.cc
OBJ="partition.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"partition.pdb" /out:"partition.exe" 
"partition.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<


SOURCE=rank_1_gen_mat.cc
OBJ="rank_1_gen_mat.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"rank_1_gen_mat.pdb" /out:"rank_1_gen_mat.exe" 
"rank_1_gen_mat.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=sparse_copy.cc
OBJ="sparse_copy.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"sparse_copy.pdb" /out:"sparse_copy.exe" 
"sparse_copy.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=sparse_matrix.cc
OBJ="sparse_matrix.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"sparse_matrix.pdb" /out:"sparse_matrix.exe" 
"sparse_matrix.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=sparse_mult.cc
OBJ="sparse_mult.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"sparse_mult.pdb" /out:"sparse_mult.exe" 
"sparse_mult.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=sparse_mult_col.cc
OBJ="sparse_mult_col.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"sparse_mult_col.pdb" /out:"sparse_mult_col.exe" 
"sparse_mult_col.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=sparse_tri_solve.cc
OBJ="sparse_tri_solve.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"sparse_tri_solve.pdb" /out:"sparse_tri_solve.exe" 
"sparse_tri_solve.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=sparse_vec_prod.cc
OBJ="sparse_vec_prod.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"sparse_vec_prod.pdb" /out:"sparse_vec_prod.exe" 
"sparse_vec_prod.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=swap_rows.cc
OBJ="swap_rows.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"swap_rows.pdb" /out:"swap_rows.exe" 
"swap_rows.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=symm_banded_vec_prod.cc
OBJ="symm_banded_vec_prod.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"symm_banded_vec_prod.pdb" /out:"symm_banded_vec_prod.exe" 
"symm_banded_vec_prod.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=symm_packed_vec_prod.cc
OBJ="symm_packed_vec_prod.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"symm_packed_vec_prod.pdb" /out:"symm_packed_vec_prod.exe" 
"symm_packed_vec_prod.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=symm_sparse_vec_prod.cc
OBJ="symm_sparse_vec_prod.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"symm_sparse_vec_prod.pdb" /out:"symm_sparse_vec_prod.exe" 
"symm_sparse_vec_prod.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=trans_mult.cc
OBJ="trans_mult.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"trans_mult.pdb" /out:"trans_mult.exe" 
"trans_mult.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=tri_band_sol.cc
OBJ="tri_band_sol.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"tri_band_sol.pdb" /out:"tri_band_sol.exe" 
"tri_band_sol.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=tri_band_vect.cc
OBJ="tri_band_vect.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"tri_band_vect.pdb" /out:"tri_band_vect.exe" 
"tri_band_vect.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=tri_matvec_mult.cc
OBJ="tri_matvec_mult.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"tri_matvec_mult.pdb" /out:"tri_matvec_mult.exe" 
"tri_matvec_mult.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=tri_pack_sol.cc
OBJ="tri_pack_sol.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"tri_pack_sol.pdb" /out:"tri_pack_sol.exe" 
"tri_pack_sol.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=tri_pack_vect.cc
OBJ="tri_pack_vect.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"tri_pack_vect.pdb" /out:"tri_pack_vect.exe" 
"tri_pack_vect.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=tri_solve.cc
OBJ="tri_solve.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"tri_solve.pdb" /out:"tri_solve.exe" 
"tri_solve.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=vec_copy.cc
OBJ="vec_copy.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"vec_copy.pdb" /out:"vec_copy.exe" 
"vec_copy.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=vec_max_index.cc
OBJ="vec_max_index.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"vec_max_index.pdb" /out:"vec_max_index.exe" 
"vec_max_index.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=vec_scale.cc
OBJ="vec_scale.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"vec_scale.pdb" /out:"vec_scale.exe" 
"vec_scale.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=y_ax_y.cc
OBJ="y_ax_y.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"y_ax_y.pdb" /out:"y_ax_y.exe" 
"y_ax_y.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<
