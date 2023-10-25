# -*- Makefile -*-
#  makefile for use with nmake (comes iwth Visual C++)
# 
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
		matrix_iterator_test.exe \
		matrix_oned_test.exe \
		matrix_indices_test.exe \
		matrix_ij_test.exe \
		matrix_iter_ij_test.exe \
		matrix_row_col_test.exe \
		matrix_trans_test.exe \
		matrix_scaled_test.exe \
		matrix_mat_algo_test.exe \
		matvec_mult_test.exe \
		matvec_rankone_test.exe \
		ranktwo_test.exe \
		tri_solve_test.exe \
		matmat_add_test.exe \
		matmat_mult_test.exe \
		matmat_copy_test.exe


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


SOURCE=matrix_iterator_test.cc
OBJ="matrix_iterator_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matrix_iterator_test.pdb" /out:"matrix_iterator_test.exe" 
"matrix_iterator_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matrix_oned_test.cc
OBJ="matrix_oned_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matrix_oned_test.pdb" /out:"matrix_oned_test.exe" 
"matrix_oned_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matrix_indices_test.cc
OBJ="matrix_indices_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matrix_indices_test.pdb" /out:"matrix_indices_test.exe" 
"matrix_indices_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matrix_ij_test.cc
OBJ="matrix_ij_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matrix_ij_test.pdb" /out:"matrix_ij_test.exe" 
"matrix_ij_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matrix_iter_ij_test.cc
OBJ="matrix_iter_ij_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matrix_iter_ij_test.pdb" /out:"matrix_iter_ij_test.exe" 
"matrix_iter_ij_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matrix_row_col_test.cc
OBJ="matrix_row_col_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matrix_row_col_test.pdb" /out:"matrix_row_col_test.exe" 
"matrix_row_col_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matrix_trans_test.cc
OBJ="matrix_trans_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matrix_trans_test.pdb" /out:"matrix_trans_test.exe" 
"matrix_trans_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matrix_scaled_test.cc
OBJ="matrix_scaled_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matrix_scaled_test.pdb" /out:"matrix_scaled_test.exe" 
"matrix_scaled_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matrix_mat_algo_test.cc
OBJ="matrix_mat_algo_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matrix_mat_algo_test.pdb" /out:"matrix_mat_algo_test.exe" 
"matrix_mat_algo_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matvec_mult_test.cc
OBJ="matvec_mult_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matvec_mult_test.pdb" /out:"matvec_mult_test.exe" 
"matvec_mult_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matvec_rankone_test.cc
OBJ="matvec_rankone_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matvec_rankone_test.pdb" /out:"matvec_rankone_test.exe" 
"matvec_rankone_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=ranktwo_test.cc
OBJ="ranktwo_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"ranktwo_test.pdb" /out:"ranktwo_test.exe" 
"ranktwo_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=tri_solve_test.cc
OBJ="tri_solve_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"tri_solve_test.pdb" /out:"tri_solve_test.exe" 
"tri_solve_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matmat_add_test.cc
OBJ="matmat_add_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matmat_add_test.pdb" /out:"matmat_add_test.exe" 
"matmat_add_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matmat_mult_test.cc
OBJ="matmat_mult_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matmat_mult_test.pdb" /out:"matmat_mult_test.exe" 
"matmat_mult_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

SOURCE=matmat_copy_test.cc
OBJ="matmat_copy_test.obj" 
$(OBJ) : $(SOURCE) 
LINK32_FLAGS = $(LINK32_COMMON) /LIBPATH:"$(VCLIB)" /pdb:"matmat_copy_test.pdb" /out:"matmat_copy_test.exe" 
"matmat_copy_test.exe" : $(DEF_FILE) $(OBJ)
    $(LINK32) @<<
  $(LINK32_FLAGS) $(OBJ)
<<

