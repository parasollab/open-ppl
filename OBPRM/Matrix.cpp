// $Id$
////////////////////////////////////////////////////////////////////////////
//  Matrix.c
//
//  Created   4/24/98 Aaron Michalk
//  Added     5/16/98 Wookho Son
//  Added     5/27/98 Wookho Son
//  Added     6/ 5/98 Wookho Son
////////////////////////////////////////////////////////////////////////////

// This is a test
#include "Matrix.h"
#include <math.h>
// this is test 3
//#include <imsl.h>


//==========================================================================
//  Constructors and Destructor
//==========================================================================
Matrix::Matrix(int _nrow, int _ncol) {
    int  i, j;

    ncol = _ncol;
    nrow = _nrow;
    m = new double[ncol*nrow];
    
    for (i=0; i<_nrow; i++)
        for (j=0; j<_ncol; j++)
	    m[i*ncol + j] = 0.0;	    
}

Matrix::Matrix(double *_m, int _nrow, int _ncol) {
    m = _m;
    ncol = _ncol;
    nrow = _nrow;
}

#if 0
// Added  6/ 5/98  Wookho Son
Matrix::Matrix(double *_m, int _n) {
    int  i, j;

    ncol = ncol = _n;
    m = new double[_n*_n];
    
    for (i=0; i<_n; i++)
        for (j=0; j<_n; j++)
	    m[i*_n + j] = 0.0;	    

    for (i=0; i<_n; i++)
        m[i*_n + i] = _m[i];	    
}
#endif

Matrix::Matrix(const Matrix & _m) {
    m = 0;
    Copy(_m);
}

Matrix::~Matrix() {
    if (m)
        delete[] m;
}

//==========================================================================
//  Operators
//==========================================================================
Matrix Matrix::operator+(Matrix & _m) {
    if ((ncol != _m.ncol) || (nrow != _m.nrow))
        return Matrix();
    Matrix result(nrow, ncol);
    for (int i=0; i < ncol*nrow; i++)
        result.m[i] = m[i] + _m.m[i];
    return result;
}

Matrix Matrix::operator-(Matrix & _m) {
    if ((ncol != _m.ncol) || (nrow != _m.nrow))
        return Matrix();
    Matrix result(nrow, ncol);
    for (int i=0; i < ncol*nrow; i++)
        result.m[i] = m[i] - _m.m[i];
    return result;
}

Matrix Matrix::operator-() {
    Matrix result(nrow, ncol);
    for (int i=0; i < ncol*nrow; i++)
        result.m[i] = -m[i];
    return result;
}

Matrix Matrix::operator*(Matrix & _m) {
    if (ncol != _m.nrow)
        return Matrix();

    double *resultm = new double [nrow*_m.ncol];
//    double * resultm = imsl_d_mat_mul_rect("A*B", IMSL_A_MATRIX, nrow, ncol, m, IMSL_B_MATRIX, _m.nrow, _m.ncol, _m.m, 0);

    for (int i=0; i<nrow; i++)
        for (int j=0; j<_m.ncol; j++){
	    resultm[i*_m.ncol + j] = 0.0;
	    for (int k=0; k<nrow; k++)
	        resultm[i*_m.ncol + j] += m[i*ncol + k] * _m.m[k*_m.nrow + j];	        
	}

    return Matrix(resultm, nrow, _m.ncol);
}

Matrix & Matrix::operator=(const Matrix & _m) {
    if (m)
        delete[] m;
    Copy(_m);
    return *this;
}

//==========================================================================
//  Copy
//==========================================================================
void Matrix::Copy(const Matrix & _m) {
    ncol = _m.ncol;
    nrow = _m.nrow;
    m = new double[nrow*ncol];
    for (int i=0; i < nrow*ncol; i++)
        m[i] = _m.m[i];
}

//==========================================================================
//  Inverse
//==========================================================================
Matrix Matrix::Inverse() {
    if (ncol != nrow)
        return Matrix();
    double * result;
//    imsl_d_lin_sol_gen(nrow, m, m, IMSL_INVERSE, &result, 0);
    return Matrix(result, nrow, ncol);
}

//==========================================================================
//  Transpose
//==========================================================================
Matrix Matrix::Transpose() {
    int  i, j;
//    cout << "Transposing from dimension (" << nrow << ", " << ncol << ")" << " to dimension (" << ncol << ", " << nrow << ")" << endl;

    double * resultm = new double [ncol*nrow];
//    result = imsl_d_mat_mul_rect("trans(A)", IMSL_A_MATRIX, nrow, ncol, m, 0);
    for (i=0; i<nrow; i++)
        for (j=0; j<ncol; j++)
	    resultm[j*nrow + i] = m[i*ncol + j];

    return Matrix(resultm, ncol, nrow);
}


//==========================================================================
//  RowColumnCouple
//  Added   Wookho Son
//==========================================================================
void Matrix::RowColumnCouple(Matrix & _m) {
    // Reallocate the memory by the amount increased
    double * resultm = new double[(nrow+_m.nrow)*(ncol+_m.ncol)];
    int i;

    for (i=0; i < nrow*ncol; i++)
        resultm[i] = m[i];

    for (i=0; i < _m.nrow*_m.ncol; i++)
        resultm[ncol*nrow + i] = _m.m[i];

    delete[] m;
    m = resultm;

    nrow += _m.nrow;
    ncol += _m.ncol;
}


//==========================================================================
//  Couple
//  Added   Wookho Son
//==========================================================================
void Matrix::RowCouple(Matrix & _m) {
    int  i, j;

    // Make sure that the number of rows are the same
    if (ncol!=_m.ncol){
       cout << "Trying to couple two matrices of different column-size" << endl;
       exit(1);
     }

    // Reallocate the memory by the amount increased
    double * resultm = new double[(nrow+_m.nrow)*ncol];

    for (i=0; i < nrow*ncol; i++)
        resultm[i] = m[i];

    for (i=0; i < _m.nrow; i++)
        for (j=0; j < _m.ncol; j++)
	    resultm[nrow*ncol + i*_m.ncol + j] = _m.m[i*_m.ncol + j];

    delete[] m;
    m = resultm;

    nrow += _m.nrow;
}

//==========================================================================
//  Couple
//  Added   Wookho Son
//==========================================================================
void Matrix::ColumnCouple(Matrix & _m) {
    int  i, j;

    // Make sure that the number of rows are the same
    if (nrow!=_m.nrow){
       cout << "Trying to couple two matrices of different row-size" << endl;
       exit(1);
     }

    // Reallocate the memory by the amount increased
    double * resultm = new double[nrow*(ncol+_m.ncol)];

    for (i=0; i < nrow; i++)
        for (j=0; j < ncol; j++)
	    resultm[i*(ncol+_m.ncol) + j] = m[i*ncol + j];

    for (i=0; i < _m.nrow; i++)
        for (j=0; j < _m.ncol; j++)
	    resultm[i*(ncol+_m.ncol) + (ncol+j)] = _m.m[i*_m.ncol + j];

    delete[] m;
    m = resultm;

    ncol += _m.ncol;
}

//==========================================================================
//  Couple
//  Added   Wookho Son
//==========================================================================
Matrix Matrix::Extract(int _startRow, int _endRow, int _startCol, int _endCol) {
    int  i, j;

    int rowSize = _endRow - _startRow + 1;
    int colSize = _endCol - _startCol + 1;
    double *resultm = new double [rowSize*colSize];

    for (i = _startRow; i <= _endRow; i++)
        for (j = _startCol; j <= _endCol; j++)
	    resultm[(i - _startRow)*colSize + (j - _startCol)] = m[i*ncol + j];

//    for (i=_startRow; i<_endRow; i++){
//        for (j=_startCol; j<_endCol; j++)
//	    cout << m[i*ncol + j] << " ";
//	cout << endl;
//    }

    Matrix result(resultm, rowSize, colSize);
    result.Print();
    return result;
}

//==========================================================================
//  Read
//==========================================================================
void Matrix::Read(ifstream & _is) {
    _is >> nrow;
    _is >> ncol;
    if (m)
        delete[] m;
    m = new double[nrow*ncol];
    for (int i=0; i < nrow*ncol; i++)
        _is >> m[i];
}

//==========================================================================
//  Write
//==========================================================================
void Matrix::Write(ostream & _os) {
    _os << nrow << " ";
    _os << ncol << " ";
    for (int i=0; i < nrow*ncol; i++)
        _os << m[i] << " ";
}

//==========================================================================
//  Print
//
//  Added  5/27/98   Wookho Son
//==========================================================================
void Matrix::Print() {
    int  i, j;

    cout << "nrow=" << nrow << ",  ncol=" << ncol << endl;

    for (i=0; i<nrow; i++){
        for (j=0; j<ncol; j++)
	    cout << m[i*ncol + j] << " ";
	cout << endl;
    }

    cout << endl;
}
