// $Id$
////////////////////////////////////////////////////////////////////////////
//  Matrix.h
//
//  Created   4/24/98 Aaron Michalk
//  Added     5/15/98 Wookho Son
//  Added     6/ 5/98 Wookho Son
////////////////////////////////////////////////////////////////////////////

#ifndef Matrix_h
#define Matrix_h

#include <stdlib.h>
#include <fstream.h>
#include <iostream.h>


class Matrix {
public:
    //----------------------------------------------------------------------
    //  Constructors and Destructor
    //----------------------------------------------------------------------
    Matrix(int _ncol, int _nrow);
    Matrix(double *_m = 0, int _ncol = 0, int _nrow = 0);
//    Matrix(double *_m = 0, int _n = 0);
    Matrix(const Matrix & _m);
    ~Matrix();
    //----------------------------------------------------------------------
    //  Operators
    //----------------------------------------------------------------------
    Matrix operator+(Matrix & _matrix);
    Matrix operator-(Matrix & _matrix);
    Matrix operator-();
    Matrix operator*(Matrix & _matrix);
    Matrix & operator=(const Matrix & _matrix);
    double & operator()(int _i, int _j);
    //----------------------------------------------------------------------
    //  Methods
    //----------------------------------------------------------------------
    int GetNrows();
    int GetNcols();
    double * GetArray();
    virtual Matrix Inverse();
    Matrix Transpose();
    void RowColumnCouple(Matrix & _m);
    void RowCouple(Matrix & _m);
    void ColumnCouple(Matrix & _m);
    Matrix Extract(int _startRow, int _endRow, int _startCol, int _endCol);

    virtual void Copy(const Matrix & _matrix);
    virtual void Read(ifstream & _is);
    virtual void Write(ostream & _os);
    void Print();
    //----------------------------------------------------------------------
    //  Data
    //----------------------------------------------------------------------
    double *m;
    int ncol;
    int nrow;
protected:
};

//==========================================================================
// Inline functions
//==========================================================================
inline double & Matrix::operator()(int _i, int _j) {
    return m[(_i-1)*ncol + (_j-1)];
}

//==========================================================================
// Inline functions
//==========================================================================
inline int Matrix::GetNrows() {
    return  nrow;
}

//==========================================================================
// Inline functions
//==========================================================================
inline int Matrix::GetNcols() {
    return  ncol;
}

//==========================================================================
// Inline functions
//==========================================================================
inline double * Matrix::GetArray() {
    return  m;
}

#endif
