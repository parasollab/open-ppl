// $Id$
////////////////////////////////////////////////////////////////////////////
//  LinearSystem.c
//
//  Created   6/ 4/98  Wookho Son
//  Added   6/ 10/98  Wookho Son
////////////////////////////////////////////////////////////////////////////

//typedef int bool;

#include "LinearSystem.h"
//#include <engine.h>
//#include <math.h>

#define MAX(a, b)  ( (a > b) ? a : b )
#define MIN(a, b)  ( (a > b) ? b : a )

#define NUMERICAL_RECIPES
//#define IMSL
//#define MATLAB



//==========================================================================
//  Constructors and Destructor
//==========================================================================
LinearSystem::LinearSystem(double * _m, int _nrow, int _ncol, double *_rhs, int n)
{
    lhs = Matrix(_m, _nrow, _ncol);
    rhs = _rhs;
    s = 0;
    x = 0;
}


//==========================================================================
//  Constructors and Destructor
//==========================================================================
LinearSystem::LinearSystem(Matrix & _m, double * _rhs) :
    lhs(_m),
    rhs(_rhs)
{
}

LinearSystem::~LinearSystem() {
    if (s)
        delete[] s;
    if (x)
        delete[] x;
    if (rhs)
        delete[] rhs;
}


//==========================================================================
//  Operators
//==========================================================================

#ifdef IMSL
//==========================================================================
//  Solve
//  
//  Numerical Recipe version
//
//  Linear System :  Ax = b
//  Decompose  A = U S V_T
//  Compute U * S(diag/W) * V_T * b
//==========================================================================
void LinearSystem::Solve() {
    int  i;
    double  *x;

    if (ncol != nrow){  // Need SVD
       // First, solve to get SVD values
       imsl_d_lin_svd_gen(nrow, ncol, m, 
			  IMSL_RETURN_USER, x,
			  IMSL_U_USER, &u,
			  IMSL_V_USER, &v,
			  0);
       // Something wrong with the dimensions here!!!
       reciprocal_s = new double[MIN(nrow, ncol)]
       for (i=0; i<MIN(nrow, ncol); i++)
	   reciprocal_s[i] = 1./reciprocal_s[i];

       // Make dialgonal matrix out of the vector
       Matrix diagS(s, MIN(nrow, ncol));
       Matrix U(u, nrow, MIN(nrows, ncol));
       Matrix V(v, ncol, MIN(nrows, ncol));

       // Then, solve for the solution
       x = U * diagS * V.transpose() * rhs;
    }
    else{
       // Solve the corresponding linear system
       x = imsl_d_lin_sol_gen(nrow, ncol, m, 0);
    }

    // the dimension of the solution is "ncol"
//    double x;
}
#endif


#ifdef NUMERICAL_RECIPES
//==========================================================================
 //  Solve
//  
//  Numerical Recipe version
//
//  Linear System :  Ax = b
//  Decompose  A = U S V_T
//  Compute V * S(diag/W) * U_T * b
//==========================================================================
void LinearSystem::Solve() {
    int  i, j;

    int nrow = lhs.nrow;
    int ncol = lhs.ncol;
    if (ncol != nrow){  // Need SVD
       // For SVD
       double **u = dmatrix(0, nrow-1, 0, ncol-1);
       double *w = dvector(0, ncol-1);
//       double *sol = dvector(0, ncol-1);
       double **v = dmatrix(0, ncol-1, 0, ncol-1);
       x = new double [ncol];

       // Copy "m" into "u" 
       for (i=0; i<nrow; i++)
	   for (j=0; j<ncol; j++)
	       u[i][j] = lhs.m[i*ncol + j]; 
		
       cout << "(nrow, ncol) = " << nrow << ", " << ncol << endl;
       // First, solve to get SVD values
       svdcmp(u, nrow, ncol, w, v);

       double wmax = 0.0;
       for (j=0; j<ncol; j++)
	   if (w[j] > wmax)  wmax = w[j];
       
       double wmin = wmax*1.0e-6;
       for (j=0; j<ncol; j++)
	   if (w[j] < wmin)  w[j] = 0.0;

       svbksb(u, w, v, nrow, ncol, rhs, x);
#if 0
       double * tm = new double [ncol*ncol];       
       for (i=0; i<ncol; i++)
	   for (j=0; j<ncol; j++)
	       tm[i*ncol + j] = v[i][j] * 1.0/w[j];
       Matrix tmatrix1(tm, ncol, ncol); 
       cout << "V*w" << endl;
       tmatrix1.Print();

       double * um = new double [nrow*ncol];       
       for (i=0; i<nrow; i++)
	   for (j=0; j<ncol; j++)
	       um[i*ncol + j] = u[i][j];
       Matrix umatrix(um, nrow, ncol); 

       Matrix bmatrix(rhs, nrow, 1);
       Matrix tmatrix2 = umatrix.Transpose(); // * bmatrix;
       cout << "Transposeed" << endl;
       tmatrix2.Print();
       Matrix tmatrix3 = tmatrix1 * tmatrix2;
       Matrix tmatrix4 = tmatrix3 * bmatrix;
   
       for (i=0; i<ncol; i++)
	   x[i] = tmatrix4.m[i];
#endif

       free_dmatrix(u, 0, nrow-1, 0, ncol-1);
       free_dvector(w, 0, ncol-1);
//       free_dvector(sol, 0, ncol-1);
       free_dmatrix(v, 0, ncol-1, 0, ncol-1); 
       
#if 0
       if (tm)
	  delete [] tm;

       if (um)
	  delete [] um;
#endif
    }
    else{
       double **a = dmatrix(0, nrow-1, 0, ncol-1);
       double **b = dmatrix(0, nrow-1, 0, 0);

       // Copy "m" into "a" 
       for (i=0; i<nrow; i++)
	   for (j=0; j<ncol; j++)
	       a[i][j] = lhs.m[i*ncol + j]; 

       for (i=0; i<nrow; i++)
	   b[i][0] = rhs[i]; 

       // Solve the corresponding linear system by using the Simplex method
       gaussj(a, nrow, b, 1);

       // Get the solution
       for (i=0; i<nrow; i++)
	   x[i] = b[i][0];

       // Free local memory
       free_dmatrix(a, 0, nrow-1, 0, ncol-1);
       free_dmatrix(b, 0, nrow-1, 0, 0);
    }

    // the dimension of the solution is "ncol"
//    double x;
}
#endif


#ifdef MATLAB
//==========================================================================
//  Solve
//  
//  Numerical Recipe version
//
//  Linear System :  Ax = b
//  Decompose  A = U S V_T
//  Compute V * S(diag/W) * U_T * b
//==========================================================================
void LinearSystem::Solve() {
    int  i, j;
    Engine *ep;
    mxArray *a, *u, *s, *v, *b, *d, *w;


    int nrow = lhs.GetNrows();
    int ncol = lhs.GetNcols();
    if (ncol != nrow){  // Need SVD
       // For SVD
       a = mxCreateDoubleMatrix(nrow, ncol, mxREAL);
       u = mxCreateDoubleMatrix(nrow, nrow, mxREAL);
       s = mxCreateDoubleMatrix(ncol, nrow, mxREAL);
       v = mxCreateDoubleMatrix(ncol, ncol, mxREAL);
       w = mxCreateDoubleMatrix(ncol, ncol, mxREAL);
       d = mxCreateDoubleMatrix(ncol, 1, mxREAL);
       b = mxCreateDoubleMatrix(nrow, 1, mxREAL);

       x = new double [ncol];

       double *tmp = new double [ncol*ncol];
       double *singular = new double [ncol];

       cout << "(nrow, ncol) = " << nrow << ", " << ncol << endl;

       mxSetName(a, "A");
       mxSetName(u, "U");
       mxSetName(s, "S");
       mxSetName(v, "V");
       mxSetName(d, "D");
       mxSetName(w, "W");
       mxSetName(b, "B");

       memcpy(mxGetPr(b), rhs, nrow*sizeof(double));

       if (!( ep = engOpen("\0") ))
	  cout << "Can't start up MATLAB engine." << endl;

       engPutArray(ep, a);

       // First, solve to get SVD values
       engEvalString(ep, "[U, S, V] = svd(A)");
       u = engGetArray(ep, "U");
       s = engGetArray(ep, "S");
       v = engGetArray(ep, "V");
       w = engGetArray(ep, "W");

       engPutArray(ep, u);
       engPutArray(ep, s);
       engPutArray(ep, v);
       engPutArray(ep, w);
       engPutArray(ep, b);

       singular = mxGetPr(s);
       for (i=0; i<ncol*ncol; i++)
	   tmp[i*ncol + i] = 0.0;

       for (i=0; i<ncol; i++)
	   tmp[i*ncol + i] = 1./singular[i];

       memcpy(mxGetPr(w), tmp, ncol*ncol*sizeof(double));

       engEvalString(ep, "D = V * W * U' * b");
       d = engGetArray(ep, "D");

       engClose(ep);

       // Retrieve the solution
       x = mxGetPr(d);


       mxDestroyArray(a);
       mxDestroyArray(u);
       mxDestroyArray(s);
       mxDestroyArray(v);
       mxDestroyArray(w);
       mxDestroyArray(d);
       mxDestroyArray(b);

       mxDestroyArray(a);
       mxDestroyArray(a);
       mxDestroyArray(a);

       if (singular)
	 delete[] singular;
       if (tmp)
	 delete[] tmp;
    }
    else{
       double **a = dmatrix(0, nrow-1, 0, ncol-1);
       double **b = dmatrix(0, nrow-1, 0, 0);

       // Copy "m" into "a" 
       for (i=0; i<nrow; i++)
	   for (j=0; j<ncol; j++)
	       a[i][j] = lhs.m[i*ncol + j]; 

       for (i=0; i<nrow; i++)
	   b[i][0] = rhs[i]; 

       // Solve the corresponding linear system by using the Simplex method
       gaussj(a, nrow, b, 1);

       // Get the solution
       for (i=0; i<nrow; i++)
	   x[i] = b[i][0];

       // Free local memory
       free_dmatrix(a, 0, nrow-1, 0, ncol-1);
       free_dmatrix(b, 0, nrow-1, 0, 0);
    }

    // the dimension of the solution is "ncol"
//    double x;
}
#endif

//==========================================================================
//  PrintSolution
//==========================================================================
void LinearSystem::PrintSolution() {
    int  i;

    int ncol = lhs.GetNcols();

    cout << "Dim of solution =" << ncol << endl;

    for (i=0; i<ncol; i++){
        cout << x[i] << " ";
    }
    cout << endl;
}

