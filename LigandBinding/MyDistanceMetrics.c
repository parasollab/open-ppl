// $Id$
////////////////////////////////////////////////////
//
//  MyDistanceMetrics.c
//
//  derived class of DistanceMetrics
//
/////////////////////////////////////////////////////


#include"MyDistanceMetrics.h"
#include"BioPotentials.h"


MyDistanceMetrics::MyDistanceMetrics() : DistanceMetric() {}
MyDistanceMetrics::~MyDistanceMetrics() {}

double 
MyDistanceMetrics::Distance(Environment *env, Cfg _c1, Cfg _c2, SID _dmsetid) {
    //_dmsetid = -1; // this is very temporary.
#if 0
tmp.push_back(Vector3D(-9.5834,-1.1208,-0.5844));
tmp.push_back(Vector3D(-8.9474,-1.5598,-1.6694));
tmp.push_back(Vector3D(-10.8614,-1.1188,-0.5884));
tmp.push_back(Vector3D(-8.8034,-0.6598,0.5976));
tmp.push_back(Vector3D(-8.2054,-1.4978,1.3336));
tmp.push_back(Vector3D(-8.7194,0.573196,0.8866));
#endif
#if 0
tmp.push_back(Vector3D(-3.9252,-3.20191,-7.28318));
tmp.push_back(Vector3D(-1.3812,-4.73091,-5.54118));
tmp.push_back(Vector3D(-3.3352,-4.55291,-6.81518));
tmp.push_back(Vector3D(-3.7062,-4.83092,-5.36218));
tmp.push_back(Vector3D(-4.5492,-3.66791,-4.84818));
tmp.push_back(Vector3D(-3.9642,-2.18491,-5.75018));
tmp.push_back(Vector3D(-2.4052,-4.91891,-4.70118));
tmp.push_back(Vector3D(-1.8742,-4.52491,-6.78318));
tmp.push_back(Vector3D(-0.156204,-4.73391,-5.29318));
tmp.push_back(Vector3D(-3.2212,-2.72991,-8.49018));
tmp.push_back(Vector3D(-3.3612,-1.31591,-9.05217));
tmp.push_back(Vector3D(-4.6332,-1.04091,-9.74618));
tmp.push_back(Vector3D(-4.5272,0.125094,-10.7902));
tmp.push_back(Vector3D(-5.79719,0.288091,-11.5472));
tmp.push_back(Vector3D(-6.15119,-0.789905,-12.2202));
tmp.push_back(Vector3D(-6.70119,1.08509,-11.2602));
#endif
vector<Vector3D> tmp;
tmp.push_back(Vector3D(2.0852,8.0409,-6.0771));
tmp.push_back(Vector3D(1.41921,9.1449,-6.2581));
tmp.push_back(Vector3D(2.9602,7.8549,-5.1931));
tmp.push_back(Vector3D(2.06019,7.0249,-7.2761));
tmp.push_back(Vector3D(1.21119,7.2269,-8.1981));
tmp.push_back(Vector3D(3.23319,6.56989,-7.4331));

    if(_dmsetid == -1) { 
       vector<Vector3D> x = BioPotentials::GetCoordinatesLigandBinding(_c1, env);
       //vector<Vector3D> y = BioPotentials::GetCoordinatesLigandBinding(_c2, env);
       vector<Vector3D> y = tmp;
       return RMSD(x,y,x.size());
    } else if(_dmsetid == -2) 
       return WeightedEuclidianDistance(_c1,_c2);
    else // use the old one.
       return DistanceMetric::Distance(env,_c1,_c2, _dmsetid);
}

double 
MyDistanceMetrics::WeightedEuclidianDistance(Cfg _c1, Cfg _c2) {
    vector<double> v = (_c1-_c2).GetData();
    double wed = 0.0;
    int size = v.size();
    for(int i=0; i<size; ++i) {
	double value = v[i] > 0.5 ? (1.0-v[i])*(1.0-v[i]) : v[i]*v[i];
        double weight = i < size/2 ? (i+1)*(i+1) : (size-i)*(size-i);
	wed += weight*value;
    }
    return sqrt(wed)/size;

}

// A more general RMSD calc. takes 'weights' also.
//double MyDistanceMetrics::RMSD(const vector<Vector3D> &x, const vector<Vector3D> &y, 
//const vector<double>& weight) {

double MyDistanceMetrics::RMSD(vector<Vector3D> x, vector<Vector3D> y, 
int dim) {  // vector x will translated, so is not passed as a const, but a copy.

    // rsmd = sqrt( sum_of[(U*xn - yn)^2]/N )
    // while U is the rotation that minimize rsmd.
    
    // reference: B.Kabsch '78. Acta Cryst. (1978) A34 page 827-828
   
    // first step, remove any translation between x and y.
    if(x.size() < dim || y.size() < dim || dim <= 0) {
	cout << "Error in MyDistanceMetrics::RMSD, not enough data in vectors" << endl;
	exit(101);
    }
    int n;
#if 0
    Vector3D sumx(0,0,0), sumy(0,0,0);
    for(n=0; n<dim; ++n) {
	sumx = sumx + x[n];
	sumy = sumy + y[n];
    }
    for(n=0; n<dim; ++n) {
	x[n] = x[n] - sumx/dim;
	y[n] = y[n] - sumy/dim;
    }
#endif
  
    // now calc. E0 = 1/2*sum_of[xn^2 + yn^2]
    double E0 = 0.0;
    double R[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for(n=0; n<dim; ++n) {
	E0 += x[n].normsqr() + y[n].normsqr();
	for(int i=0; i<3; ++i) {
	   for(int j=0; j<3; ++j) 
	      R[i][j] += y[n][i]*x[n][j]; // *weight[n] if needed.
	}
    }
    E0 /= 2;

    //let matrix R~*R = { a[0]   d    e
    //                     d    a[1]  f
    //                     e     f    a[2] };
    //Now, decide this parameters.
    //using a matrix here would be clearer, simply S = R.transpose()*R; 
    Vector3D col[3];
    double a[3], d, e, f;
    double detR; // determint of R, we need its sign later.
    for(int i=0; i<3; ++i) {
	col[i] = Vector3D(R[0][i], R[1][i], R[2][i]);
	a[i] = col[i].normsqr();
    }
    d = col[0].dotProduct(col[1]);
    e = col[0].dotProduct(col[2]);
    f = col[1].dotProduct(col[2]);
    Vector3D col1X2 = col[1].crossProduct(col[2]);
    detR = col[0].dotProduct(col1X2);


    // now solve for the eigenvalues of the matrix, since we 
    // know we have three non-negative eigenvalue, we can directly
    // solve a cubic equation for the roots...
    // the equation looks like: z^3 + a2*z^2 + a1*z + a0 = 0
    // in our case, 
    double a2 = -(a[0] + a[1] + a[2]);
    double a1 = a[0]*a[1] + a[1]*a[2] + a[2]*a[0] - d*d - e*e - f*f;
    double a0 = a[0]*f*f + a[1]*e*e + a[2]*d*d - a[0]*a[1]*a[2] - 2*d*e*f;
 
    // reference for cubic equation solution: 
    // http://mathworld.wolfram.com/CubicEquation.html
    // following the symbols using there, define:
    double Q = (3*a1 - a2*a2) / 9;
    double RR = (9.0*a2*a1 - 27.0*a0 - 2*a2*a2*a2) / 54;
   
    // if our case, since we know there are three real roots, 
    // so D = Q^3 + R^2 <= 0, 
    double z1, z2, z3; 
    if(Q == 0) { // which means three identical roots, 
 	z1 = z2 = z3 = -a2/3;
    } else { // Q < 0
	double rootmq = sqrt(-Q);
	double ceta = acos(-RR/Q/rootmq);
	double cc3 = cos(ceta/3);      // = cos(ceta/3)
	double sc3 = sqrt(1-cc3*cc3);  // = sin(ceta/3)
	z1 = 2*rootmq*cc3 - a2/3;
	z2 = rootmq*(-cc3 + sc3*1.7320508) - a2/3;
	z3 = rootmq*(-cc3 - sc3*1.7320508) - a2/3;
    }
    if(z3 < 0) // small numercal error
	z3 = 0;
 	
    int sign = detR > 0 ? 1 : -1;
    double E = E0 - sqrt(z1) - sqrt(z2) - sqrt(z3)*sign;
    if(E<0) // small numercal error
	return 0;
	
    // since E = 1/2 * sum_of[(Uxn - yn)^2], so rmsd is:
    double rmsd = sqrt(E*2/dim);
 
    return rmsd;
} 
    

