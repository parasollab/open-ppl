#include <Kripke/Directions.h>
#include <Kripke/Input_Variables.h>
#include <Kripke/User_Data.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>

namespace {
  /*
    GaussLegendre returns the n point Gauss-Legendre quadrature rule for
    the integral between x1 and x2.
  */
  void GaussLegendre(double x1, double x2, std::vector<double> &x,
      std::vector<double> &w, double eps)
  {
    int n = x.size();
    int m, j, i;
    double z1, z, xm, xl, pp, p3, p2, p1;

    m=(n+1)/2;
    xm=0.5*(x2+x1);
    xl=0.5*(x2-x1);
    for(i=1; i<=m; i++){
      z=cos(M_PI*(i-0.25)/(n+0.5));
      do {
        p1=1.0;
        p2=0.0;
        for(j=1; j<=n; j++){
          p3=p2;
          p2=p1;
          p1=((2.0*j-1.0)*z*p2-(j-1.0)*p3)/j;
        }
        pp=n*(z*p1-p2)/(z*z-1.0);
        z1=z;
        z=z1-p1/pp;
      } while(fabs(z-z1) > eps);
      x[i-1]=xm-xl*z;
      x[n-i]=xm+xl*z;
      w[i-1]=2.0*xl/((1.0-z*z)*pp*pp);

      w[n-i]=w[i-1];
    }
  }


  bool dirSortFcn(Directions const &a, Directions const &b){
    return b.octant < a.octant;
  }
}

/**
 * Initializes the quadrature set information for a User_Data object.
 * This guarantees that each <GS,DS> pair have a single originating octant.
 */
void InitDirections(User_Data *user_data, Input_Variables *input_vars)
{
  Grid_Data_Base *grid_data = user_data->grid_data;
  std::vector<Directions> &directions = user_data->directions;

  int num_directions = input_vars->num_directions;
  directions.resize(num_directions);

  int num_polar = input_vars->quad_num_polar;
  int num_azimuth = input_vars->quad_num_azimuthal;

  if (num_polar == 0) {
    // Do (essentialy) an S2 quadrature.. but with repeated directions
    int id, jd, kd;
    int in, ip, jn, jp, kn, kp;

    int num_directions_per_octant = input_vars->num_directions/8;

    // Compute x,y,z cosine values
    double mu  = cos(M_PI/4);
    double eta = sqrt(1-mu*mu) * cos(M_PI/4);
    double xi  = sqrt(1-mu*mu) * sin(M_PI/4);

    int d = 0;
    for(int octant = 0;octant < 8;++ octant){
      double omegas[3];
      omegas[0] = octant & 0x1;
      omegas[1] = (octant>>1) & 0x1;
      omegas[2] = (octant>>2) & 0x1;

      for(int sd=0; sd<num_directions_per_octant; sd++, d++){
        // Store which logical direction of travel we have
        directions[d].id = (omegas[0] > 0.) ? 1 : -1;
        directions[d].jd = (omegas[1] > 0.) ? 1 : -1;
        directions[d].kd = (omegas[2] > 0.) ? 1 : -1;

        // Store quadrature point's weight
        directions[d].w = 4.0*M_PI / (double)num_directions;
        directions[d].xcos = mu;
        directions[d].ycos = eta;
        directions[d].zcos = xi;
      }
    }

    /**
     * In reference v1.1, the octant is not calculated for S2 quadrature,
     * and the i_src_subd, j_src_subd, k_src_subd, i_dst_subd, j_dst_subd,
     * k_dst_subd are computed in Layout.
     * TODO: move the code below to a more suitable place if needed.
     */
    in = grid_data->mynbr()[0][0];
    ip = grid_data->mynbr()[0][1];
    jn = grid_data->mynbr()[1][0];
    jp = grid_data->mynbr()[1][1];
    kn = grid_data->mynbr()[2][0];
    kp = grid_data->mynbr()[2][1];

    for(d=0; d<num_directions; d++){
      id = directions[d].id;
      jd = directions[d].jd;
      kd = directions[d].kd;

      directions[d].octant = 0;
      if(id == -1){
        directions[d].octant += 1;
      }
      if(jd == -1){
        directions[d].octant += 2;
      }
      if(kd == -1){
        directions[d].octant += 4;
      }

      directions[d].i_src_subd = (id>0) ? in : ip;
      directions[d].j_src_subd = (jd>0) ? jn : jp;
      directions[d].k_src_subd = (kd>0) ? kn : kp;
      directions[d].i_dst_subd = (id>0) ? ip : in;
      directions[d].j_dst_subd = (jd>0) ? jp : jn;
      directions[d].k_dst_subd = (kd>0) ? kp : kn;
    }
  }
  else {
    std::vector<double> polar_cos;
    std::vector<double> polar_weight;

    // make sure the user specified the correct number of quadrature points
    if(num_polar % 4 != 0){
      printf("Must have number of polar angles be a multiple of 4\n");
      std::exit(1);
    }
    if(num_azimuth % 2 != 0){
      printf("Must have number of azimuthal angles be a multiple of 2\n");
      std::exit(1);
    }
    if(num_polar*num_azimuth != num_directions){
      printf("You need to specify %d total directions, not %d\n",
          num_polar*num_azimuth, num_directions);
      std::exit(1);
    }

    // Compute gauss legendre weights
    polar_cos.resize(num_polar);
    polar_weight.resize(num_polar);
    GaussLegendre(-1.0, 1.0, polar_cos, polar_weight, DBL_EPSILON);

    // compute azmuhtal angles and weights
    std::vector<double> az_angle(num_azimuth);
    std::vector<double> az_weight(num_azimuth);
    double dangle = 2.0*M_PI/((double) num_azimuth);

    for(int i=0; i<num_azimuth; i++){
      if(i == 0){
        az_angle[0] = dangle/2.0;
      }
      else{
        az_angle[i] = az_angle[i-1] + dangle;
      }
      az_weight[i] = dangle;
    }

    // Loop over polar 'octants
    int d = 0;
    for(int i=0; i< num_polar; i++){
      for(int j=0; j< num_azimuth; j++){
        double xcos = sqrt(1.0-polar_cos[i]*polar_cos[i]) * cos(az_angle[j]);
        double ycos = sqrt(1.0-polar_cos[i]*polar_cos[i]) * sin(az_angle[j]);
        double zcos = polar_cos[i];
        double w = polar_weight[i]*az_weight[j];

        directions[d].id = (xcos > 0.) ? 1 : -1;
        directions[d].jd = (ycos > 0.) ? 1 : -1;
        directions[d].kd = (zcos > 0.) ? 1 : -1;

        directions[d].octant = 0;
        if(directions[d].id == -1){
          directions[d].octant += 1;
        }
        if(directions[d].jd == -1){
          directions[d].octant += 2;
        }
        if(directions[d].kd == -1){
          directions[d].octant += 4;
        }

        directions[d].xcos = std::abs(xcos);
        directions[d].ycos = std::abs(ycos);
        directions[d].zcos = std::abs(zcos);
        directions[d].w = w;

        ++d;
      }
    }

    // Sort by octant.. so each set has same directions
    std::sort(directions.begin(), directions.end(), dirSortFcn);
  }
}




