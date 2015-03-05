
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////


// Maintains a vector of size 2 with Min at the front and Max at the end
void PushMinMax(vector<double>& _vec, double _num) {
  // Empty Case
  if (_vec.size() == 0)
    _vec.push_back(_num);
  // Only one
  else if (_vec.size() == 1) {
    if (_vec.front() < _num)
      _vec.push_back(_num);
    else {
      _vec.push_back(_vec.front());
      _vec[0] = _num;
    }
  }
  // Compare and update if necessary
  else {
    if (_num < _vec[0])
      _vec[0] = _num;
    else if (_num > _vec[1])
      _vec[1] = _num;
  }
}


template<class CfgType, class Environment>
CfgType
SelectDirection(Environment* _env, CfgType dir){
  if(dir == CfgType()){
    dir.GetRandomCfg(_env);
  }else{
    CfgType r;
    r.GetRandomCfg(_env);
    dir.subtract(dir,r);
  }
  return dir;
}

// from cartesion to spherical
template<class CfgType>
vector<double>
GetSphericalCoordinates(CfgType& _cfg) {
  vector<double> coordinates(3);
  double rho = 0;   // = sqrt(x^2 + y^2 + z^2)
  double theta = 0; // = arctan(y/x)
  double phi = 0;   // = arccos(z/rho)
  // Getting cartesian coordinates
  for (size_t j = 0; j < _cfg.PosDOF(); ++j) {
    coordinates[j] = _cfg[j];
    rho += pow(_cfg[j], 2.0);
  }
  // Coordinates = [X,Y,Z]
  rho = sqrt(rho);
  theta = atan2(coordinates[1],coordinates[0]);
  phi = MAX_INT;
  if(_cfg.PosDOF() == 3) {
    phi = acos(coordinates[2] / rho);

  }

  // Lets make all angles positive for more accurate comparison between quadrants, since atan2 returns [-2/pi,2/pi]
  while(theta < 0)
    theta += TWOPI;
  // from cartesian to polar
  coordinates[0] = rho;
  coordinates[1] = theta;
  coordinates[2] = phi;

  for (int i=_cfg.PosDOF(); i<_cfg.DOF(); i++)
    coordinates.push_back(2*DRand()-1.0);

  return coordinates;
}


// Gets the point that is in between the candidate and the neighbor
template<class CfgType>
CfgType GetMiddlePoint(CfgType _regionCand, CfgType _neighbor, double _radius) {
  CfgType middlePoint;
  middlePoint = _regionCand + _neighbor;
  middlePoint = middlePoint/2;
  // Getting middle point across the circumference between candidate and neighbor
  vector<double> midPointCoordinates = GetSphericalCoordinates(middlePoint);
  midPointCoordinates[0] = _radius;
  midPointCoordinates = GetCartesianCoordinates(midPointCoordinates);

  middlePoint.SetData(midPointCoordinates);
  return middlePoint;

}


template<class CfgType>
vector<CfgType>
GetMiddlePoints(CfgType& _regionCand, vector<CfgType>& _neighbors, double _radius) {

  vector<CfgType> middlePoints;
  // Getting middle point across the circumference between candidate and neighbor
  for (size_t i = 0; i < _neighbors.size(); ++i)
    middlePoints.push_back(GetMiddlePoint(_regionCand, _neighbors[i], _radius));

  return middlePoints;

}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Random Node within a region
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*
 * Returns a random node within a region characterized by a region candidate
 * and its neighbors
 *
 * */

template<class CfgType>
CfgType
SelectDirection(CfgType& _regionCand, vector<CfgType>& _neighbors, double _radius) {

  SelectDirection(_regionCand, _neighbors, _radius, 0);
}


template<class CfgType>
CfgType
SelectDirection(CfgType& _regionCand, vector<CfgType>& _neighbors, double _radius, double _overlap) {
  CfgType dir = CfgType();

  // Store all the angles to get the max and min
  vector<double> thetas;
  vector<double> phis;

  if (dir.PosDOF() > 0) {

    vector<double> candCoordinates = GetSphericalCoordinates(_regionCand);
    vector<CfgType> midPoints = GetMiddlePoints(_regionCand, _neighbors, _radius);
    // TODO DEBUG only one neighbor, split region into halves
    if (midPoints.size() == 1) {
      CfgType point = -(midPoints[0]);
      midPoints.push_back(point);
    }

    for (size_t i = 0; i < midPoints.size(); ++i) {
      // [0] = Rho, [1] = Theta, [2] = Phi
      vector<double> neighborCoordinates;
      if(_neighbors.size() == 1)
        neighborCoordinates = GetSphericalCoordinates(_neighbors[0]);
      else
        neighborCoordinates = GetSphericalCoordinates(_neighbors[i]);

      vector<double> midPointCoordinates = GetSphericalCoordinates(midPoints[i]);
      vector<double> increments(3);

      // FIXME Increments not working, some angles are being shrinked
      // instead of enlarged
      increments[1] = (neighborCoordinates[1] - midPointCoordinates[1] ) * _overlap;
      midPointCoordinates[1] += increments[1];
      double increment = (neighborCoordinates[2] - midPointCoordinates[2]) * _overlap;
      midPointCoordinates[2] += increment;

      // We use this function to avoid pushing all angles and sorting them at the end
      PushMinMax(thetas, midPointCoordinates[1]);
      PushMinMax(phis, midPointCoordinates[2]);
    }

    // Is the range calculated correct? If cand theta is outside out [min,max] then fix the range to be [max-2PI, min]
    while (thetas[0] > candCoordinates[1] ) {
      double temp = thetas[0];
      thetas[0] = thetas[1] - TWOPI;
      thetas[1] = temp;
    }
    while (thetas[1] < candCoordinates[1]) {
      double temp = thetas[1];
      thetas[1] = thetas[0] + TWOPI;
      thetas[0] = temp;
    }
    // Randomizing
    // Rho = radius
    vector<double> randCoordinates(3);
    randCoordinates[0] = _radius  * sqrt(DRand());
    randCoordinates[1] = (thetas[1] - thetas[0]) * DRand() + thetas[0];
    randCoordinates[2] = MAX_INT;
    if(_regionCand.PosDOF() == 3)
      randCoordinates[2] = (phis[1] - phis[0]) * DRand() + phis[0];

    randCoordinates = GetCartesianCoordinates(randCoordinates);

    dir.SetData(randCoordinates);

    // Randomizing Rotational DOFs
    for (size_t i = dir.PosDOF(); i < dir.DOF(); i++)
      dir[i] = (2*DRand()-1.0);

    // TODO Fixed-Tree - I am not sure how to divide this space into regions.
  }else {
    for (size_t i=0; i < _regionCand.DOF(); i++) {
      vector<double> minMax;
      for (size_t j=0; j < _neighbors.size(); j++) {
        PushMinMax(minMax, _neighbors[j][i]);
      }
      dir[i] = (minMax.back() - minMax.front()) * DRand() + minMax.front();
    }
  }

  return dir;
}

// from spherical to cartesian
vector<double>
GetCartesianCoordinates(vector<double> sphericalCoordinates) {
  vector<double> coordinates(2);
  double rho = sphericalCoordinates[0];
  double theta = sphericalCoordinates[1];
  double phi = sphericalCoordinates[2];

  // from cartesian to polar
  coordinates[0] = rho * cos(theta) ;
  coordinates[1] = rho * sin(theta) ;

  // 3D case
  if(phi != MAX_INT) {
    coordinates[0] *= sin(phi) ;
    coordinates[1] *= sin(phi) ;
    coordinates.push_back(rho * cos(phi));
    for (int i=0; i<3; i++)
      coordinates.push_back(2*DRand()-1.0);
  }
  return coordinates;
}


