#include "ObstacleBased.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

ObstacleBased::
ObstacleBased(MPProblem* _problem) : PlacementMethod(_problem) {}

ObstacleBased::
ObstacleBased(XMLNode& _node) : PlacementMethod(_node) {
  m_dimensions = _node.Read("dimensions", true, nan(""), 0., 1000., "number of dimensions (2D, 3D)");
  m_precision = _node.Read("precision", true, nan(""), 0., 1000., "precision for ball filling");
  m_startCornerAngle = _node.Read("startCornerAngle", true, nan(""), 0., 1000., "start angle to check around corners"); 
  m_endCornerAngle = _node.Read("endCornerAngle", true, nan(""), 0., 1000., "end angle to check around corners");
  m_stepCornerAngle = _node.Read("stepCornerAngle", true, nan(""), 0., 1000., "angle size of step for corner check");
  m_checksPerSide = _node.Read("checksPerSide", true, nan(""), 0., 1000., "number of checks per obstacle side");
  m_refineMethod = _node.Read("refineMethod", true, nan(""), 0., 1000., "0: linear, 1: random");
  m_environment = m_problem->GetEnvironment();
  m_boundary = m_environment->GetBoundary();
  m_ballDiameter = m_ballRadius*2.0;
  // retrieves the map with the obstacles coordinates and changes it to have the obastacle
  // numbers as keys 
  std::map<Vector3d, std::vector<size_t>> obstacles = m_environment->ComputeObstacleVertexMap();

  for(auto it = obstacles.begin(); it != obstacles.end(); ++it){
  	size_t obstacleNum = it->second[0];
  	if(m_obstacles.find(obstacleNum)==m_obstacles.end()){
  		vector<Vector3d> coordinates;
  		m_obstacles[obstacleNum] = coordinates;
  	}
  	m_obstacles[obstacleNum].push_back(it->first);
  }
}

void
ObstacleBased::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, Coordinator* _coordinator) {

  m_library=_library;

  Fill();

  for(auto location : m_locations){
  	_it->GetInformation()->AddTemplateLocation(location);
  }
}


// The main function to fill the map
void
ObstacleBased::
Fill(){
	
	vector<vector<double>> possibleLocations;
	// 2D or 3D
  if(m_dimensions==2){

  	Range<double> xDimension = m_boundary->GetRange(0);
  	Range<double> yDimension = m_boundary->GetRange(1);
  	//Range<double> zDimension = m_boundary->GetRange(2);
  	cout<<xDimension<<yDimension<<"\n";
  	auto robot = m_problem->GetRobot(0);
  	m_obstaclesBounds = GetObstaclesBounds();
  	vector<vector<double>> xBounds = m_obstaclesBounds[0];
    vector<vector<double>> yBounds = m_obstaclesBounds[1];

    // start from each obstacle and tries to place ITs from there
    size_t obstacleIndex = 0;
    while(obstacleIndex<xBounds.size()){

      vector<double> xPair = xBounds[obstacleIndex];
      vector<double> yPair = yBounds[obstacleIndex];
      double xMin = xPair[0];
      double xMax = xPair[1];
      double yMin = yPair[0];
      double yMax = yPair[1];
      double distance = m_precision;

      // for the side points
      // tries to place ITs startng from the edges of the obstacles
      for(double i=0; i<2; i++){
      	for(double j=1; j<m_checksPerSide+1; j++){
	      	double newX = (xMin+xMax)/((m_checksPerSide+1)*j);
	      	double newY2 = (yMin+yMax)/((m_checksPerSide+1)*j);
	      	double shift = distance;
	      	shift = i==0 ? -shift : shift;
	      	double newY = yPair[i]+shift;
	      	double newX2 = xPair[i]+shift;
	      	vector<double> newPointX = {newX,newY};
	      	vector<double> newPointY = {newX2,newY2};

	      	bool isValidNewPointX = IsPointValid(newPointX);
	      	while(isValidNewPointX){
	      		possibleLocations.push_back(newPointX);
	      		double nX = newPointX[0];
	      		double nY = newPointX[1]+shift;
	      		vector<double> nPX = {nX,nY};
	      		isValidNewPointX = IsPointValid(nPX);
	      		newPointX = isValidNewPointX ? nPX : newPointX;
	      	}

	  			bool isValidNewPointY = IsPointValid(newPointY);
	  			while(isValidNewPointY){
	  				possibleLocations.push_back(newPointY);
	  				double nX = newPointY[0]+shift;
	  				double nY = newPointY[1];
	  				vector<double> nPY = {nX, nY};
	  				isValidNewPointY = IsPointValid(nPY);
	  				newPointY = isValidNewPointY ? nPY : newPointY; 
	  			}
      	}
    	}
  		// for the corner points
  		// tries to place ITs from the corners of the obstacles
			m_startCornerAngle = m_startCornerAngle<0 ? -m_startCornerAngle : m_startCornerAngle;
    	m_startCornerAngle = m_startCornerAngle>90 ? 90 : m_startCornerAngle;
    	m_endCornerAngle = m_endCornerAngle<0 ? -m_endCornerAngle : m_endCornerAngle;
    	m_endCornerAngle = m_endCornerAngle>90 ? 90 : m_endCornerAngle;
    	m_stepCornerAngle = m_stepCornerAngle==0 ? 1 : (m_stepCornerAngle<0 ? -m_stepCornerAngle : m_stepCornerAngle);
    	m_stepCornerAngle = m_stepCornerAngle>90 ? 90 : m_stepCornerAngle;
    	
    	if(m_startCornerAngle>m_endCornerAngle){
    		swap(m_startCornerAngle, m_endCornerAngle);
    	}

    	for(double theta=m_startCornerAngle; theta<=m_endCornerAngle; theta=theta+m_stepCornerAngle){

    		double thetaRad = theta*M_PI/180;
    		double hShift = distance*cos(thetaRad);
      	double vShift = distance*sin(thetaRad);

      	double bLX = xPair[0]-hShift;
      	double bLY = yPair[0]-vShift;
      	double tLX = xPair[0]-hShift;
      	double tLY = yPair[1]+vShift;
      	double bRX = xPair[1]+hShift;
      	double bRY = yPair[0]-vShift;
      	double tRX = xPair[1]+hShift;
      	double tRY = yPair[1]+vShift;

      	vector<double> newPointBL = {bLX,bLY};
      	vector<double> newPointTL = {tLX,tLY};
      	vector<double> newPointBR = {bRX,bRY};
      	vector<double> newPointTR = {tRX,tRY};

      	bool isValidNewPointBL = IsPointValid(newPointBL);
      	while(isValidNewPointBL){
      		possibleLocations.push_back(newPointBL);
      		double nBLX = newPointBL[0]-hShift;
      		double nBLY = newPointBL[1]-vShift;
      		vector<double> nPBL = {nBLX,nBLY};
      		isValidNewPointBL = IsPointValid(nPBL);
      		newPointBL = isValidNewPointBL ? nPBL : newPointBL;
      	}
      	bool isValidNewPointTL = IsPointValid(newPointTL);
      	while(isValidNewPointTL){
      		possibleLocations.push_back(newPointTL);
      		double nTLX = newPointTL[0]-hShift;
      		double nTLY = newPointTL[1]+vShift;
      		vector<double> nPTL = {nTLX,nTLY};
      		isValidNewPointTL = IsPointValid(nPTL);
      		newPointTL = isValidNewPointTL ? nPTL : newPointTL;
      	}
      	bool isValidNewPointBR = IsPointValid(newPointBR);
      	while(isValidNewPointBR){
      		possibleLocations.push_back(newPointBR);
      		double nBRX = newPointBR[0]+hShift;
      		double nBRY = newPointBR[1]-vShift;
      		vector<double> nPBR = {nBRX,nBRY};
      		isValidNewPointBR = IsPointValid(nPBR);
      		newPointBR = isValidNewPointBR ? nPBR : newPointBR;
      	}
      	bool isValidNewPointTR = IsPointValid(newPointTR);
      	while(isValidNewPointTR){
      		possibleLocations.push_back(newPointTR);
      		double nTRX = newPointTR[0]+hShift;
      		double nTRY = newPointTR[1]+vShift;
      		vector<double> nPTR = {nTRX,nTRY};
      		isValidNewPointTR = IsPointValid(nPTR);
      		newPointTR = isValidNewPointTR ? nPTR : newPointTR;
      	}
    	}
  		obstacleIndex++;
  	}

    vector<vector<double>> finalLocations = RefinePossibleLocations(possibleLocations);

    for(vector<double> point : finalLocations){
    	double x = point[0];
    	double y = point[1];
    	Cfg position({x,y,0}, robot);
    	m_locations.push_back(position);
    }    
  }
  else if(m_dimensions==3){

  	//TODO
  }
}

// function to retrieve the obstacles geometric coordinates
// returns the xMin/xMax and the yMin/yMax of each obstacle
// the X pairs and the Y pairs are in separate vectors
// the index of the air matches the obstacle number
vector<vector<vector<double>>>
ObstacleBased::
GetObstaclesBounds(){

  vector<vector<double>> xBounds;
  vector<vector<double>> yBounds;

  for(auto it = m_obstacles.begin(); it != m_obstacles.end(); it++){
  	
  	double xMin = it->second[0][0];
  	double xMax = it->second[it->second.size()-1][0];
  	double yMin = it->second[0][1];
  	double yMax = it->second[it->second.size()-1][1];

  	for(Vector3d v : it->second){
  	  xMin = v[0]<xMin ? v[0] : xMin;
  	  xMax = v[0]>xMax ? v[0] : xMax;
  	}
  	for(Vector3d v : it->second){
  	  yMin = v[1]<yMin ? v[1] : yMin;
  	  yMax = v[1]>yMax ? v[1] : yMax;
  	}

  	vector<double> xCoordinates = {xMin, xMax};
  	vector<double> yCoordinates = {yMin, yMax};
  	xBounds.push_back(xCoordinates);
  	yBounds.push_back(yCoordinates);
  }
  vector<vector<vector<double>>> bounds = {xBounds, yBounds};
  return bounds;
}


//Function to check if a given point is in an obstacle
bool
ObstacleBased::
IsInObstacle(vector<double> point, int method){
	
	if(method==0){
	  auto& robot = m_problem->GetRobots()[0];
	  Cfg check({point[0], point[1], 0}, robot.get());
	  auto vcm = m_library->GetValidityChecker("pqp_solid");
	  return !vcm->IsValid(check, "ValidateITCfg");
	}

	double distance = method==1 ? m_precision : 0;
  vector<vector<double>> xBounds = m_obstaclesBounds[0];
  vector<vector<double>> yBounds = m_obstaclesBounds[1];
  for(size_t index=0; index<xBounds.size(); index++){
		if(point[0]>=xBounds[index][0]-distance && point[0]<=xBounds[index][1]+distance){
			if(point[1]>=yBounds[index][0]-distance && point[1]<=yBounds[index][1]+distance){
				return true;
			}
		}
	}
	return false;
}

//Function to check if a point is valid, verifies if it's inside the environment,
// and if it's not inside an obstacle
bool
ObstacleBased::
IsPointValid(vector<double> point){
  point.push_back(0);
	if(!m_boundary->InBoundary(point)){
		return false;
	}
	if(IsInObstacle(point,0)){
		return false;
	}

	return true;
}

// Function to keep only some of the ITs
// selects only a few ITs from the big list inputed,
// 0 is linear, 1 is random
vector<vector<double>>
ObstacleBased::
RefinePossibleLocations(vector<vector<double>> possibleLocations){

	vector<vector<double>> finalLocations;

  int firstNum = m_refineMethod==0 ? 0 : rand()%possibleLocations.size();
  finalLocations.push_back(possibleLocations[firstNum]);
  possibleLocations.erase(possibleLocations.begin()+firstNum);
  int num = 0;
  while(possibleLocations.size()>0){
    num = m_refineMethod==0 ? num+num : rand()%possibleLocations.size();
    vector<double> possiblePoint = possibleLocations[num];
    bool pointValid = true;
    for(vector<double> point : finalLocations){
    	if(abs(point[0]-possiblePoint[0])<2*m_precision){
    		if(abs(point[1]-possiblePoint[1])<2*m_precision){
    			pointValid = false;
    		}
    	}
    }
    if(pointValid){
    	finalLocations.push_back(possibleLocations[num]);
    }	    
    possibleLocations.erase(possibleLocations.begin()+num);
	}
	return finalLocations;
}


void
ObstacleBased::
TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg){
  double x = _relativeCfg[0];
  double y = _relativeCfg[1];
  double theta = _centerCfg[2];

  double newX = x*cos(theta) - y*sin(theta);
  double newY = x*sin(theta) + y*cos(theta);
  double oldTheta = _relativeCfg[2];

  _relativeCfg.SetLinearPosition({newX, newY, oldTheta});
  _relativeCfg += _centerCfg;
}




