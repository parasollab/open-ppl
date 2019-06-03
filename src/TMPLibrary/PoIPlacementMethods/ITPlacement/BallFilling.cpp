#include "BallFilling.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

#include <chrono> 
using namespace std::chrono; 

// To add in the RelayTest.xml 
//<PlacementMethod type="ballfilling" label="BallFilling" radius="0.8" dimension="2" step=".2" precision=".0001"/>

BallFilling::
BallFilling(MPProblem* _problem) : ITPlacementMethod(_problem) {
	m_environment = m_problem->GetEnvironment();
	m_boundary = m_environment->GetBoundary();
	CreateBalls();
}


BallFilling::
BallFilling(XMLNode& _node) : ITPlacementMethod(_node) {
  //m_label = _node.Read("label", true, "", "label for a fixed base it placement method");
  m_environment = m_problem->GetEnvironment();
	m_boundary = m_environment->GetBoundary();
	m_radius = _node.Read("radius", true, nan(""), 0., 1000., "Radius of a ball");
	m_dimension = _node.Read("dimension", true, nan(""), 0., 1000., "Number of dimensions, 2 for 2D and 3 for 3D");
	m_step = _node.Read("step", true, nan(""), 0., 1000., "");
	m_precision = _node.Read("precision", true, nan(""), 0., 1000., "");
	std::map<Vector3d, std::vector<size_t>> allObstacles = m_environment->ComputeObstacleVertexMap();

	for(auto it = allObstacles.begin(); it!= allObstacles.end(); ++it){
		size_t obstacleNbr = it->second[0];
		if (m_obstacles.find(obstacleNbr) == m_obstacles.end()){
			vector<Vector3d> coordinates;
			m_obstacles[obstacleNbr] = coordinates;
		}
		m_obstacles[obstacleNbr].push_back(it->first);
	}
	auto start = high_resolution_clock::now(); 
	CreateBalls();
	auto stop = high_resolution_clock::now(); 
	auto duration = duration_cast<microseconds>(stop - start); 
  
	  cout << "Time taken by CreateBalls: "
         << duration.count() << " microseconds " << endl;
}


void
BallFilling::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, TMPStrategyMethod* _tmpMethod){
  _solution->AddInteractionTemplate(_it);

  for(auto location : m_locations){
  	_it->GetInformation()->AddTemplateLocation(location);
  }
}

//Ball Filling Fast
void
BallFilling::
CreateBalls() {   

	Range<double> xDimension = m_boundary->GetRange(0); 
	Range<double> yDimension = m_boundary->GetRange(1);  
	auto robot = m_problem->GetRobot(0);
	vector<vector<double>> vertices = GetObstaclesVertices();
	bool isObstacle = false;
	bool moveI = false;
	double yMaxObstacle;


	for(double i = xDimension.min + m_radius; i <= xDimension.max - m_radius; i = i + m_step) {  
		for(double j = yDimension.min + m_radius; j <= yDimension.max - m_radius; j = j + m_step) {  
			yMaxObstacle = yDimension.min; 			//reset yMaxObstacle
			
			for(vector<double> it : vertices) {
				double xMin = it[0];
				double yMin = it[3];
				double rectWidth = it[2];
				double rectHeight = it[5];
				bool checkObstacle = isObstacle2D(i, j, m_radius, xMin, yMin, rectWidth, rectHeight);
				if (checkObstacle == true) {
					isObstacle = true;
					yMaxObstacle = (it[4] > yMaxObstacle) ? it[4] : yMaxObstacle; 
				}
			}

			if(isObstacle == false) {
					Cfg position({i,j,0}, robot);
				  m_locations.push_back(position);
				  j = j + m_radius * 2 - m_step;
				  moveI = true;
			}
			else {
				isObstacle = false;
				j = yMaxObstacle + m_radius - m_step + m_precision; 
			}
		}

		if(moveI == true) {
			i = i + m_radius * 2 - m_step;
			moveI = false; 
		}
	}
}



vector<vector<double>>
BallFilling::
GetObstaclesVertices() {
	vector<vector<double>> minMaxVertices;

	for(auto it = m_obstacles.begin(); it!= m_obstacles.end(); ++it) {
		double xMin = it->second[0][0];
		double xMax = it->second[it->second.size()-1][0];
		double yMin = it->second[0][1];
		double yMax = it->second[it->second.size()-1][1];
		double obstacleNum = it->first;
		double rectWidth = xMax - xMin;
		double rectHeight = yMax - yMin;

/*		for(Vector3d v3d : it->second) {
			xMin = (v3d[0] < xMin) ? v3d[0] : xMin;  
			xMax = (v3d[0] > xMax) ? v3d[0] : xMax;  
		}

		for(Vector3d v3d : it->second) {
			yMin = (v3d[1] < yMin) ? v3d[1] : yMin;  
			yMax = (v3d[1] > yMax) ? v3d[1] : yMax;  
		}*/

		vector<double> coordinates = {xMin, xMax, rectWidth, yMin, yMax, rectHeight, obstacleNum};
		minMaxVertices.push_back(coordinates);
	}

	return minMaxVertices;
}


bool
BallFilling::
isObstacle2D(double circleX, double circleY, double radius, double topLeftRectX, double topLeftRectY, double rectWidth, double rectHeight) {
	double deltaX = circleX - max(topLeftRectX, min(circleX, topLeftRectX + rectWidth));
	double deltaY = circleY - max(topLeftRectY, min(circleY, topLeftRectY + rectHeight));
	return (deltaX * deltaX + deltaY * deltaY) < (radius * radius);

}

bool
BallFilling::
checkCircleCollide(double x1, double y1, double r1, double x2, double y2, double r2){ 
     return abs((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) < (r1 + r2) * (r1 + r2);
}

//Ball Filling Slow
/*void
BallFilling::
CreateBalls() {

	Range<double> xDimension = m_boundary->GetRange(0); 
	Range<double> yDimension = m_boundary->GetRange(1);  
	auto robot = m_problem->GetRobot(0);
	vector<vector<double>> vertices = GetObstaclesVertices();
	vector<vector<double>> ITsPosition; 
	bool isObstacle = false;
	bool isCircle = false;
	double yMaxObstacle;
	double startI = xDimension.min + m_radius;
	double endI = xDimension.max - m_radius;
	double startJ = yDimension.min + m_radius;
	double endJ = yDimension.max - m_radius;


	for(double i = startI; i <= endI; i = i + m_step) {  
		for(double j = startJ; j <= endJ; j = j + m_step) {  
			yMaxObstacle = yDimension.min; 			//reset yMaxObstacle
			for(vector<double> it : vertices) {
				double xMin = it[0];
				double yMin = it[3];
				double rectWidth = it[2];
				double rectHeight = it[5];
				bool checkObstacle = isObstacle2D(i, j, m_radius, xMin, yMin, rectWidth, rectHeight);
				if (checkObstacle == true) {
					isObstacle = true;
					yMaxObstacle = (it[4] > yMaxObstacle) ? it[4] : yMaxObstacle; 
				}

			}
			
			if(isObstacle == false) {
				if (i != startI) {
					for(vector<double> it : ITsPosition) {
						bool checkCircle = checkCircleCollide(i, j, m_radius, it[0], it[1], m_radius);
						if(checkCircle == true) {
							isCircle = true;
							break;
						} 
					}
				}
				if(isCircle == false) {
					Cfg position({i,j,0}, robot);
					m_locations.push_back(position);
					vector<double> ITCoordinates = {i, j};
					ITsPosition.push_back(ITCoordinates);
					j = j + m_radius * 2 - m_step;
					if(i >= ITsPosition[0][0] + m_radius*2) {
						ITsPosition.erase(ITsPosition.begin());
					}
				}
				else {
					isCircle = false;
				}
			}
			else {
				isObstacle = false;
				j = yMaxObstacle + m_radius - m_step + m_precision; 
			}
		}
	}
}
*/


//Ball Filling test (doesn't work)
/*void
BallFilling::
CreateBalls() {

	Range<double> xDimension = m_boundary->GetRange(0); 
	Range<double> yDimension = m_boundary->GetRange(1);  
	auto robot = m_problem->GetRobot(0);
	vector<vector<double>> vertices = GetObstaclesVertices();
	vector<vector<double>> ITsPosition; 
	bool isObstacle = false;
	//bool isCircle = false;
	double yMaxObstacle;
	double startI = xDimension.min + m_radius;
	double endI = xDimension.max - m_radius;
	double startJ = yDimension.min + m_radius;
	double endJ = yDimension.max - m_radius;


	for(double i = startI; i <= endI; i = i + m_step) {  
		for(double j = startJ; j <= endJ; j = j + m_step) {  
			yMaxObstacle = yDimension.min; 			//reset yMaxObstacle
			for(vector<double> it : vertices) {
				double xMin = it[0];
				double yMin = it[3];
				double rectWidth = it[2];
				double rectHeight = it[5];
				bool checkObstacle = isObstacle2D(i, j, m_radius, xMin, yMin, rectWidth, rectHeight);
				if (checkObstacle == true) {
					isObstacle = true;
					yMaxObstacle = (it[4] > yMaxObstacle) ? it[4] : yMaxObstacle; 
				}

			}
			
			if(isObstacle == false) {
		
					Cfg position({i,j,0}, robot);
					m_locations.push_back(position);
					vector<double> ITCoordinates = {i, j};
					ITsPosition.push_back(ITCoordinates);
					j = j + m_radius * 2 - m_step;

			}
			else {
				isObstacle = false;
				j = yMaxObstacle + m_radius - m_step + m_precision; 
			}
			for(auto it : m_locations) {
				//cout << it << endl;
				bool checkCircle = checkCircleCollide(i, j, m_radius, it[0], it[1], m_radius);
				//cout << m_locations[0][0] << " " << m_locations[0][1] << endl;
				auto test = m_locations[0][1];
				cout << test << endl;
				if (i >= m_locations[0][0] + m_radius*2){
					if(checkCircle == true) {																												
						m_locations.erase(m_locations.begin());
					}
				}
			}	
		}
	}
}*/
