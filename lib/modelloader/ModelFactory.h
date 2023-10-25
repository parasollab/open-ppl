#ifndef _MODEL_FACTORY_H_
#define _MODEL_FACTORY_H_

#include <string>
#include <vector>

#include "Vector.h"

class IModel;
class CBVHDataModelFactory;

IModel*
CreateModelLoader(const std::string& _filename, bool _silent = false);

std::vector<IModel*>&
CreateModelLoaderVec(CBVHDataModelFactory* _modelFactory,
    const std::string& _filename, double _radius, double _height);

IModel*
CreateModelLoaderFromPts(std::vector<mathtool::Point2d>& boundary);

#endif
