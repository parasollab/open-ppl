#include "Body.h"
#include "Connection.h"
#include "MultiBody.h"

#include "Utilities/URDFParser.h"

#include <urdf/model.h>
/*----------------------------------- Body -----------------------------------*/

void
Body::
TranslateURDFLink(const std::shared_ptr<const urdf::Link>& _link,
                  const bool _base) {


  // Check if body is virtual
  const auto collision = _link->collision;
  if(!collision) {
    m_virtual = true;
  }
  // Translate geometric elements.
  else if(collision->geometry->type == urdf::Geometry::MESH) {
    const auto geometry = collision->geometry.get();

    const auto mesh = dynamic_cast<urdf::Mesh*>(geometry);

    auto fullROSPath = mesh->filename;

    if(fullROSPath.empty())
      throw RunTimeException(WHERE) << "ROS geometry filename is empty." << std::endl;
    else if(fullROSPath[0] == '/')
      m_filename = fullROSPath;
    else {

      size_t sl = fullROSPath.find("//");
      auto packagePath = fullROSPath.substr(sl+2,fullROSPath.size()-1);

      sl = packagePath.find("/");
      m_filename = packagePath.substr(sl+1,packagePath.size()-1);
    }

    // Read the mesh file.
    //ReadGeometryFile(m_comAdjust);
    ReadGeometryFile(GMSPolyhedron::COMAdjust::None);

    // Apply mesh scaling
    auto scale = mesh->scale;

    auto newCenter = m_polyhedron.GetCentroid();
    newCenter[0] = newCenter[0]*scale.x;
    newCenter[1] = newCenter[1]*scale.y;
    newCenter[2] = newCenter[2]*scale.z;

    m_polyhedron.Scale(scale.x,scale.y,scale.z,newCenter);

  }
  else if(collision->geometry->type == urdf::Geometry::BOX) {
    // Convert Box to GMSPolyhedron m_plohedron
    const auto geometry = collision->geometry.get();

    auto box = dynamic_cast<urdf::Box*>(geometry);
    auto x = box->dim.x;
    auto y = box->dim.y;
    auto z = box->dim.z;

    Range<double> xRange(-x/2,x/2);
    Range<double> yRange(-y/2,y/2);
    Range<double> zRange(-z/2,z/2);

    m_polyhedron = GMSPolyhedron::MakeBox(xRange,yRange,zRange);
    ComputeMomentOfInertia();
    ComputeBoundingBox();
    MarkDirty();
    //Validate();
  }
  else if(collision->geometry->type == urdf::Geometry::CYLINDER) {
    const auto geometry = collision->geometry.get();
    auto cyl = dynamic_cast<urdf::Cylinder*>(geometry);

    Range<double> height;
    height.min = -cyl->length/2;
    height.max = cyl->length/2;

    double radius = cyl->radius;

    size_t fidelity = 33;

    m_polyhedron = GMSPolyhedron::MakeCylinder(height, radius, fidelity);
  }
  else {
    throw RunTimeException(WHERE) << "Unsupported ROS Geometry." << std::endl;
  }

  // Translate visual elements.
  if(_link->visual and _link->visual->material) {
    const auto material = _link->visual->material;
    const auto color = material->color;
    m_color = glutils::color(color.a, color.b, color.g, color.r);
    m_textureFile = material->texture_filename;
  }


  // Determine if base link.
  const auto parent = _link->getParent();
  if(!_base) {
    SetMovementType(MovementType::Joint);
    SetBodyType(Type::Joint);
  }
  else {
    if(m_virtual)
      throw RunTimeException(WHERE) << "Unsupported behavior with virtual link being base of robot.";
  }

}

/*-------------------------------- Connection --------------------------------*/

void
Connection::
TranslateURDFJoint(const std::shared_ptr<urdf::Joint>& _joint,
                   const std::pair<size_t,size_t> _bodyIndices,
                   const Vector3d _bodyPosition,
                   const MatrixOrientation _bodyOrientation) {

  // body indices
  m_bodyIndices = _bodyIndices;

  m_jointParamType=JointParamType::URDF;

  // grab the joint type
  switch(_joint->type) {
    case urdf::Joint::REVOLUTE :
      m_jointType = Connection::JointType::Revolute;
      break;
    case urdf::Joint::FIXED :
      m_jointType = Connection::JointType::NonActuated;
      break;
    case urdf::Joint::PRISMATIC :
      m_jointType = Connection::JointType::Prismatic;
      break;
    default :
      throw RunTimeException(WHERE) << "Unsupported joint type."
                                    << std::endl;
  }

  // grab the joint limits for revolute and spherical joints
  if(IsRevolute() || IsSpherical() || IsPrismatic()) {
    m_jointRange[0].min = m_jointRange[1].min = -1;
    m_jointRange[0].max = m_jointRange[1].max = 1;
    size_t numRange = IsSpherical() ? 2 : 1;

    auto limits = _joint->limits;

    for(size_t i = 0; i < numRange; ++i) {
      //m_jointRange[i].min = limits->lower/PI;
      //m_jointRange[i].max = limits->upper/PI;
      double lower = limits->lower/PI;
      double upper = limits->upper/PI;
      m_jointRange[i].min = std::max(-1.0,lower);
      m_jointRange[i].max = std::min(1.0,upper);
      m_jointValues.push_back(0);
    }
  }

  auto& transform = _joint->parent_to_joint_origin_transform;
  Vector3d position = {transform.position.x,
                       transform.position.y,
                       transform.position.z};

  Quaternion quaternion(transform.rotation.w,
                        {transform.rotation.x,
                        transform.rotation.y,
                        transform.rotation.z});

  MatrixOrientation orientation(quaternion);

  //transformation to Child Frame
  m_transformationToChildFrame =  Transformation(position,
                                                 orientation);

  //transformation to next body
  m_transformationToBody2 = Transformation(_bodyPosition,
                                          _bodyOrientation);

  if(IsRevolute())
    m_jointAxis = {_joint->axis.x,_joint->axis.y,_joint->axis.z};

  if(IsPrismatic())
    m_jointAxis = {_joint->axis.x,_joint->axis.y,_joint->axis.z};

  if(_joint->mimic) {
    m_mimicConnectionName = _joint->mimic->joint_name;
    m_mimicMultiplier = _joint->mimic->multiplier;
    m_mimicOffset = _joint->mimic->offset;
    if(m_jointType != Connection::JointType::Prismatic) {
      m_mimicOffset = m_mimicOffset / PI;
    }
    m_jointType = Connection::JointType::Mimic;
  }

}

/*--------------------------------- MultiBody --------------------------------*/

void
MultiBody::
TranslateURDF(std::string _filename,std::string _worldLink, Body::Type
    _baseType, Body::MovementType _baseMovement) {
  // Parse the urdf.
  urdf::Model model = ParseURDF(_filename);

  // Set the model direcotry for body geometries
  size_t sl = _filename.rfind("urdf/");
  auto modelDir = _filename.substr(0,sl);
  Body::m_modelDataDir = modelDir;


  // Collect topological ordering of links because the
  // urdf model has them in a different order than pmpl
  std::vector<std::shared_ptr<urdf::Link>> links;
  model.getLinks(links);
  const auto& joints = model.joints_;

  std::unordered_map<std::string, std::vector<std::string>> childMap;
  std::unordered_map<std::string, std::string> parentMap;

  for(size_t i = 0; i < links.size(); i++) {
    parentMap[links[i]->name] = "";
  }

  for(auto j : joints) {
    childMap[j.second->parent_link_name].push_back(j.second->child_link_name);
    parentMap[j.second->child_link_name] = j.second->parent_link_name;
  }

  // Identify base link/body
  std::string baseName;
  for(auto kv : parentMap) {
    if(kv.second != "" and kv.second != _worldLink)
      continue;
    if(kv.first == _worldLink)
      continue;
    if(!model.getLink(kv.first)->collision.get())
      continue;
    baseName = kv.first;
  }


  // Extract bodies
  std::unordered_map<std::string,size_t> linkMap;

  size_t count = 0;

  AddURDFLink(baseName, count, model, linkMap, childMap, true);

  // AddURDFLinke assigns a "fixed" bodyType value by default, here we override that in
  // case the base is not fixed
  auto base = GetBase();
  base->SetBodyType(_baseType);
  base->SetMovementType(_baseMovement);
  SetBaseType(_baseType);
  SetBaseMovementType(_baseMovement);

  // Extract joints

  std::unordered_map<std::string,Connection*> jointNameMap;

  for(const auto& joint : joints) {
    if(joint.second->parent_link_name == _worldLink)
      continue;

    // Check that both links are inside the linkMap - sometimes thing specified before the world link
    auto iter = linkMap.find(joint.second->parent_link_name);
    if(iter == linkMap.end())
      continue;

    iter = linkMap.find(joint.second->child_link_name);

    // Check if links in joint have physical geometries or are virtual
    /*if(!model.getLink(joint.second->parent_link_name)->collision.get()
       or !model.getLink(joint.second->child_link_name)->collision.get()) {

      std::cout << joint.second->name << std::endl;
      std::cout << joint.second->parent_link_name << " " << model.getLink(joint.second->parent_link_name)->collision.get() << std::endl;
      std::cout << joint.second->child_link_name << " " << model.getLink(joint.second->child_link_name)->collision.get() << std::endl;

      continue;
    }*/

    // add connection info to multibody connection map
    m_joints.emplace_back(new Connection(this));

    //body indices
    auto bodyIndices = std::make_pair(
                          linkMap.at(joint.second->parent_link_name),
                          linkMap.at(joint.second->child_link_name));

    Vector3d position = {0,0,0};
    MatrixOrientation orientation;

    if(model.getLink(joint.second->child_link_name)->collision) {

      auto bodyTransform = model.getLink(
          joint.second->child_link_name)->collision->origin;

      position = Vector3d({bodyTransform.position.x,
        bodyTransform.position.y,
        bodyTransform.position.z});

      Quaternion quaternion(bodyTransform.rotation.w,
          {bodyTransform.rotation.x,
          bodyTransform.rotation.y,
          bodyTransform.rotation.z});

      orientation = MatrixOrientation(quaternion);

    }

    m_joints.back()->TranslateURDFJoint(joint.second,bodyIndices,
        position,orientation);

    m_joints.back()->SetBodies();

    jointNameMap[joint.second->name] = m_joints.back().get();
  }

  // Connect all the mimic joints
  for(auto& joint : m_joints) {
    if(!joint->IsMimic())
      continue;

    auto name = joint->GetMimicConnectionName();
    joint->SetMimicConnection(jointNameMap[name]);
  }
}

void
MultiBody::
AddURDFLink(std::string _name, size_t& _count,
            urdf::Model& _model,
            std::unordered_map<std::string,size_t>& _linkMap,
            std::unordered_map<std::string,std::vector<std::string>>& _childMap,
            bool _base) {

  auto link = _model.getLink(_name);

  // Check if the link has a physical geometry or is virtual
  //if(!link->collision.get()) {
  //  _count--;
  //  return;
  //}

  const size_t index = AddBody(Body(this, _count));
  auto free = GetBody(index);


  free->TranslateURDFLink(link,_base);

  _linkMap[link->name] = index;

  if(_base && free->IsBase()) {
    m_baseIndex = index;
    SetBaseBody(m_baseIndex);
  }

  for(auto child : _childMap[_name]) {
    AddURDFLink(child, ++_count, _model, _linkMap, _childMap);
  }
}

/*----------------------------------------------------------------------------*/
